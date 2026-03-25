#!/usr/bin/env python3
"""
maze_ros2_node.py — Central coordination node for aerial-assisted maze navigation.

Coordinates a drone (hovering observer) and one Go2 dog (maze navigator).
The drone climbs to 10.5m, photographs the maze from above, the vision pipeline
builds an occupancy grid, A* finds the shortest path, and the dog follows it.

State machine:
    INIT -> ARMING -> CLIMBING -> WAITING_HOVER -> PERCEIVING -> PLANNING
         -> GUIDING_DOG -> SUCCESS / FAILURE
                             ^
                         DOG_STUCK (re-enter PERCEIVING from current cell)

Follows the structure of leader_follower_ros2_node.py exactly.

Usage:
    ./run_maze.sh node
    or directly:
    python maze_ros2_node.py
"""

import sys
import os
import csv
import math
import time
from datetime import datetime
from enum import Enum, auto

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from nav_msgs.msg import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool, Trigger

try:
    from cv_bridge import CvBridge
    _CV_BRIDGE_AVAILABLE = True
except ImportError:
    _CV_BRIDGE_AVAILABLE = False
    print("[MazeNode] WARNING: cv_bridge not available; camera frames will be skipped.")

# Project imports
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from maze_generator import (
    generate_maze_grid, get_occupancy_grid, get_cell_center_world,
    ROWS, COLS,
)
from maze_astar import MazeAstar
from maze_vision import MazeVisionNode
from maze_dog_controller import MazeDogController


# ---------------------------------------------------------------------------
# State machine
# ---------------------------------------------------------------------------
class MissionState(Enum):
    INIT          = auto()
    ARMING        = auto()
    CLIMBING      = auto()
    WAITING_HOVER = auto()
    PERCEIVING    = auto()
    PLANNING      = auto()
    GUIDING_DOG   = auto()
    DOG_STUCK     = auto()
    SUCCESS       = auto()
    FAILURE       = auto()


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def quat_to_yaw(x, y, z, w):
    """Extract yaw from quaternion."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def world_pos_to_cell(x, y, rows=ROWS, cols=COLS):
    """Convert world (x, y) to nearest maze cell (row, col)."""
    half_w = (cols - 1) / 2.0
    half_h = (rows - 1) / 2.0
    col = int(round(x + half_w))
    row = int(round(y + half_h))
    row = max(0, min(rows - 1, row))
    col = max(0, min(cols - 1, col))
    return (row, col)


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------
class MazeMissionNode(Node):

    DRONE_HOVER_ALT     = 20.0   # m — target altitude for perception (covers full 12m×12m maze)
    HOVER_ALT_THRESHOLD = 0.95   # fraction of target that counts as "reached"
    HOVER_DWELL_SEC     = 5.0    # seconds to dwell at altitude before PERCEIVING
    PERCEIVE_FRAMES     = 10     # number of camera frames to accumulate
    PERCEIVE_TIMEOUT    = 30.0   # seconds before PERCEIVING gives up
    CONTROL_HZ          = 10.0   # Hz for main timer

    def __init__(self):
        super().__init__('maze_mission_node')

        # Ground-truth occupancy grid (from maze_generator — used for IoU baseline)
        self._gt_grid = None

        # Vision
        self.vision = MazeVisionNode(drone_altitude=self.DRONE_HOVER_ALT)
        if _CV_BRIDGE_AVAILABLE:
            self._bridge = CvBridge()

        # Planner — initialized once we have the grid
        self.planner = None

        # Dog controller
        self.dog_ctrl = MazeDogController()

        # ---- State ----
        self.state = MissionState.INIT
        self.state_enter_time = None
        self.mission_start_time = None
        self._perceive_retry_count = 0
        self._max_perceive_retries = 3

        # ---- Odometry ----
        self.drone_pos = None
        self.drone_odom_received = False
        self.dog_pos = None
        self.dog_yaw = 0.0
        self.dog_odom_received = False

        # ---- Service futures ----
        self._arm_future = None
        self._takeoff_future = None

        # ---- Mission metrics ----
        self.wp_reached = 0
        self.wp_total = 0
        self.stuck_count = 0
        self.vision_iou = None
        self.collision_count = 0  # placeholder (requires contact sensor topic)

        # ---- ROS2 setup ----
        qos = QoSProfile(depth=10)
        image_qos = QoSProfile(depth=1)
        image_qos.reliability = ReliabilityPolicy.BEST_EFFORT

        # Subscribers
        self.create_subscription(Odometry, '/drone/odom', self._drone_odom_cb, qos)
        self.create_subscription(Odometry, '/robot0/odom', self._dog_odom_cb, qos)
        self.create_subscription(Image, '/drone/front_cam/rgb', self._camera_cb, image_qos)

        # Publishers
        self.dog_cmd_pub = self.create_publisher(Twist, '/robot0/cmd_vel', qos)
        self.drone_cmd_pub = self.create_publisher(PoseStamped, '/drone/cmd_position', qos)
        self.occ_grid_pub = self.create_publisher(OccupancyGrid, '/maze/occupancy_grid', qos)
        self.path_pub = self.create_publisher(Path, '/maze/planned_path', qos)

        # Service clients
        self.arm_client = self.create_client(SetBool, '/drone/arm')
        self.takeoff_client = self.create_client(Trigger, '/drone/takeoff')

        # Control timer
        self.timer = self.create_timer(1.0 / self.CONTROL_HZ, self._control_loop)

        # CSV logger
        log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'logs')
        os.makedirs(log_dir, exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(log_dir, f'maze_{ts}.csv')
        self._csv_file = open(self.csv_path, 'w', newline='')
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow([
            't', 'state',
            'dog_x', 'dog_y', 'drone_z',
            'wp_idx', 'wp_total',
            'vel_x', 'ang_z',
            'dist_to_wp',
        ])

        self.get_logger().info('MazeMissionNode initialized. Waiting for odometry...')
        self.get_logger().info(f'CSV log: {self.csv_path}')

    # ------------------------------------------------------------------
    # Odometry callbacks
    # ------------------------------------------------------------------
    def _drone_odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self.drone_pos = np.array([p.x, p.y, p.z])
        if not self.drone_odom_received:
            self.drone_odom_received = True
            self.get_logger().info(f'Drone odom: ({p.x:.2f}, {p.y:.2f}, {p.z:.2f})')

    def _dog_odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self.dog_pos = np.array([p.x, p.y, p.z])
        q = msg.pose.pose.orientation
        self.dog_yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        if not self.dog_odom_received:
            self.dog_odom_received = True
            self.get_logger().info(f'Dog odom: ({p.x:.2f}, {p.y:.2f}, {p.z:.2f})')

    def _camera_cb(self, msg: Image):
        """Accumulate camera frames during PERCEIVING state."""
        if self.state != MissionState.PERCEIVING:
            return
        if not _CV_BRIDGE_AVAILABLE:
            return
        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            rgb = np.array(cv_image)
            self.vision.add_frame(rgb)
            if self.vision.frame_count() % 5 == 0:
                self.get_logger().info(
                    f'[PERCEIVING] Frames accumulated: {self.vision.frame_count()}/{self.PERCEIVE_FRAMES}')
        except Exception as e:
            self.get_logger().warn(f'Camera callback error: {e}')

    # ------------------------------------------------------------------
    # Command helpers
    # ------------------------------------------------------------------
    def _publish_drone_hover(self):
        """Command drone to hover at its current XY at DRONE_HOVER_ALT."""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        if self.drone_pos is not None:
            msg.pose.position.x = float(self.drone_pos[0])
            msg.pose.position.y = float(self.drone_pos[1])
        msg.pose.position.z = float(self.DRONE_HOVER_ALT)
        msg.pose.orientation.w = 1.0
        self.drone_cmd_pub.publish(msg)

    def _stop_dog(self):
        self.dog_cmd_pub.publish(Twist())

    def _publish_dog_twist(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.dog_cmd_pub.publish(msg)

    def _publish_occupancy_grid(self, occ):
        """Publish 13×13 occupancy grid as nav_msgs/OccupancyGrid."""
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.info.resolution = 0.5   # each cell in expanded grid ~ 0.5m
        msg.info.width = occ.shape[1]
        msg.info.height = occ.shape[0]
        msg.info.origin.position.x = -3.25
        msg.info.origin.position.y = -3.25
        msg.info.origin.orientation.w = 1.0
        msg.data = [int(v) * 100 for v in occ.flatten()]
        self.occ_grid_pub.publish(msg)

    def _publish_planned_path(self, waypoints):
        """Publish waypoints as nav_msgs/Path."""
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        for wp in waypoints:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = float(wp[0])
            ps.pose.position.y = float(wp[1])
            ps.pose.position.z = float(wp[2])
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.path_pub.publish(msg)

    # ------------------------------------------------------------------
    # State machine helpers
    # ------------------------------------------------------------------
    def _transition(self, new_state):
        self.get_logger().info(f'State: {self.state.name} -> {new_state.name}')
        self.state = new_state
        self.state_enter_time = time.monotonic()

    def _time_in_state(self):
        if self.state_enter_time is None:
            return 0.0
        return time.monotonic() - self.state_enter_time

    def _log_csv(self, vel_x=0.0, ang_z=0.0, dist_to_wp=0.0):
        t = 0.0
        if self.mission_start_time is not None:
            t = time.monotonic() - self.mission_start_time
        dx = self.dog_pos[0] if self.dog_pos is not None else 0.0
        dy = self.dog_pos[1] if self.dog_pos is not None else 0.0
        dz = self.drone_pos[2] if self.drone_pos is not None else 0.0
        self._csv_writer.writerow([
            f'{t:.3f}',
            self.state.name,
            f'{dx:.3f}', f'{dy:.3f}', f'{dz:.3f}',
            self.dog_ctrl.current_waypoint_index(),
            self.dog_ctrl.total_waypoints(),
            f'{vel_x:.3f}', f'{ang_z:.3f}',
            f'{dist_to_wp:.3f}',
        ])

    # ------------------------------------------------------------------
    # PERCEIVING helpers
    # ------------------------------------------------------------------
    def _start_perceiving(self):
        self.vision.clear_buffer()
        self._transition(MissionState.PERCEIVING)

    def _get_start_cell_from_dog(self):
        """Return the maze cell nearest to the dog's current position."""
        if self.dog_pos is None:
            return (0, 0)
        return world_pos_to_cell(self.dog_pos[0], self.dog_pos[1])

    # ------------------------------------------------------------------
    # Main control loop (10 Hz)
    # ------------------------------------------------------------------
    def _control_loop(self):

        # ---- INIT ----
        if self.state == MissionState.INIT:
            if self.drone_odom_received and self.dog_odom_received:
                self.get_logger().info('Odometry received. Starting mission.')
                self._transition(MissionState.ARMING)
            return

        # ---- ARMING ----
        if self.state == MissionState.ARMING:
            if self._arm_future is None:
                if not self.arm_client.service_is_ready():
                    if self._time_in_state() > 15.0:
                        self.get_logger().error('/drone/arm not available after 15s')
                    return
                req = SetBool.Request()
                req.data = True
                self._arm_future = self.arm_client.call_async(req)
                self.get_logger().info('Arming drone...')
            elif self._arm_future.done():
                result = self._arm_future.result()
                if result is not None and result.success:
                    self.get_logger().info(f'Drone armed: {result.message}')
                    self.mission_start_time = time.monotonic()
                    self._transition(MissionState.CLIMBING)
                else:
                    msg = result.message if result else 'no response'
                    self.get_logger().warn(f'Arm failed: {msg}. Retrying...')
                    self._arm_future = None
            return

        # ---- CLIMBING ----
        if self.state == MissionState.CLIMBING:
            if self._takeoff_future is None:
                if not self.takeoff_client.service_is_ready():
                    return
                self._takeoff_future = self.takeoff_client.call_async(Trigger.Request())
                self.get_logger().info('Takeoff requested...')
            elif self._takeoff_future.done():
                result = self._takeoff_future.result()
                if result is not None and result.success:
                    self.get_logger().info(f'Takeoff OK: {result.message}')
                    self._transition(MissionState.WAITING_HOVER)
                else:
                    msg = result.message if result else 'no response'
                    self.get_logger().warn(f'Takeoff failed: {msg}. Retrying...')
                    self._takeoff_future = None
            self._publish_drone_hover()
            return

        # ---- WAITING_HOVER ----
        if self.state == MissionState.WAITING_HOVER:
            self._publish_drone_hover()
            if self.drone_pos is not None:
                alt = self.drone_pos[2]
                alt_ok = alt >= self.DRONE_HOVER_ALT * self.HOVER_ALT_THRESHOLD
                dwell_ok = self._time_in_state() >= self.HOVER_DWELL_SEC
                if alt_ok and dwell_ok:
                    self.get_logger().info(
                        f'Hover stable at {alt:.2f}m. Starting PERCEIVING.')
                    self._start_perceiving()
            return

        # ---- PERCEIVING ----
        if self.state == MissionState.PERCEIVING:
            self._publish_drone_hover()
            self._stop_dog()

            frames = self.vision.frame_count()
            elapsed = self._time_in_state()

            if frames >= self.PERCEIVE_FRAMES:
                self.get_logger().info(
                    f'PERCEIVING complete ({frames} frames). Running median filter...')
                grid = self.vision.get_median_grid()
                if grid is None:
                    self.get_logger().error('Median grid is None — this should not happen.')
                    self._transition(MissionState.FAILURE)
                    return

                # IoU vs ground truth (if available)
                if self._gt_grid is not None:
                    self.vision_iou = self.vision.compute_iou(grid, self._gt_grid)
                    self.get_logger().info(f'Vision IoU vs ground truth: {self.vision_iou:.3f}')
                else:
                    self.get_logger().info('No ground-truth grid set; skipping IoU.')

                self._publish_occupancy_grid(grid)
                self.vision.clear_buffer()

                # Store grid and move to PLANNING
                self._current_occ = grid
                self._transition(MissionState.PLANNING)

            elif elapsed > self.PERCEIVE_TIMEOUT:
                self.get_logger().warn(
                    f'PERCEIVING timed out ({elapsed:.1f}s). Frames={frames}. '
                    f'Retry {self._perceive_retry_count}/{self._max_perceive_retries}')
                self._perceive_retry_count += 1
                if self._perceive_retry_count > self._max_perceive_retries:
                    self.get_logger().error('Too many PERCEIVING retries. FAILURE.')
                    self._transition(MissionState.FAILURE)
                else:
                    self.vision.clear_buffer()
                    self._transition(MissionState.PERCEIVING)
            return

        # ---- PLANNING ----
        if self.state == MissionState.PLANNING:
            self._stop_dog()
            start_cell = self._get_start_cell_from_dog()
            end_cell = (ROWS - 1, COLS - 1)

            self.get_logger().info(
                f'Planning path: cell {start_cell} -> cell {end_cell}')
            self.planner = MazeAstar(occ=self._current_occ)
            waypoints = self.planner.plan(start_cell=start_cell, end_cell=end_cell)

            if waypoints is None:
                self.get_logger().warn(
                    'A* found no path. Possibly bad perception. Re-entering PERCEIVING.')
                self._start_perceiving()
                return

            smoothed = self.planner.smooth_path(waypoints)
            self.get_logger().info(
                f'Path found: {len(waypoints)} waypoints -> {len(smoothed)} after smoothing')
            self._publish_planned_path(smoothed)

            self.dog_ctrl.reset(smoothed)
            self.wp_total = len(smoothed)
            self.wp_reached = 0
            self._transition(MissionState.GUIDING_DOG)
            return

        # ---- GUIDING_DOG ----
        if self.state == MissionState.GUIDING_DOG:
            self._publish_drone_hover()

            if self.dog_pos is None:
                self._stop_dog()
                return

            dt = 1.0 / self.CONTROL_HZ
            lin_x, ang_z = self.dog_ctrl.update(self.dog_pos, self.dog_yaw, dt)

            # Track waypoint completion
            new_wp_idx = self.dog_ctrl.current_waypoint_index()
            if new_wp_idx > self.wp_reached:
                self.wp_reached = new_wp_idx
                self.get_logger().info(
                    f'Waypoint reached: {self.wp_reached}/{self.wp_total}')

            # Distance to current waypoint
            dist_to_wp = 0.0
            wp = self.dog_ctrl.current_waypoint()
            if wp is not None:
                dist_to_wp = float(np.linalg.norm(self.dog_pos[:2] - wp[:2]))

            self._publish_dog_twist(lin_x, ang_z)
            self._log_csv(lin_x, ang_z, dist_to_wp)

            # Check stuck
            if self.dog_ctrl.is_stuck(self.dog_pos[:2]):
                self.stuck_count += 1
                self.get_logger().warn(
                    f'Dog stuck! Attempt {self.stuck_count}. Re-entering PERCEIVING.')
                self._stop_dog()
                self._start_perceiving()
                return

            # Check success
            if self.dog_ctrl.is_done():
                end_x, end_y = get_cell_center_world(ROWS - 1, COLS - 1)
                dist_to_end = float(np.linalg.norm(
                    self.dog_pos[:2] - np.array([end_x, end_y])))
                self.get_logger().info(
                    f'Dog reached final waypoint! Distance to end cell: {dist_to_end:.3f}m')
                self._stop_dog()
                self._transition(MissionState.SUCCESS)
            return

        # ---- DOG_STUCK (alias for PERCEIVING re-entry logic above) ----
        # (Handled inline in GUIDING_DOG — this state is kept for logging clarity)
        if self.state == MissionState.DOG_STUCK:
            self._start_perceiving()
            return

        # ---- SUCCESS ----
        if self.state == MissionState.SUCCESS:
            self._stop_dog()
            self._publish_drone_hover()
            if self._time_in_state() < 1.0:
                self._print_mission_summary()
            return

        # ---- FAILURE ----
        if self.state == MissionState.FAILURE:
            self._stop_dog()
            if self._time_in_state() < 1.0:
                self.get_logger().error('Mission FAILED. See logs.')
                self._csv_file.flush()
            return

    # ------------------------------------------------------------------
    # Mission summary
    # ------------------------------------------------------------------
    def _print_mission_summary(self):
        mission_time = 0.0
        if self.mission_start_time is not None:
            mission_time = time.monotonic() - self.mission_start_time

        lines = [
            '=' * 55,
            'MAZE MISSION SUMMARY',
            '=' * 55,
            f'Mission time:        {mission_time:.1f}s',
            f'Waypoints reached:   {self.wp_reached}/{self.wp_total}',
            f'Stuck events:        {self.stuck_count}',
            f'Vision IoU:          {self.vision_iou if self.vision_iou is not None else "N/A"}',
            f'CSV log:             {self.csv_path}',
            '=' * 55,
        ]
        for line in lines:
            self.get_logger().info(line)
        self._csv_file.flush()

    def destroy_node(self):
        self._stop_dog()
        self._csv_file.close()
        super().destroy_node()

    # ------------------------------------------------------------------
    # Ground-truth injection (called from omniverse_sim.py after maze spawn)
    # ------------------------------------------------------------------
    def set_ground_truth_grid(self, gt_grid):
        """
        Inject the ground-truth occupancy grid (from MazeGenerator) for IoU evaluation.
        Call this from omniverse_sim.py after spawn_walls_in_stage().
        """
        self._gt_grid = gt_grid
        self.get_logger().info('Ground-truth occupancy grid set.')


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = MazeMissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
