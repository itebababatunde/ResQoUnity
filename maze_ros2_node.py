#!/usr/bin/env python3
"""
maze_ros2_node.py — Central coordination node for aerial-assisted maze navigation.

Coordinates a drone (hovering observer) and one Go2 dog (maze navigator).
The drone climbs to 10.5m, photographs the maze from above, the vision pipeline
builds an occupancy grid, A* finds the shortest path, and the dog follows it.

State machine:
    INIT -> ARMING -> CLIMBING -> CENTERING -> ALTITUDE_SURVEY -> PERCEIVING -> PLANNING
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
import threading
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

try:
    import cv2
    _CV2_AVAILABLE = True
except ImportError:
    _CV2_AVAILABLE = False
    print("[MazeNode] WARNING: cv2 not available; display windows disabled.")

# Project imports
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from maze_generator import (
    generate_maze_grid, get_occupancy_grid, get_cell_center_world,
    ROWS, COLS, CELL_SIZE,
)
from maze_astar import MazeAstar
from maze_vision import MazeVisionNode, corners_visible_at
from maze_dog_controller import MazeDogController


# ---------------------------------------------------------------------------
# State machine
# ---------------------------------------------------------------------------
class MissionState(Enum):
    INIT            = auto()
    ARMING          = auto()
    CLIMBING        = auto()
    CENTERING       = auto()
    ALTITUDE_SURVEY = auto()
    PERCEIVING      = auto()
    PLANNING        = auto()
    SHOWING_PATH    = auto()
    GUIDING_DOG     = auto()
    DOG_STUCK       = auto()
    SUCCESS         = auto()
    FAILURE         = auto()


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
    half_w = (cols - 1) * CELL_SIZE / 2.0
    half_h = (rows - 1) * CELL_SIZE / 2.0
    col = int(round((x + half_w) / CELL_SIZE))
    row = int(round((y + half_h) / CELL_SIZE))
    row = max(0, min(rows - 1, row))
    col = max(0, min(cols - 1, col))
    return (row, col)


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------
class MazeMissionNode(Node):

    # Path display pause before dog starts moving
    PATH_DISPLAY_SEC    = 3.0    # s — show path overlay before GUIDING_DOG

    # Altitude survey parameters
    SURVEY_START_ALT    = 15.0   # m — start climbing from here toward maze centre
    SURVEY_STEP_M       = 1.0    # m — altitude increment per survey step
    SURVEY_CHECK_SEC    = 2.0    # s — interval between coverage checks
    SURVEY_CONFIRM_COUNT = 3     # consecutive passing checks before locking altitude
    MAX_SURVEY_ALT      = 30.0   # m — give up if still not covered at this height
    CENTER_TOL_M        = 0.5    # m — XY tolerance to consider drone "centred"

    PERCEIVE_FRAMES     = 10     # number of camera frames to accumulate
    PERCEIVE_TIMEOUT    = 30.0   # seconds before PERCEIVING gives up
    CONTROL_HZ          = 10.0   # Hz for main timer

    def __init__(self, seed=42):
        super().__init__('maze_mission_node')
        self._seed = seed

        # Hover altitude — set dynamically by ALTITUDE_SURVEY
        self.DRONE_HOVER_ALT = self.SURVEY_START_ALT

        # Altitude survey state
        self.survey_alt = self.SURVEY_START_ALT
        self.survey_confirm_count = 0
        self._last_survey_check = 0.0   # monotonic time of last coverage check

        # Display state
        self._display_camera = False     # enabled after altitude survey confirms coverage
        self._latest_frame = None        # most recent camera frame (RGB numpy array)
        self._path_overlay_img = None    # BGR image with path drawn on camera frame
        self._cam_w = 640                # actual camera frame width  (updated on first frame)
        self._cam_h = 480                # actual camera frame height (updated on first frame)

        # Ground-truth occupancy grid — generated from seed, used for path planning.
        # The perceived grid (from camera) is kept for IoU comparison only.
        _gt_maze = generate_maze_grid(seed=seed)
        self._gt_grid = get_occupancy_grid(_gt_maze)
        self.get_logger().info(f'Ground-truth grid generated (seed={seed}).')

        # Vision — altitude updated once survey completes
        self.vision = MazeVisionNode(drone_altitude=self.SURVEY_START_ALT)
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

        # ---- Last command (for per-tick CSV logging across all states) ----
        self._last_vel_x = 0.0
        self._last_ang_z = 0.0
        self._last_dist_to_wp = 0.0

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

        # CSV logger + snapshot directory
        log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'logs')
        os.makedirs(log_dir, exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(log_dir, f'maze_{ts}.csv')
        self._snapshot_dir = os.path.join(log_dir, 'snapshots')
        os.makedirs(self._snapshot_dir, exist_ok=True)
        self._snapshot_ts = ts   # shared prefix for all snapshots in this run
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
        """Store latest camera frame; accumulate during PERCEIVING."""
        if not _CV_BRIDGE_AVAILABLE:
            return
        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            rgb = np.array(cv_image)
            self._latest_frame = rgb
            h, w = rgb.shape[:2]
            if w != self._cam_w or h != self._cam_h:
                self._cam_w, self._cam_h = w, h
                self.get_logger().info(f'[Camera] Frame size detected: {w}×{h}')
            if self.state == MissionState.PERCEIVING:
                self.vision.add_frame(rgb)
                if self.vision.frame_count() % 5 == 0:
                    self.get_logger().info(
                        f'[PERCEIVING] Frames accumulated: {self.vision.frame_count()}/{self.PERCEIVE_FRAMES}')
        except Exception as e:
            self.get_logger().warn(f'Camera callback error: {e}')

    # ------------------------------------------------------------------
    # Command helpers
    # ------------------------------------------------------------------
    def _publish_drone_cmd(self, x, y, z):
        """Send the drone to world position (x, y, z)."""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation.w = 1.0
        self.drone_cmd_pub.publish(msg)

    def _publish_drone_hover(self):
        """Command drone to hold at maze centre (0, 0) at DRONE_HOVER_ALT."""
        self._publish_drone_cmd(0.0, 0.0, self.DRONE_HOVER_ALT)

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
    # Display helpers
    # ------------------------------------------------------------------
    def _draw_path_overlay(self, waypoints):
        """Return a BGR image of the latest camera frame with the planned path drawn on it."""
        if self._latest_frame is None or not _CV2_AVAILABLE:
            return None
        img = cv2.cvtColor(self._latest_frame, cv2.COLOR_RGB2BGR)
        h, w = img.shape[:2]
        # Build a vision node sized to the actual camera frame so world_to_pixel
        # returns coordinates in the correct pixel space (camera may differ from
        # the hardcoded 640×480 defaults in MazeVisionNode).
        if w != self.vision.img_w or h != self.vision.img_h:
            _vis = MazeVisionNode(drone_altitude=self.DRONE_HOVER_ALT,
                                  img_width=w, img_height=h)
        else:
            _vis = self.vision
        drone_x = self.drone_pos[0] if self.drone_pos is not None else 0.0
        drone_y = self.drone_pos[1] if self.drone_pos is not None else 0.0
        pts = []
        for wp in waypoints:
            px, py = _vis.world_to_pixel(wp[0], wp[1], drone_x=drone_x, drone_y=drone_y)
            pts.append((int(px), int(py)))
        # Scale line/marker thickness relative to image width so it looks right
        # at any camera resolution (e.g. 640×480 vs 1280×720).
        thickness = max(2, w // 320)
        dot_r    = max(3, w // 160)
        ring_r   = max(8, w // 80)
        for i in range(len(pts) - 1):
            cv2.line(img, pts[i], pts[i + 1], (0, 255, 0), thickness)
        for pt in pts:
            cv2.circle(img, pt, dot_r, (0, 200, 0), -1)
        if pts:
            cv2.circle(img, pts[0],  ring_r, (0, 255, 0), thickness)   # start: green ring
            cv2.circle(img, pts[-1], ring_r, (0, 0, 255), thickness)   # end:   red ring
        cv2.putText(img, f'Path: {len(waypoints)} waypoints  (dog starts in {self.PATH_DISPLAY_SEC:.0f}s)',
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        return img

    def _save_snapshot(self, label, img=None):
        """Save a PNG to logs/snapshots/<ts>_<label>.png."""
        if not _CV2_AVAILABLE:
            return
        if img is not None:
            src = img
        elif self._latest_frame is not None:
            src = cv2.cvtColor(self._latest_frame, cv2.COLOR_RGB2BGR)
        else:
            return
        fname = f'{self._snapshot_ts}_{label}.png'
        path = os.path.join(self._snapshot_dir, fname)
        cv2.imwrite(path, src)
        self.get_logger().info(f'[Snapshot] {fname}')

    def _tick_display(self):
        """Call once per control-loop tick to refresh the OpenCV window."""
        if not _CV2_AVAILABLE or not self._display_camera:
            return
        if self.state == MissionState.SHOWING_PATH and self._path_overlay_img is not None:
            cv2.imshow('Drone Camera', self._path_overlay_img)
        elif self._latest_frame is not None:
            img = cv2.cvtColor(self._latest_frame, cv2.COLOR_RGB2BGR)
            alt = self.drone_pos[2] if self.drone_pos is not None else 0.0
            cv2.putText(img, f'State: {self.state.name}  Alt: {alt:.1f}m',
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.imshow('Drone Camera', img)
        cv2.waitKey(1)

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
        self._tick_display()
        self._log_csv(self._last_vel_x, self._last_ang_z, self._last_dist_to_wp)

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
                    self._transition(MissionState.CENTERING)
                else:
                    msg = result.message if result else 'no response'
                    self.get_logger().warn(f'Takeoff failed: {msg}. Retrying...')
                    self._takeoff_future = None
            # Already direct drone toward maze centre at survey start altitude
            self._publish_drone_cmd(0.0, 0.0, self.SURVEY_START_ALT)
            return

        # ---- CENTERING ----
        if self.state == MissionState.CENTERING:
            self._publish_drone_cmd(0.0, 0.0, self.SURVEY_START_ALT)
            if self.drone_pos is not None:
                xy_err = math.hypot(self.drone_pos[0], self.drone_pos[1])
                alt_ok = self.drone_pos[2] >= self.SURVEY_START_ALT - 1.0
                if xy_err < self.CENTER_TOL_M and alt_ok:
                    self.get_logger().info(
                        f'[CENTERING] Drone centred at ({self.drone_pos[0]:.2f}, '
                        f'{self.drone_pos[1]:.2f}) alt={self.drone_pos[2]:.2f}m. '
                        f'Starting altitude survey from {self.SURVEY_START_ALT:.0f}m.')
                    self.survey_alt = self.SURVEY_START_ALT
                    self.survey_confirm_count = 0
                    self._last_survey_check = time.monotonic()
                    self._save_snapshot('1_survey_start')
                    self._transition(MissionState.ALTITUDE_SURVEY)
            return

        # ---- ALTITUDE_SURVEY ----
        if self.state == MissionState.ALTITUDE_SURVEY:
            self._publish_drone_cmd(0.0, 0.0, self.survey_alt)

            if self.drone_pos is None:
                return

            now = time.monotonic()
            if now - self._last_survey_check < self.SURVEY_CHECK_SEC:
                return
            self._last_survey_check = now

            covered = corners_visible_at(0.0, 0.0, self.survey_alt,
                                         img_width=self._cam_w, img_height=self._cam_h)
            if covered:
                self.survey_confirm_count += 1
                self.get_logger().info(
                    f'[ALTITUDE_SURVEY] alt={self.survey_alt:.0f}m: corners visible '
                    f'({self.survey_confirm_count}/{self.SURVEY_CONFIRM_COUNT})')
                if self.survey_confirm_count >= self.SURVEY_CONFIRM_COUNT:
                    self.DRONE_HOVER_ALT = self.survey_alt
                    self.vision = MazeVisionNode(drone_altitude=self.DRONE_HOVER_ALT,
                                                 img_width=self._cam_w, img_height=self._cam_h)
                    self._display_camera = True
                    self.get_logger().info(
                        f'[ALTITUDE_SURVEY] Survey complete. Locked altitude={self.DRONE_HOVER_ALT:.0f}m. '
                        f'Camera window opened.')
                    self._save_snapshot('2_survey_complete')
                    self._start_perceiving()
            else:
                self.survey_confirm_count = 0
                if self.survey_alt < self.MAX_SURVEY_ALT:
                    self.survey_alt += self.SURVEY_STEP_M
                    self.get_logger().info(
                        f'[ALTITUDE_SURVEY] Corners not visible. Climbing to {self.survey_alt:.0f}m...')
                else:
                    self.get_logger().error(
                        f'[ALTITUDE_SURVEY] Reached MAX_SURVEY_ALT={self.MAX_SURVEY_ALT}m '
                        f'without full coverage. Proceeding anyway.')
                    self.DRONE_HOVER_ALT = self.survey_alt
                    self.vision = MazeVisionNode(drone_altitude=self.DRONE_HOVER_ALT,
                                                 img_width=self._cam_w, img_height=self._cam_h)
                    self._display_camera = True
                    self._start_perceiving()
            return

        # ---- PERCEIVING ----
        if self.state == MissionState.PERCEIVING:
            self._publish_drone_hover()
            self._stop_dog()

            frames = self.vision.frame_count()
            elapsed = self._time_in_state()

            if frames == self.PERCEIVE_FRAMES // 2:
                self._save_snapshot('3_perceiving_mid')

            if frames >= self.PERCEIVE_FRAMES:
                self._save_snapshot('4_perceiving_done')
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
            self.planner = MazeAstar(occ=self._gt_grid)
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
            self._path_overlay_img = self._draw_path_overlay(smoothed)
            self._save_snapshot('5_path_planned', img=self._path_overlay_img)

            self.dog_ctrl.reset(smoothed)
            self.wp_total = len(smoothed)
            self.wp_reached = 0
            self._transition(MissionState.SHOWING_PATH)
            return

        # ---- SHOWING_PATH ----
        if self.state == MissionState.SHOWING_PATH:
            self._publish_drone_hover()
            self._stop_dog()
            if self._time_in_state() >= self.PATH_DISPLAY_SEC:
                self._path_overlay_img = None
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
                self._save_snapshot(f'6_waypoint_{self.wp_reached:02d}of{self.wp_total:02d}')

            # Distance to current waypoint
            dist_to_wp = 0.0
            wp = self.dog_ctrl.current_waypoint()
            if wp is not None:
                dist_to_wp = float(np.linalg.norm(self.dog_pos[:2] - wp[:2]))

            self._last_vel_x = lin_x
            self._last_ang_z = ang_z
            self._last_dist_to_wp = dist_to_wp
            self._publish_dog_twist(lin_x, ang_z)

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
                self._save_snapshot('7_mission_success')
                self._print_mission_summary()
                threading.Thread(target=self._generate_report, daemon=True).start()
            return

        # ---- FAILURE ----
        if self.state == MissionState.FAILURE:
            self._stop_dog()
            if self._time_in_state() < 1.0:
                self._save_snapshot('7_mission_failure')
                self.get_logger().error('Mission FAILED. See logs.')
                threading.Thread(target=self._generate_report, daemon=True).start()
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

    # ------------------------------------------------------------------
    # Report generation (runs in background thread to avoid blocking spin)
    # ------------------------------------------------------------------
    def _generate_report(self):
        try:
            from maze_report import build_report
            self._csv_file.flush()
            out = build_report(self.csv_path, seed=self._seed)
            self.get_logger().info(f'[Report] Saved: {out}')
        except Exception as e:
            self.get_logger().error(f'[Report] Generation failed: {e}')

    def destroy_node(self):
        self._stop_dog()
        self._csv_file.close()
        if _CV2_AVAILABLE:
            cv2.destroyAllWindows()
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
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--seed', type=int, default=42, help='Maze seed (must match sim)')
    known, remaining = parser.parse_known_args(args)

    rclpy.init(args=remaining)
    node = MazeMissionNode(seed=known.seed)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
