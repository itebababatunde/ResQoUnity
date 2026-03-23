#!/usr/bin/env python3
"""
Leader-Follower Coordination ROS2 Node.

Standalone node that coordinates a drone (leader) and two Go2 dogs (followers)
by reading odometry from the simulation and publishing commands.

Both dogs follow the drone independently using APF controllers with
inter-dog repulsion to avoid collisions.

The simulation (run_sim.sh) must already be running with both robots spawned.

Usage:
    Terminal 1: ./start_simulation.sh
    Terminal 2: ./run_leader_follower_ros2.sh
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
from rclpy.qos import QoSProfile

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Twist, Point
from std_srvs.srv import SetBool, Trigger
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

# Add project root so we can import planner/controller
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from lawnmower_planner import LawnmowerPlanner, LawnmowerConfig
from apf_controller import APFController, APFConfig
from follower_strategies import get_strategy, STRATEGIES


class MissionState(Enum):
    INIT = auto()
    ARMING = auto()
    TAKING_OFF = auto()
    WAITING_ALTITUDE = auto()
    RUNNING = auto()
    COMPLETING = auto()
    LANDING = auto()
    DONE = auto()


def quat_to_yaw(x, y, z, w):
    """Extract yaw (heading) from quaternion (x, y, z, w)."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class LeaderFollowerNode(Node):
    def __init__(self):
        super().__init__('leader_follower_node')

        # Declare ROS2 parameters with defaults
        self.declare_parameter('workspace_size', 10.0)
        self.declare_parameter('sweep_spacing', 2.0)
        self.declare_parameter('drone_speed', 0.4)
        self.declare_parameter('drone_altitude', 3.0)
        self.declare_parameter('k_att', 1.0)
        self.declare_parameter('k_rep', 2.0)
        self.declare_parameter('d_safe', 2.0)
        self.declare_parameter('d_influence', 3.0)
        self.declare_parameter('max_velocity', 1.5)
        self.declare_parameter('yaw_p_gain', 1.5)
        self.declare_parameter('altitude_wait_sec', 3.0)
        self.declare_parameter('completion_hover_sec', 3.0)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('strategy', 'naive')
        self.declare_parameter('lateral_offset', 2.0)
        self.declare_parameter('trail_delay', 3.0)

        # Read parameters
        ws = self.get_parameter('workspace_size').value
        spacing = self.get_parameter('sweep_spacing').value
        speed = self.get_parameter('drone_speed').value
        alt = self.get_parameter('drone_altitude').value
        k_att = self.get_parameter('k_att').value
        k_rep = self.get_parameter('k_rep').value
        d_safe = self.get_parameter('d_safe').value
        d_inf = self.get_parameter('d_influence').value
        max_vel = self.get_parameter('max_velocity').value
        self.yaw_p_gain = self.get_parameter('yaw_p_gain').value
        self.altitude_wait_sec = self.get_parameter('altitude_wait_sec').value
        self.completion_hover_sec = self.get_parameter('completion_hover_sec').value
        control_hz = self.get_parameter('control_rate_hz').value

        # Planner & controllers
        planner_cfg = LawnmowerConfig(
            workspace_size=ws,
            sweep_spacing=spacing,
            drone_speed=speed,
            drone_altitude=alt,
        )
        self.planner = LawnmowerPlanner(planner_cfg)

        apf_cfg = APFConfig(
            k_att=k_att,
            k_rep=k_rep,
            d_safe=d_safe,
            d_influence=d_inf,
            max_velocity=max_vel,
        )
        self.apf = APFController(apf_cfg)
        self.apf_second = APFController(apf_cfg)

        # Follower strategy
        strategy_name = self.get_parameter('strategy').value
        strategy_kwargs = {}
        if strategy_name == 'lateral_offset':
            strategy_kwargs = dict(
                offset_distance=self.get_parameter('lateral_offset').value,
                workspace_origin=planner_cfg.start_corner,
                workspace_size=ws,
            )
        elif strategy_name == 'trailing':
            control_hz = self.get_parameter('control_rate_hz').value
            strategy_kwargs = dict(
                trail_delay=self.get_parameter('trail_delay').value,
                dt=1.0 / control_hz,
            )
        self.strategy = get_strategy(strategy_name, **strategy_kwargs)
        self.strategy_name = strategy_name

        self.get_logger().info(f'Planner: {self.planner}')
        self.get_logger().info(f'APF: {self.apf}')
        self.get_logger().info(f'Strategy: {self.strategy}')
        self.get_logger().info(f'Total mission time: {self.planner.total_time:.1f}s')

        # State machine
        self.state = MissionState.INIT
        self.mission_start_time = None
        self.state_enter_time = None

        # Odometry state — Dog 0 (robot0)
        self.drone_pos = None       # [x, y, z]
        self.drone_quat = None      # [x, y, z, w]
        self.dog_pos = None         # [x, y, z]
        self.dog_quat = None        # [x, y, z, w]
        self.dog_yaw = 0.0
        self.drone_odom_received = False
        self.dog_odom_received = False

        # Dog 0 velocity estimation
        self.dog_prev_pos = None
        self.dog_prev_time = None
        self.dog_vel_estimate = np.array([0.0, 0.0])

        # Odometry state — Dog 1 (robot1)
        self.second_dog_pos = None
        self.second_dog_quat = None
        self.second_dog_yaw = 0.0
        self.second_dog_odom_received = False

        # Dog 1 velocity estimation
        self.second_dog_prev_pos = None
        self.second_dog_prev_time = None
        self.second_dog_vel_estimate = np.array([0.0, 0.0])

        # Safety tracking
        self.min_separation = float('inf')
        self.max_separation = 0.0
        self.safety_violations = 0
        self.min_separation_second = float('inf')
        self.max_separation_second = 0.0
        self.safety_violations_second = 0
        self.min_dog_dog_separation = float('inf')
        self.total_control_steps = 0

        # Subscribers
        qos = QoSProfile(depth=10)
        self.create_subscription(Odometry, '/drone/odom', self._drone_odom_cb, qos)
        self.create_subscription(Odometry, '/robot0/odom', self._dog_odom_cb, qos)
        self.create_subscription(Odometry, '/robot1/odom', self._second_dog_odom_cb, qos)

        # Publishers
        self.drone_cmd_pub = self.create_publisher(PoseStamped, '/drone/cmd_position', qos)
        self.dog_cmd_pub = self.create_publisher(Twist, '/robot0/cmd_vel', qos)
        self.second_dog_cmd_pub = self.create_publisher(Twist, '/robot1/cmd_vel', qos)

        # Visualization publishers
        self.planned_path_pub = self.create_publisher(Path, '/viz/planned_path', qos)
        self.workspace_boundary_pub = self.create_publisher(Marker, '/viz/workspace_boundary', qos)
        self.sweep_lines_pub = self.create_publisher(Marker, '/viz/sweep_lines', qos)
        self.drone_trail_pub = self.create_publisher(Path, '/viz/drone_trail', qos)
        self.dog0_trail_pub = self.create_publisher(Path, '/viz/dog0_trail', qos)
        self.dog1_trail_pub = self.create_publisher(Path, '/viz/dog1_trail', qos)
        self.current_target_pub = self.create_publisher(Marker, '/viz/current_target', qos)
        self.progress_pub = self.create_publisher(Marker, '/viz/progress', qos)

        # Trail state
        self.drone_trail_msg = Path()
        self.drone_trail_msg.header.frame_id = 'odom'
        self.dog0_trail_msg = Path()
        self.dog0_trail_msg.header.frame_id = 'odom'
        self.dog1_trail_msg = Path()
        self.dog1_trail_msg.header.frame_id = 'odom'
        self.static_viz_published = False

        # Service clients
        self.arm_client = self.create_client(SetBool, '/drone/arm')
        self.takeoff_client = self.create_client(Trigger, '/drone/takeoff')
        self.land_client = self.create_client(Trigger, '/drone/land')

        # Pending service futures
        self._arm_future = None
        self._takeoff_future = None
        self._land_future = None

        # Control timer
        period = 1.0 / control_hz
        self.timer = self.create_timer(period, self._control_loop)

        # CSV position logger
        log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'logs')
        os.makedirs(log_dir, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(log_dir, f'mission_{self.strategy_name}_{timestamp}.csv')
        self._csv_file = open(self.csv_path, 'w', newline='')
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow([
            't', 'step',
            'drone_x', 'drone_y', 'drone_z',
            'dog0_x', 'dog0_y', 'dog0_z',
            'dog1_x', 'dog1_y', 'dog1_z',
            'target_x', 'target_y', 'target_z',
        ])

        self.get_logger().info('Leader-Follower node initialized (2 dogs). Waiting for odometry...')
        self.get_logger().info(f'CSV log: {self.csv_path}')

    # ------------------------------------------------------------------
    # Odometry callbacks
    # ------------------------------------------------------------------
    def _drone_odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self.drone_pos = np.array([p.x, p.y, p.z])
        q = msg.pose.pose.orientation
        self.drone_quat = (q.x, q.y, q.z, q.w)
        if not self.drone_odom_received:
            self.drone_odom_received = True
            self.get_logger().info(
                f'Drone odom received: ({p.x:.2f}, {p.y:.2f}, {p.z:.2f})')

    def _dog_odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self.dog_pos = np.array([p.x, p.y, p.z])
        q = msg.pose.pose.orientation
        self.dog_quat = (q.x, q.y, q.z, q.w)
        self.dog_yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

        # Estimate velocity by differentiating position
        now = time.monotonic()
        if self.dog_prev_pos is not None and self.dog_prev_time is not None:
            dt = now - self.dog_prev_time
            if dt > 0.001:
                self.dog_vel_estimate = (self.dog_pos[:2] - self.dog_prev_pos[:2]) / dt
        self.dog_prev_pos = self.dog_pos.copy()
        self.dog_prev_time = now

        if not self.dog_odom_received:
            self.dog_odom_received = True
            self.get_logger().info(
                f'Dog0 odom received: ({p.x:.2f}, {p.y:.2f}, {p.z:.2f})')

    def _second_dog_odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self.second_dog_pos = np.array([p.x, p.y, p.z])
        q = msg.pose.pose.orientation
        self.second_dog_quat = (q.x, q.y, q.z, q.w)
        self.second_dog_yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

        # Estimate velocity by differentiating position
        now = time.monotonic()
        if self.second_dog_prev_pos is not None and self.second_dog_prev_time is not None:
            dt = now - self.second_dog_prev_time
            if dt > 0.001:
                self.second_dog_vel_estimate = (self.second_dog_pos[:2] - self.second_dog_prev_pos[:2]) / dt
        self.second_dog_prev_pos = self.second_dog_pos.copy()
        self.second_dog_prev_time = now

        if not self.second_dog_odom_received:
            self.second_dog_odom_received = True
            self.get_logger().info(
                f'Dog1 odom received: ({p.x:.2f}, {p.y:.2f}, {p.z:.2f})')

    # ------------------------------------------------------------------
    # Command helpers
    # ------------------------------------------------------------------
    def _publish_drone_position(self, pos):
        """Publish a PoseStamped to /drone/cmd_position."""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.pose.position.x = float(pos[0])
        msg.pose.position.y = float(pos[1])
        msg.pose.position.z = float(pos[2])
        msg.pose.orientation.w = 1.0
        self.drone_cmd_pub.publish(msg)

    def _publish_dog_velocity(self, world_vx, world_vy, yaw_rate, dog_index=0):
        """Convert world-frame velocity to body frame and publish Twist."""
        if dog_index == 0:
            yaw = self.dog_yaw
            pub = self.dog_cmd_pub
        else:
            yaw = self.second_dog_yaw
            pub = self.second_dog_cmd_pub

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        body_vx = cos_yaw * world_vx + sin_yaw * world_vy
        body_vy = -sin_yaw * world_vx + cos_yaw * world_vy

        msg = Twist()
        msg.linear.x = float(body_vx)
        msg.linear.y = float(body_vy)
        msg.angular.z = float(yaw_rate)
        pub.publish(msg)

    def _stop_dogs(self):
        """Publish zero velocity to both dogs."""
        msg = Twist()
        self.dog_cmd_pub.publish(msg)
        self.second_dog_cmd_pub.publish(msg)

    # ------------------------------------------------------------------
    # State machine
    # ------------------------------------------------------------------
    def _transition(self, new_state):
        self.get_logger().info(f'State: {self.state.name} -> {new_state.name}')
        self.state = new_state
        self.state_enter_time = time.monotonic()

    def _time_in_state(self):
        if self.state_enter_time is None:
            return 0.0
        return time.monotonic() - self.state_enter_time

    def _control_loop(self):
        """Main 20 Hz control loop with state machine."""

        # ---- INIT: wait for all 3 odom topics ----
        if self.state == MissionState.INIT:
            if self.drone_odom_received and self.dog_odom_received and self.second_dog_odom_received:
                self.get_logger().info('Odometry received from all 3 robots (drone + 2 dogs)')
                self._transition(MissionState.ARMING)
            return

        # ---- ARMING: call /drone/arm ----
        if self.state == MissionState.ARMING:
            if self._arm_future is None:
                if not self.arm_client.service_is_ready():
                    if self._time_in_state() > 15.0:
                        self.get_logger().error('/drone/arm service not available after 15s')
                    return
                req = SetBool.Request()
                req.data = True
                self._arm_future = self.arm_client.call_async(req)
                self.get_logger().info('Arming drone...')
            elif self._arm_future.done():
                result = self._arm_future.result()
                if result is not None and result.success:
                    self.get_logger().info(f'Drone armed: {result.message}')
                    self._transition(MissionState.TAKING_OFF)
                else:
                    msg = result.message if result else 'no response'
                    self.get_logger().warn(f'Arm failed: {msg}, retrying...')
                    self._arm_future = None
            return

        # ---- TAKING_OFF: call /drone/takeoff ----
        if self.state == MissionState.TAKING_OFF:
            if self._takeoff_future is None:
                if not self.takeoff_client.service_is_ready():
                    return
                self._takeoff_future = self.takeoff_client.call_async(Trigger.Request())
                self.get_logger().info('Taking off...')
            elif self._takeoff_future.done():
                result = self._takeoff_future.result()
                if result is not None and result.success:
                    self.get_logger().info(f'Takeoff initiated: {result.message}')
                    self._transition(MissionState.WAITING_ALTITUDE)
                else:
                    msg = result.message if result else 'no response'
                    self.get_logger().warn(f'Takeoff failed: {msg}, retrying...')
                    self._takeoff_future = None
            return

        # ---- WAITING_ALTITUDE: wait for drone to reach cruise altitude ----
        if self.state == MissionState.WAITING_ALTITUDE:
            if self.drone_pos is not None:
                alt = self.drone_pos[2]
                target_alt = self.planner.config.drone_altitude
                if self._time_in_state() > self.altitude_wait_sec or alt >= target_alt * 0.7:
                    self.get_logger().info(
                        f'Altitude OK ({alt:.2f}m / {target_alt:.1f}m target). Starting mission.')
                    self.mission_start_time = time.monotonic()
                    self._transition(MissionState.RUNNING)
            return

        # ---- RUNNING: main leader-follower loop ----
        if self.state == MissionState.RUNNING:
            if not self.static_viz_published:
                self._publish_static_viz()
                self.static_viz_published = True
            self._run_leader_follower()
            return

        # ---- COMPLETING: brief hover before landing ----
        if self.state == MissionState.COMPLETING:
            self._stop_dogs()
            if self._time_in_state() > self.completion_hover_sec:
                self._print_mission_summary()
                self._transition(MissionState.LANDING)
            return

        # ---- LANDING: call /drone/land ----
        if self.state == MissionState.LANDING:
            self._stop_dogs()
            if self._land_future is None:
                if not self.land_client.service_is_ready():
                    return
                self._land_future = self.land_client.call_async(Trigger.Request())
                self.get_logger().info('Landing drone...')
            elif self._land_future.done():
                result = self._land_future.result()
                msg = result.message if result else 'done'
                self.get_logger().info(f'Land result: {msg}')
                self._transition(MissionState.DONE)
            return

        # ---- DONE ----
        if self.state == MissionState.DONE:
            self._stop_dogs()
            return

    # ------------------------------------------------------------------
    # Leader-follower control
    # ------------------------------------------------------------------
    def _run_leader_follower(self):
        if self.drone_pos is None or self.dog_pos is None or self.second_dog_pos is None:
            return

        t = time.monotonic() - self.mission_start_time

        # Check completion
        if self.planner.is_complete(t):
            self.get_logger().info('Lawnmower sweep complete!')
            self._stop_dogs()
            self._transition(MissionState.COMPLETING)
            return

        # --- Drone: publish next waypoint from planner ---
        target_pos = self.planner.get_position(t)
        self._publish_drone_position(target_pos)

        drone_2d = self.drone_pos[:2]
        dog0_2d = self.dog_pos[:2]
        dog1_2d = self.second_dog_pos[:2]

        # --- Compute strategy targets for each dog ---
        drone_vel = self.planner.get_velocity(t)
        target0_2d, target1_2d = self.strategy.get_targets(
            self.drone_pos, drone_vel, self.dog_pos, self.second_dog_pos)

        # --- Dog 0: compute APF velocity toward strategy target ---
        apf_vel_0 = self.apf.compute_velocity(
            target0_2d, dog0_2d, self.dog_vel_estimate, obstacles=[dog1_2d])

        dx0 = target0_2d[0] - self.dog_pos[0]
        dy0 = target0_2d[1] - self.dog_pos[1]
        desired_yaw_0 = math.atan2(dy0, dx0)
        yaw_error_0 = math.atan2(
            math.sin(desired_yaw_0 - self.dog_yaw),
            math.cos(desired_yaw_0 - self.dog_yaw))
        yaw_rate_0 = self.yaw_p_gain * yaw_error_0

        self._publish_dog_velocity(apf_vel_0[0], apf_vel_0[1], yaw_rate_0, dog_index=0)

        # --- Dog 1: compute APF velocity toward strategy target ---
        apf_vel_1 = self.apf_second.compute_velocity(
            target1_2d, dog1_2d, self.second_dog_vel_estimate, obstacles=[dog0_2d])

        dx1 = target1_2d[0] - self.second_dog_pos[0]
        dy1 = target1_2d[1] - self.second_dog_pos[1]
        desired_yaw_1 = math.atan2(dy1, dx1)
        yaw_error_1 = math.atan2(
            math.sin(desired_yaw_1 - self.second_dog_yaw),
            math.cos(desired_yaw_1 - self.second_dog_yaw))
        yaw_rate_1 = self.yaw_p_gain * yaw_error_1

        self._publish_dog_velocity(apf_vel_1[0], apf_vel_1[1], yaw_rate_1, dog_index=1)

        # --- Safety metrics ---
        sep0 = np.linalg.norm(drone_2d - dog0_2d)
        sep1 = np.linalg.norm(drone_2d - dog1_2d)
        dog_dog_sep = np.linalg.norm(dog0_2d - dog1_2d)

        self.total_control_steps += 1

        # Log to CSV
        self._csv_writer.writerow([
            f'{t:.4f}', self.total_control_steps,
            f'{self.drone_pos[0]:.4f}', f'{self.drone_pos[1]:.4f}', f'{self.drone_pos[2]:.4f}',
            f'{self.dog_pos[0]:.4f}', f'{self.dog_pos[1]:.4f}', f'{self.dog_pos[2]:.4f}',
            f'{self.second_dog_pos[0]:.4f}', f'{self.second_dog_pos[1]:.4f}', f'{self.second_dog_pos[2]:.4f}',
            f'{target_pos[0]:.4f}', f'{target_pos[1]:.4f}', f'{target_pos[2]:.4f}',
        ])

        self.min_separation = min(self.min_separation, sep0)
        self.max_separation = max(self.max_separation, sep0)
        if sep0 < self.apf.config.d_safe:
            self.safety_violations += 1
        self.min_separation_second = min(self.min_separation_second, sep1)
        self.max_separation_second = max(self.max_separation_second, sep1)
        if sep1 < self.apf_second.config.d_safe:
            self.safety_violations_second += 1
        self.min_dog_dog_separation = min(self.min_dog_dog_separation, dog_dog_sep)

        # Visualization
        self._publish_dynamic_viz(t)

        # Periodic logging (every ~2 seconds at 20 Hz)
        if self.total_control_steps % 40 == 0:
            progress = self.planner.get_progress(t) * 100.0
            self.get_logger().info(
                f'[{progress:5.1f}%] '
                f'drone=({self.drone_pos[0]:.1f},{self.drone_pos[1]:.1f},{self.drone_pos[2]:.1f}) '
                f'dog0=({self.dog_pos[0]:.1f},{self.dog_pos[1]:.1f}) sep={sep0:.2f}m '
                f'dog1=({self.second_dog_pos[0]:.1f},{self.second_dog_pos[1]:.1f}) sep={sep1:.2f}m '
                f'dog-dog={dog_dog_sep:.2f}m')

    # ------------------------------------------------------------------
    # Visualization
    # ------------------------------------------------------------------
    def _publish_static_viz(self):
        """Publish one-time visualization: planned path, workspace boundary, sweep lines."""
        now = self.get_clock().now().to_msg()
        ws = self.planner.config.workspace_size
        x0, y0 = self.planner.config.start_corner

        # --- Planned path ---
        path_msg = Path()
        path_msg.header.stamp = now
        path_msg.header.frame_id = 'odom'
        for wp in self.planner.get_waypoints():
            ps = PoseStamped()
            ps.header.stamp = now
            ps.header.frame_id = 'odom'
            ps.pose.position.x = float(wp[0])
            ps.pose.position.y = float(wp[1])
            ps.pose.position.z = float(wp[2])
            ps.pose.orientation.w = 1.0
            path_msg.poses.append(ps)
        self.planned_path_pub.publish(path_msg)

        # --- Workspace boundary (LINE_STRIP) ---
        boundary = Marker()
        boundary.header.stamp = now
        boundary.header.frame_id = 'odom'
        boundary.ns = 'workspace'
        boundary.id = 0
        boundary.type = Marker.LINE_STRIP
        boundary.action = Marker.ADD
        boundary.scale.x = 0.15
        boundary.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        boundary.pose.orientation.w = 1.0
        corners = [
            (x0, y0), (x0 + ws, y0), (x0 + ws, y0 + ws), (x0, y0 + ws), (x0, y0)
        ]
        for cx, cy in corners:
            p = Point(x=float(cx), y=float(cy), z=0.1)
            boundary.points.append(p)
        self.workspace_boundary_pub.publish(boundary)

        # --- Sweep lines (LINE_LIST) ---
        sweep = Marker()
        sweep.header.stamp = now
        sweep.header.frame_id = 'odom'
        sweep.ns = 'sweep'
        sweep.id = 0
        sweep.type = Marker.LINE_LIST
        sweep.action = Marker.ADD
        sweep.scale.x = 0.05
        sweep.color = ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.5)
        sweep.pose.orientation.w = 1.0
        wps = self.planner.get_waypoints()
        for i in range(0, len(wps) - 1, 2):
            sweep.points.append(Point(x=float(wps[i][0]), y=float(wps[i][1]), z=0.1))
            sweep.points.append(Point(x=float(wps[i+1][0]), y=float(wps[i+1][1]), z=0.1))
        self.sweep_lines_pub.publish(sweep)

        self.get_logger().info('Published static visualization (path, boundary, sweep lines)')

    def _publish_dynamic_viz(self, t):
        """Publish per-step visualization: trails, current target, progress text."""
        now = self.get_clock().now().to_msg()

        # --- Current target sphere (every step) ---
        target_pos = self.planner.get_position(t)
        target_marker = Marker()
        target_marker.header.stamp = now
        target_marker.header.frame_id = 'odom'
        target_marker.ns = 'target'
        target_marker.id = 0
        target_marker.type = Marker.SPHERE
        target_marker.action = Marker.ADD
        target_marker.pose.position.x = float(target_pos[0])
        target_marker.pose.position.y = float(target_pos[1])
        target_marker.pose.position.z = float(target_pos[2])
        target_marker.pose.orientation.w = 1.0
        target_marker.scale.x = 0.5
        target_marker.scale.y = 0.5
        target_marker.scale.z = 0.5
        target_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.9)
        self.current_target_pub.publish(target_marker)

        # --- Trails (every 5th step to keep messages small) ---
        if self.total_control_steps % 5 == 0:
            def _make_pose(pos):
                ps = PoseStamped()
                ps.header.stamp = now
                ps.header.frame_id = 'odom'
                ps.pose.position.x = float(pos[0])
                ps.pose.position.y = float(pos[1])
                ps.pose.position.z = float(pos[2]) if len(pos) > 2 else 0.0
                ps.pose.orientation.w = 1.0
                return ps

            self.drone_trail_msg.header.stamp = now
            self.drone_trail_msg.poses.append(_make_pose(self.drone_pos))
            self.drone_trail_pub.publish(self.drone_trail_msg)

            self.dog0_trail_msg.header.stamp = now
            self.dog0_trail_msg.poses.append(_make_pose(self.dog_pos))
            self.dog0_trail_pub.publish(self.dog0_trail_msg)

            self.dog1_trail_msg.header.stamp = now
            self.dog1_trail_msg.poses.append(_make_pose(self.second_dog_pos))
            self.dog1_trail_pub.publish(self.dog1_trail_msg)

        # --- Progress text (every 40th step, matching logging rate) ---
        if self.total_control_steps % 40 == 0:
            progress = self.planner.get_progress(t) * 100.0
            ws = self.planner.config.workspace_size
            x0, y0 = self.planner.config.start_corner

            text_marker = Marker()
            text_marker.header.stamp = now
            text_marker.header.frame_id = 'odom'
            text_marker.ns = 'progress'
            text_marker.id = 0
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = float(x0 + ws / 2.0)
            text_marker.pose.position.y = float(y0 + ws + 1.0)
            text_marker.pose.position.z = 1.0
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = 1.0
            text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            text_marker.text = f'{progress:.1f}% complete'
            self.progress_pub.publish(text_marker)

    # ------------------------------------------------------------------
    # Summary
    # ------------------------------------------------------------------
    def _print_mission_summary(self):
        self._csv_file.flush()
        self.get_logger().info(f'CSV log saved: {self.csv_path}')
        self.get_logger().info('=' * 60)
        self.get_logger().info('              MISSION SUMMARY')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'  Total control steps : {self.total_control_steps}')
        elapsed = time.monotonic() - self.mission_start_time if self.mission_start_time else 0
        self.get_logger().info(f'  Mission duration    : {elapsed:.1f}s')
        self.get_logger().info(f'  --- Dog 0 (robot0) ---')
        self.get_logger().info(f'  Min separation      : {self.min_separation:.2f}m')
        self.get_logger().info(f'  Max separation      : {self.max_separation:.2f}m')
        self.get_logger().info(f'  APF min observed    : {self.apf.get_min_distance_observed():.2f}m')
        self.get_logger().info(f'  Safety violations   : {self.safety_violations} '
                               f'({self.safety_violations/max(self.total_control_steps,1)*100:.1f}%)')
        self.get_logger().info(f'  --- Dog 1 (robot1) ---')
        self.get_logger().info(f'  Min separation      : {self.min_separation_second:.2f}m')
        self.get_logger().info(f'  Max separation      : {self.max_separation_second:.2f}m')
        self.get_logger().info(f'  APF min observed    : {self.apf_second.get_min_distance_observed():.2f}m')
        self.get_logger().info(f'  Safety violations   : {self.safety_violations_second} '
                               f'({self.safety_violations_second/max(self.total_control_steps,1)*100:.1f}%)')
        self.get_logger().info(f'  --- Inter-dog ---')
        self.get_logger().info(f'  Min dog-dog dist    : {self.min_dog_dog_separation:.2f}m')
        self.get_logger().info(f'  d_safe threshold    : {self.apf.config.d_safe:.1f}m')
        self.get_logger().info('=' * 60)

    # ------------------------------------------------------------------
    # Graceful shutdown
    # ------------------------------------------------------------------
    def shutdown(self):
        """Attempt to land drone on shutdown."""
        self.get_logger().info('Shutdown requested — landing drone...')
        self._csv_file.close()
        self.get_logger().info(f'CSV log saved: {self.csv_path}')
        self._stop_dogs()
        if self.land_client.service_is_ready():
            future = self.land_client.call_async(Trigger.Request())
            # Best-effort wait
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        self.get_logger().info('Shutdown complete.')


def main(args=None):
    rclpy.init(args=args)
    node = LeaderFollowerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.shutdown()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
