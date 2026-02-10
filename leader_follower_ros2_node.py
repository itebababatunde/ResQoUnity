#!/usr/bin/env python3
"""
Leader-Follower Coordination ROS2 Node.

Standalone node that coordinates a drone (leader) and Go2 dog (follower)
by reading odometry from the simulation and publishing commands.

The simulation (run_sim.sh) must already be running with both robots spawned.

Usage:
    Terminal 1: ./run_sim.sh
    Terminal 2: ./run_leader_follower_ros2.sh
"""

import sys
import os
import math
import time
from enum import Enum, auto

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
from std_srvs.srv import SetBool, Trigger

# Add project root so we can import planner/controller
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from lawnmower_planner import LawnmowerPlanner, LawnmowerConfig
from apf_controller import APFController, APFConfig


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
        self.declare_parameter('drone_speed', 1.0)
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

        # Planner & controller
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

        self.get_logger().info(f'Planner: {self.planner}')
        self.get_logger().info(f'APF: {self.apf}')
        self.get_logger().info(f'Total mission time: {self.planner.total_time:.1f}s')

        # State machine
        self.state = MissionState.INIT
        self.mission_start_time = None
        self.state_enter_time = None

        # Odometry state
        self.drone_pos = None       # [x, y, z]
        self.drone_quat = None      # [x, y, z, w]
        self.dog_pos = None         # [x, y, z]
        self.dog_quat = None        # [x, y, z, w]
        self.dog_yaw = 0.0
        self.drone_odom_received = False
        self.dog_odom_received = False

        # Dog velocity estimation via position differentiation
        self.dog_prev_pos = None
        self.dog_prev_time = None
        self.dog_vel_estimate = np.array([0.0, 0.0])

        # Safety tracking
        self.min_separation = float('inf')
        self.max_separation = 0.0
        self.safety_violations = 0
        self.total_control_steps = 0

        # Subscribers
        qos = QoSProfile(depth=10)
        self.create_subscription(Odometry, '/drone/odom', self._drone_odom_cb, qos)
        self.create_subscription(Odometry, '/robot0/odom', self._dog_odom_cb, qos)

        # Publishers
        self.drone_cmd_pub = self.create_publisher(PoseStamped, '/drone/cmd_position', qos)
        self.dog_cmd_pub = self.create_publisher(Twist, '/robot0/cmd_vel', qos)

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

        self.get_logger().info('Leader-Follower node initialized. Waiting for odometry...')

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
                f'Dog odom received: ({p.x:.2f}, {p.y:.2f}, {p.z:.2f})')

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

    def _publish_dog_velocity(self, world_vx, world_vy, yaw_rate):
        """Convert world-frame velocity to body frame and publish Twist."""
        # Rotate world velocity into dog body frame using yaw
        cos_yaw = math.cos(self.dog_yaw)
        sin_yaw = math.sin(self.dog_yaw)
        body_vx = cos_yaw * world_vx + sin_yaw * world_vy
        body_vy = -sin_yaw * world_vx + cos_yaw * world_vy

        msg = Twist()
        msg.linear.x = float(body_vx)
        msg.linear.y = float(body_vy)
        msg.angular.z = float(yaw_rate)
        self.dog_cmd_pub.publish(msg)

    def _stop_dog(self):
        """Publish zero velocity to dog."""
        msg = Twist()
        self.dog_cmd_pub.publish(msg)

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

        # ---- INIT: wait for both odom topics ----
        if self.state == MissionState.INIT:
            if self.drone_odom_received and self.dog_odom_received:
                self.get_logger().info('Odometry received from both robots')
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
            self._run_leader_follower()
            return

        # ---- COMPLETING: brief hover before landing ----
        if self.state == MissionState.COMPLETING:
            self._stop_dog()
            if self._time_in_state() > self.completion_hover_sec:
                self._print_mission_summary()
                self._transition(MissionState.LANDING)
            return

        # ---- LANDING: call /drone/land ----
        if self.state == MissionState.LANDING:
            self._stop_dog()
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
            self._stop_dog()
            return

    # ------------------------------------------------------------------
    # Leader-follower control
    # ------------------------------------------------------------------
    def _run_leader_follower(self):
        if self.drone_pos is None or self.dog_pos is None:
            return

        t = time.monotonic() - self.mission_start_time

        # Check completion
        if self.planner.is_complete(t):
            self.get_logger().info('Lawnmower sweep complete!')
            self._stop_dog()
            self._transition(MissionState.COMPLETING)
            return

        # --- Drone: publish next waypoint from planner ---
        target_pos = self.planner.get_position(t)
        self._publish_drone_position(target_pos)

        # --- Dog: compute APF velocity and publish ---
        drone_2d = self.drone_pos[:2]
        dog_2d = self.dog_pos[:2]

        apf_vel = self.apf.compute_velocity(drone_2d, dog_2d, self.dog_vel_estimate)

        # Compute yaw rate to orient dog toward drone
        dx = self.drone_pos[0] - self.dog_pos[0]
        dy = self.drone_pos[1] - self.dog_pos[1]
        desired_yaw = math.atan2(dy, dx)
        yaw_error = desired_yaw - self.dog_yaw
        # Normalize to [-pi, pi]
        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))
        yaw_rate = self.yaw_p_gain * yaw_error

        self._publish_dog_velocity(apf_vel[0], apf_vel[1], yaw_rate)

        # --- Safety metrics ---
        separation = np.linalg.norm(drone_2d - dog_2d)
        self.total_control_steps += 1
        self.min_separation = min(self.min_separation, separation)
        self.max_separation = max(self.max_separation, separation)
        if separation < self.apf.config.d_safe:
            self.safety_violations += 1

        # Periodic logging (every ~2 seconds at 20 Hz)
        if self.total_control_steps % 40 == 0:
            progress = self.planner.get_progress(t) * 100.0
            self.get_logger().info(
                f'[{progress:5.1f}%] sep={separation:.2f}m '
                f'drone=({self.drone_pos[0]:.1f},{self.drone_pos[1]:.1f},{self.drone_pos[2]:.1f}) '
                f'dog=({self.dog_pos[0]:.1f},{self.dog_pos[1]:.1f})')

    # ------------------------------------------------------------------
    # Summary
    # ------------------------------------------------------------------
    def _print_mission_summary(self):
        self.get_logger().info('=' * 50)
        self.get_logger().info('       MISSION SUMMARY')
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'  Total control steps : {self.total_control_steps}')
        elapsed = time.monotonic() - self.mission_start_time if self.mission_start_time else 0
        self.get_logger().info(f'  Mission duration    : {elapsed:.1f}s')
        self.get_logger().info(f'  Min separation      : {self.min_separation:.2f}m')
        self.get_logger().info(f'  Max separation      : {self.max_separation:.2f}m')
        self.get_logger().info(f'  APF min observed    : {self.apf.get_min_distance_observed():.2f}m')
        self.get_logger().info(f'  Safety violations   : {self.safety_violations} '
                               f'({self.safety_violations/max(self.total_control_steps,1)*100:.1f}%)')
        self.get_logger().info(f'  d_safe threshold    : {self.apf.config.d_safe:.1f}m')
        self.get_logger().info('=' * 50)

    # ------------------------------------------------------------------
    # Graceful shutdown
    # ------------------------------------------------------------------
    def shutdown(self):
        """Attempt to land drone on shutdown."""
        self.get_logger().info('Shutdown requested — landing drone...')
        self._stop_dog()
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
