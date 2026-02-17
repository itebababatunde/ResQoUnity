#!/usr/bin/env python3
"""
Boids Swarm Coordination ROS2 Node.

Standalone node that coordinates a drone (leader) and two Go2 dogs (boids)
using Reynolds' Boids flocking rules (separation, alignment, cohesion).

The simulation (run_sim.sh) must already be running with 2 dogs + drone spawned.

Usage:
    Terminal 1: ./run_sim.sh
    Terminal 2: ./run_boids_ros2.sh
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
from boids_controller import BoidsController, BoidsConfig


NUM_DOGS = 2


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


class BoidsSwarmNode(Node):
    def __init__(self):
        super().__init__('boids_swarm_node')

        # Declare ROS2 parameters with defaults
        self.declare_parameter('workspace_size', 10.0)
        self.declare_parameter('sweep_spacing', 2.0)
        self.declare_parameter('drone_speed', 1.0)
        self.declare_parameter('drone_altitude', 3.0)
        self.declare_parameter('w_separation', 2.0)
        self.declare_parameter('w_alignment', 1.0)
        self.declare_parameter('w_cohesion', 1.0)
        self.declare_parameter('separation_radius', 2.5)
        self.declare_parameter('neighbor_radius', 10.0)
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
        w_sep = self.get_parameter('w_separation').value
        w_ali = self.get_parameter('w_alignment').value
        w_coh = self.get_parameter('w_cohesion').value
        sep_r = self.get_parameter('separation_radius').value
        neigh_r = self.get_parameter('neighbor_radius').value
        max_vel = self.get_parameter('max_velocity').value
        self.yaw_p_gain = self.get_parameter('yaw_p_gain').value
        self.altitude_wait_sec = self.get_parameter('altitude_wait_sec').value
        self.completion_hover_sec = self.get_parameter('completion_hover_sec').value
        control_hz = self.get_parameter('control_rate_hz').value

        # Planner
        planner_cfg = LawnmowerConfig(
            workspace_size=ws,
            sweep_spacing=spacing,
            drone_speed=speed,
            drone_altitude=alt,
        )
        self.planner = LawnmowerPlanner(planner_cfg)

        # One BoidsController per dog (independent smoothing state)
        boids_cfg = BoidsConfig(
            w_separation=w_sep,
            w_alignment=w_ali,
            w_cohesion=w_coh,
            separation_radius=sep_r,
            neighbor_radius=neigh_r,
            max_velocity=max_vel,
        )
        self.boids = [BoidsController(BoidsConfig(
            w_separation=w_sep,
            w_alignment=w_ali,
            w_cohesion=w_coh,
            separation_radius=sep_r,
            neighbor_radius=neigh_r,
            max_velocity=max_vel,
        )) for _ in range(NUM_DOGS)]

        self.get_logger().info(f'Planner: {self.planner}')
        self.get_logger().info(f'Boids[0]: {self.boids[0]}')
        self.get_logger().info(f'Total mission time: {self.planner.total_time:.1f}s')

        # State machine
        self.state = MissionState.INIT
        self.mission_start_time = None
        self.state_enter_time = None

        # Drone odometry state
        self.drone_pos = None       # [x, y, z]
        self.drone_quat = None      # [x, y, z, w]
        self.drone_odom_received = False

        # Per-dog state arrays
        self.dog_pos = [None] * NUM_DOGS         # [x, y, z]
        self.dog_quat = [None] * NUM_DOGS        # (x, y, z, w)
        self.dog_yaw = [0.0] * NUM_DOGS
        self.dog_odom_received = [False] * NUM_DOGS
        self.dog_prev_pos = [None] * NUM_DOGS
        self.dog_prev_time = [None] * NUM_DOGS
        self.dog_vel_estimate = [np.array([0.0, 0.0]) for _ in range(NUM_DOGS)]

        # Safety tracking (per-dog)
        self.min_drone_sep = [float('inf')] * NUM_DOGS
        self.max_drone_sep = [0.0] * NUM_DOGS
        self.min_inter_dog_distance = float('inf')
        self.safety_violations = 0
        self.total_control_steps = 0

        # Subscribers (store references to prevent GC)
        qos = QoSProfile(depth=10)
        self._subs = []
        self._subs.append(
            self.create_subscription(Odometry, '/drone/odom', self._drone_odom_cb, qos))
        for i in range(NUM_DOGS):
            self._subs.append(
                self.create_subscription(
                    Odometry, f'/robot{i}/odom',
                    lambda msg, idx=i: self._dog_odom_cb(msg, idx), qos))

        # Publishers
        self.drone_cmd_pub = self.create_publisher(PoseStamped, '/drone/cmd_position', qos)
        self.dog_cmd_pubs = [
            self.create_publisher(Twist, f'/robot{i}/cmd_vel', qos)
            for i in range(NUM_DOGS)
        ]

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
        self._init_start_time = time.monotonic()
        self._init_last_log = 0.0

        self.get_logger().info(
            f'Boids Swarm node initialized ({NUM_DOGS} dogs). Waiting for odometry...')

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

    def _dog_odom_cb(self, msg: Odometry, idx: int):
        p = msg.pose.pose.position
        self.dog_pos[idx] = np.array([p.x, p.y, p.z])
        q = msg.pose.pose.orientation
        self.dog_quat[idx] = (q.x, q.y, q.z, q.w)
        self.dog_yaw[idx] = quat_to_yaw(q.x, q.y, q.z, q.w)

        # Estimate velocity by differentiating position
        now = time.monotonic()
        if self.dog_prev_pos[idx] is not None and self.dog_prev_time[idx] is not None:
            dt = now - self.dog_prev_time[idx]
            if dt > 0.001:
                self.dog_vel_estimate[idx] = (
                    self.dog_pos[idx][:2] - self.dog_prev_pos[idx][:2]) / dt
        self.dog_prev_pos[idx] = self.dog_pos[idx].copy()
        self.dog_prev_time[idx] = now

        if not self.dog_odom_received[idx]:
            self.dog_odom_received[idx] = True
            self.get_logger().info(
                f'Dog{idx} odom received: ({p.x:.2f}, {p.y:.2f}, {p.z:.2f})')

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

    def _publish_dog_velocity(self, idx, world_vx, world_vy, yaw_rate):
        """Convert world-frame velocity to body frame and publish Twist."""
        cos_yaw = math.cos(self.dog_yaw[idx])
        sin_yaw = math.sin(self.dog_yaw[idx])
        body_vx = cos_yaw * world_vx + sin_yaw * world_vy
        body_vy = -sin_yaw * world_vx + cos_yaw * world_vy

        msg = Twist()
        msg.linear.x = float(body_vx)
        msg.linear.y = float(body_vy)
        msg.angular.z = float(yaw_rate)
        self.dog_cmd_pubs[idx].publish(msg)

    def _stop_all_dogs(self):
        """Publish zero velocity to all dogs."""
        for pub in self.dog_cmd_pubs:
            pub.publish(Twist())

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

    def _all_odom_received(self):
        return self.drone_odom_received and all(self.dog_odom_received)

    def _control_loop(self):
        """Main 20 Hz control loop with state machine."""

        # ---- INIT: wait for all odom topics ----
        if self.state == MissionState.INIT:
            if self._all_odom_received():
                self.get_logger().info('Odometry received from all 3 robots')
                self._transition(MissionState.ARMING)
                return

            # Periodic diagnostics every 5 seconds
            elapsed = time.monotonic() - self._init_start_time
            if elapsed - self._init_last_log >= 5.0:
                self._init_last_log = elapsed
                missing = []
                if not self.drone_odom_received:
                    missing.append('/drone/odom')
                for i in range(NUM_DOGS):
                    if not self.dog_odom_received[i]:
                        missing.append(f'/robot{i}/odom')
                self.get_logger().warn(
                    f'Still waiting for odom ({elapsed:.0f}s): {", ".join(missing)}. '
                    f'Check ros2 topic list / simulation.')
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

        # ---- RUNNING: main boids swarm loop ----
        if self.state == MissionState.RUNNING:
            self._run_boids_swarm()
            return

        # ---- COMPLETING: brief hover before landing ----
        if self.state == MissionState.COMPLETING:
            self._stop_all_dogs()
            if self._time_in_state() > self.completion_hover_sec:
                self._print_mission_summary()
                self._transition(MissionState.LANDING)
            return

        # ---- LANDING: call /drone/land ----
        if self.state == MissionState.LANDING:
            self._stop_all_dogs()
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
            self._stop_all_dogs()
            return

    # ------------------------------------------------------------------
    # Boids swarm control
    # ------------------------------------------------------------------
    def _run_boids_swarm(self):
        if self.drone_pos is None or any(p is None for p in self.dog_pos):
            return

        t = time.monotonic() - self.mission_start_time

        # Check completion
        if self.planner.is_complete(t):
            self.get_logger().info('Lawnmower sweep complete!')
            self._stop_all_dogs()
            self._transition(MissionState.COMPLETING)
            return

        # --- Drone: publish next waypoint from planner ---
        target_pos = self.planner.get_position(t)
        self._publish_drone_position(target_pos)

        # --- Drone velocity from planner (deterministic) ---
        drone_vel_3d = self.planner.get_velocity(t)
        drone_pos_2d = self.drone_pos[:2]
        drone_vel_2d = drone_vel_3d[:2]

        # --- Each dog: compute boids velocity ---
        self.total_control_steps += 1

        for i in range(NUM_DOGS):
            dog_i_pos = self.dog_pos[i][:2]

            # Build neighbor list: other dogs (not self)
            neighbors = []
            for j in range(NUM_DOGS):
                if j != i:
                    neighbors.append((self.dog_pos[j][:2], self.dog_vel_estimate[j]))

            # Compute boids velocity
            boids_vel = self.boids[i].compute_velocity(
                dog_i_pos, neighbors, drone_pos_2d, drone_vel_2d)

            # Yaw rate: P-control to orient toward drone
            dx = self.drone_pos[0] - self.dog_pos[i][0]
            dy = self.drone_pos[1] - self.dog_pos[i][1]
            desired_yaw = math.atan2(dy, dx)
            yaw_error = desired_yaw - self.dog_yaw[i]
            yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))
            yaw_rate = self.yaw_p_gain * yaw_error

            self._publish_dog_velocity(i, boids_vel[0], boids_vel[1], yaw_rate)

            # Per-dog safety metrics
            drone_sep = np.linalg.norm(drone_pos_2d - dog_i_pos)
            self.min_drone_sep[i] = min(self.min_drone_sep[i], drone_sep)
            self.max_drone_sep[i] = max(self.max_drone_sep[i], drone_sep)

        # Inter-dog distance
        inter_dog_dist = np.linalg.norm(self.dog_pos[0][:2] - self.dog_pos[1][:2])
        self.min_inter_dog_distance = min(self.min_inter_dog_distance, inter_dog_dist)
        if inter_dog_dist < self.boids[0].config.separation_radius:
            self.safety_violations += 1

        # Periodic logging (every ~2 seconds at 20 Hz)
        if self.total_control_steps % 40 == 0:
            progress = self.planner.get_progress(t) * 100.0
            d0_sep = np.linalg.norm(drone_pos_2d - self.dog_pos[0][:2])
            d1_sep = np.linalg.norm(drone_pos_2d - self.dog_pos[1][:2])
            self.get_logger().info(
                f'[{progress:5.1f}%] dog_sep={inter_dog_dist:.2f}m '
                f'd0_drone={d0_sep:.2f}m d1_drone={d1_sep:.2f}m '
                f'drone=({self.drone_pos[0]:.1f},{self.drone_pos[1]:.1f},{self.drone_pos[2]:.1f})')

    # ------------------------------------------------------------------
    # Summary
    # ------------------------------------------------------------------
    def _print_mission_summary(self):
        self.get_logger().info('=' * 55)
        self.get_logger().info('          BOIDS SWARM MISSION SUMMARY')
        self.get_logger().info('=' * 55)
        self.get_logger().info(f'  Total control steps   : {self.total_control_steps}')
        elapsed = time.monotonic() - self.mission_start_time if self.mission_start_time else 0
        self.get_logger().info(f'  Mission duration      : {elapsed:.1f}s')
        for i in range(NUM_DOGS):
            self.get_logger().info(
                f'  Dog{i} min/max drone sep: '
                f'{self.min_drone_sep[i]:.2f}m / {self.max_drone_sep[i]:.2f}m')
            self.get_logger().info(
                f'  Dog{i} boids min observed: '
                f'{self.boids[i].get_min_distance_observed():.2f}m')
        self.get_logger().info(f'  Min inter-dog distance: {self.min_inter_dog_distance:.2f}m')
        self.get_logger().info(f'  Safety violations     : {self.safety_violations} '
                               f'({self.safety_violations / max(self.total_control_steps, 1) * 100:.1f}%)')
        self.get_logger().info(f'  Separation radius     : {self.boids[0].config.separation_radius:.1f}m')
        self.get_logger().info('=' * 55)

    # ------------------------------------------------------------------
    # Graceful shutdown
    # ------------------------------------------------------------------
    def shutdown(self):
        """Attempt to land drone on shutdown."""
        self.get_logger().info('Shutdown requested — landing drone...')
        self._stop_all_dogs()
        if self.land_client.service_is_ready():
            future = self.land_client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        self.get_logger().info('Shutdown complete.')


def main(args=None):
    rclpy.init(args=args)
    node = BoidsSwarmNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.shutdown()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
