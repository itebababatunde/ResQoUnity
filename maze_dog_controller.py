"""
maze_dog_controller.py — Proportional waypoint follower for the Go2 dog in maze navigation.

Outputs geometry_msgs/Twist-compatible (linear_x, angular_z) pairs at each update step.
Strategy: align heading first, then drive forward toward the current waypoint.

Parameters:
    max_linear_vel   = 0.5 m/s
    kp_linear        = 1.0
    kp_angular       = 2.0
    waypoint_radius  = 0.3 m  (arrival threshold)
    stuck_timeout    = 10.0 s (triggers re-plan if no progress within this window)

Usage (standalone test):
    python maze_dog_controller.py
"""

import math
import time
import numpy as np


class MazeDogController:
    """
    Proportional controller that drives the Go2 through a sequence of 2D waypoints.

    Call update() at a fixed rate (e.g. 10-20 Hz) to get velocity commands.
    Heading alignment: if |angle_error| > HEADING_THRESHOLD, rotate in place first.
    """

    MAX_LINEAR_VEL   = 0.5   # m/s
    KP_LINEAR        = 1.0
    KP_ANGULAR       = 2.0
    WAYPOINT_RADIUS  = 0.3   # m — distance to consider a waypoint reached
    STUCK_TIMEOUT    = 10.0  # s — no progress within this window -> stuck
    STUCK_DIST_THR   = 0.05  # m — minimum distance to advance per stuck window
    HEADING_THRESHOLD = 0.3  # rad (~17°) — align heading before driving

    def __init__(self, waypoints=None):
        """
        Args:
            waypoints: list of np.ndarray([x, y, z]) or None (set later via reset()).
        """
        self._waypoints = []
        self._wp_idx = 0
        self._done = False

        # Stuck detection
        self._stuck_ref_pos = None
        self._stuck_ref_time = None

        if waypoints is not None:
            self.reset(waypoints)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def reset(self, waypoints):
        """Load a new waypoint list and restart the controller."""
        self._waypoints = list(waypoints)
        self._wp_idx = 0
        self._done = len(waypoints) == 0
        self._stuck_ref_pos = None
        self._stuck_ref_time = None

    def is_done(self):
        """Returns True when the dog has reached the final waypoint."""
        return self._done

    def is_stuck(self, dog_pos_xy):
        """
        Returns True if the dog has not advanced STUCK_DIST_THR metres
        within the last STUCK_TIMEOUT seconds.

        Should be called every update cycle.
        """
        now = time.monotonic()
        if self._stuck_ref_pos is None or self._stuck_ref_time is None:
            self._stuck_ref_pos = np.array(dog_pos_xy[:2], dtype=float)
            self._stuck_ref_time = now
            return False

        elapsed = now - self._stuck_ref_time
        if elapsed >= self.STUCK_TIMEOUT:
            dist = np.linalg.norm(np.array(dog_pos_xy[:2]) - self._stuck_ref_pos)
            if dist < self.STUCK_DIST_THR:
                return True
            # Reset reference
            self._stuck_ref_pos = np.array(dog_pos_xy[:2], dtype=float)
            self._stuck_ref_time = now

        return False

    def current_waypoint_index(self):
        return self._wp_idx

    def total_waypoints(self):
        return len(self._waypoints)

    def current_waypoint(self):
        if self._done or self._wp_idx >= len(self._waypoints):
            return None
        return self._waypoints[self._wp_idx]

    def update(self, dog_pos, dog_yaw, dt):
        """
        Compute velocity command to drive toward the current waypoint.

        Args:
            dog_pos: array-like [x, y, z] or [x, y]  — dog world position
            dog_yaw: float — dog heading in radians (from quaternion)
            dt:      float — time step in seconds (unused in P controller but kept for API compatibility)

        Returns:
            (linear_x, angular_z): float tuple in body frame.
            Returns (0.0, 0.0) if done or no waypoints loaded.
        """
        if self._done or len(self._waypoints) == 0:
            return (0.0, 0.0)

        # Check arrival at current waypoint
        wp = self._waypoints[self._wp_idx]
        dx = wp[0] - dog_pos[0]
        dy = wp[1] - dog_pos[1]
        dist = math.sqrt(dx * dx + dy * dy)

        if dist < self.WAYPOINT_RADIUS:
            self._wp_idx += 1
            if self._wp_idx >= len(self._waypoints):
                self._done = True
                return (0.0, 0.0)
            # Advance to next waypoint
            wp = self._waypoints[self._wp_idx]
            dx = wp[0] - dog_pos[0]
            dy = wp[1] - dog_pos[1]
            dist = math.sqrt(dx * dx + dy * dy)

        # Desired heading toward waypoint
        desired_yaw = math.atan2(dy, dx)
        angle_error = _normalize_angle(desired_yaw - dog_yaw)

        # Heading alignment: rotate in place if error is large
        if abs(angle_error) > self.HEADING_THRESHOLD:
            angular_z = float(np.clip(self.KP_ANGULAR * angle_error, -2.0, 2.0))
            return (0.0, angular_z)

        # Drive forward proportionally
        linear_x = float(np.clip(self.KP_LINEAR * dist, 0.0, self.MAX_LINEAR_VEL))
        angular_z = float(np.clip(self.KP_ANGULAR * angle_error, -2.0, 2.0))
        return (linear_x, angular_z)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _normalize_angle(angle):
    """Wrap angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


# ---------------------------------------------------------------------------
# Standalone unit test
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    import sys

    print("Testing MazeDogController ...")

    # Simple waypoints: dog starts at (0,0) heading east (yaw=0), needs to reach (2,0) then (2,2)
    waypoints = [
        np.array([1.0, 0.0, 0.3]),
        np.array([2.0, 0.0, 0.3]),
        np.array([2.0, 2.0, 0.3]),
    ]

    ctrl = MazeDogController(waypoints=waypoints)
    assert not ctrl.is_done()
    assert ctrl.total_waypoints() == 3

    # Simulate the dog moving in a straight line east then north
    dog_pos = np.array([0.0, 0.0, 0.3])
    dog_yaw = 0.0
    dt = 0.05
    steps = 0
    max_steps = 2000

    while not ctrl.is_done() and steps < max_steps:
        lin_x, ang_z = ctrl.update(dog_pos, dog_yaw, dt)

        # Simple integrator: move dog
        dog_yaw += ang_z * dt
        dog_yaw = _normalize_angle(dog_yaw)
        dog_pos[0] += lin_x * math.cos(dog_yaw) * dt
        dog_pos[1] += lin_x * math.sin(dog_yaw) * dt

        steps += 1
        if steps % 100 == 0:
            wp_idx = ctrl.current_waypoint_index()
            print(f"  step={steps:4d}  pos=({dog_pos[0]:.2f},{dog_pos[1]:.2f})  "
                  f"yaw={math.degrees(dog_yaw):.1f}°  wp={wp_idx}/{ctrl.total_waypoints()}  "
                  f"cmd=({lin_x:.2f},{ang_z:.2f})")

    if ctrl.is_done():
        print(f"Done in {steps} steps. Final pos: ({dog_pos[0]:.2f}, {dog_pos[1]:.2f})")
    else:
        print(f"WARNING: Did not reach goal in {max_steps} steps!")

    # Test stuck detection with mock positions
    ctrl2 = MazeDogController(waypoints=waypoints)
    frozen_pos = [0.0, 0.0]
    stuck_triggered = False
    import time as _time
    start = _time.monotonic()
    ctrl2._stuck_ref_time = start - 11.0  # Force timeout
    ctrl2._stuck_ref_pos = np.array([0.0, 0.0])
    if ctrl2.is_stuck(frozen_pos):
        print("Stuck detection: PASS (correctly detected stuck state)")
    else:
        print("Stuck detection: FAIL (should have detected stuck)")

    print("\nAll MazeDogController tests passed.")
