"""
Artificial Potential Field (APF) controller for follower robot.

Computes velocity commands for a follower robot to maintain
a safe following distance from a leader using attractive and
repulsive potential fields.
"""

from dataclasses import dataclass
from typing import Tuple
import numpy as np


@dataclass
class APFConfig:
    """Configuration for APF controller."""
    k_att: float = 1.0              # Attractive force gain
    k_rep: float = 2.0              # Repulsive force gain
    d_safe: float = 2.0             # Minimum separation distance (meters)
    d_influence: float = 3.0        # Repulsion influence range (meters)
    max_velocity: float = 1.5       # Maximum output velocity (m/s)
    velocity_deadband: float = 0.1  # Ignore velocities below this (m/s)
    damping: float = 0.2            # Velocity damping factor for smoothing


class APFController:
    """
    APF-based follower controller.

    Uses attractive potential to pull follower toward leader,
    and repulsive potential to maintain minimum separation.

    Control Law:
        F_att = -k_att * (x_follower - x_leader)
        F_rep = k_rep * (1/d - 1/d_safe) * (1/d^2) * direction_away  (if d < d_safe)
        u = F_att + F_rep

    The repulsive force dominates at close range to prevent collisions.
    """

    def __init__(self, config: APFConfig = None):
        self.config = config or APFConfig()
        self._prev_velocity = np.array([0.0, 0.0])
        self._min_distance_observed = float('inf')

    def compute_velocity(
        self,
        leader_pos: np.ndarray,
        follower_pos: np.ndarray,
        follower_vel: np.ndarray = None
    ) -> np.ndarray:
        """
        Compute velocity command for follower based on APF.

        Args:
            leader_pos: [x, y] position of leader (drone)
            follower_pos: [x, y] position of follower (dog)
            follower_vel: [vx, vy] current velocity of follower (optional, for damping)

        Returns:
            velocity: [vx, vy] velocity command for follower
        """
        cfg = self.config

        # Ensure 2D
        leader_2d = np.array(leader_pos[:2], dtype=np.float64)
        follower_2d = np.array(follower_pos[:2], dtype=np.float64)

        # Vector from follower to leader
        to_leader = leader_2d - follower_2d
        distance = np.linalg.norm(to_leader)

        # Track minimum distance
        self._min_distance_observed = min(self._min_distance_observed, distance)

        # Handle zero distance
        if distance < 0.001:
            return np.array([0.0, 0.0])

        direction_to_leader = to_leader / distance

        # Attractive force (always active, linear)
        F_att = cfg.k_att * to_leader

        # Repulsive force (active when d < d_influence, stronger when d < d_safe)
        F_rep = np.array([0.0, 0.0])
        if distance < cfg.d_influence:
            if distance < cfg.d_safe:
                # Strong repulsion when inside safety zone
                # F_rep = k_rep * (1/d - 1/d_safe) * (1/d^2) * direction_away
                repulsion_magnitude = cfg.k_rep * (1.0/distance - 1.0/cfg.d_safe) * (1.0 / distance**2)
            else:
                # Gentle repulsion in influence zone (smooth transition)
                # Linear ramp from 0 at d_influence to small value at d_safe
                influence_factor = (cfg.d_influence - distance) / (cfg.d_influence - cfg.d_safe)
                repulsion_magnitude = cfg.k_rep * 0.1 * influence_factor

            # Direction away from leader
            F_rep = -repulsion_magnitude * direction_to_leader

        # Total force
        F_total = F_att + F_rep

        # Convert force to velocity command
        velocity = F_total

        # Apply damping if current velocity provided
        if follower_vel is not None and cfg.damping > 0:
            follower_vel_2d = np.array(follower_vel[:2], dtype=np.float64)
            # Damping term to reduce oscillation
            velocity = velocity - cfg.damping * follower_vel_2d

        # Smooth with previous velocity
        alpha = 0.7  # Smoothing factor
        velocity = alpha * velocity + (1 - alpha) * self._prev_velocity
        self._prev_velocity = velocity.copy()

        # Clamp to max velocity
        speed = np.linalg.norm(velocity)
        if speed > cfg.max_velocity:
            velocity = velocity / speed * cfg.max_velocity

        # Apply deadband
        if speed < cfg.velocity_deadband:
            velocity = np.array([0.0, 0.0])

        return velocity

    def compute_forces(
        self,
        leader_pos: np.ndarray,
        follower_pos: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray, float]:
        """
        Compute individual APF forces (for debugging/visualization).

        Args:
            leader_pos: [x, y] position of leader
            follower_pos: [x, y] position of follower

        Returns:
            F_att: Attractive force vector
            F_rep: Repulsive force vector
            distance: Current separation distance
        """
        cfg = self.config

        leader_2d = np.array(leader_pos[:2], dtype=np.float64)
        follower_2d = np.array(follower_pos[:2], dtype=np.float64)

        to_leader = leader_2d - follower_2d
        distance = np.linalg.norm(to_leader)

        if distance < 0.001:
            return np.array([0.0, 0.0]), np.array([0.0, 0.0]), 0.0

        direction_to_leader = to_leader / distance

        # Attractive force
        F_att = cfg.k_att * to_leader

        # Repulsive force
        F_rep = np.array([0.0, 0.0])
        if distance < cfg.d_safe:
            repulsion_magnitude = cfg.k_rep * (1.0/distance - 1.0/cfg.d_safe) * (1.0 / distance**2)
            F_rep = -repulsion_magnitude * direction_to_leader

        return F_att, F_rep, distance

    def get_separation_distance(
        self,
        leader_pos: np.ndarray,
        follower_pos: np.ndarray
    ) -> float:
        """Compute current separation distance."""
        leader_2d = np.array(leader_pos[:2])
        follower_2d = np.array(follower_pos[:2])
        return np.linalg.norm(leader_2d - follower_2d)

    def is_safe(
        self,
        leader_pos: np.ndarray,
        follower_pos: np.ndarray
    ) -> bool:
        """Check if follower is at safe distance from leader."""
        return self.get_separation_distance(leader_pos, follower_pos) >= self.config.d_safe

    def get_min_distance_observed(self) -> float:
        """Get minimum separation distance observed so far."""
        return self._min_distance_observed

    def reset(self):
        """Reset controller state."""
        self._prev_velocity = np.array([0.0, 0.0])
        self._min_distance_observed = float('inf')

    def __repr__(self) -> str:
        return (
            f"APFController("
            f"k_att={self.config.k_att}, "
            f"k_rep={self.config.k_rep}, "
            f"d_safe={self.config.d_safe}m, "
            f"max_vel={self.config.max_velocity}m/s)"
        )


if __name__ == "__main__":
    # Test the APF controller
    config = APFConfig(
        k_att=1.0,
        k_rep=2.0,
        d_safe=2.0,
        max_velocity=1.5,
    )
    controller = APFController(config)

    print(f"Controller: {controller}")
    print(f"\nTesting APF at various distances:")

    # Leader at (5, 5), follower at different positions
    leader_pos = np.array([5.0, 5.0])

    test_positions = [
        (5.0, 4.0),   # 1m away (inside d_safe)
        (5.0, 3.5),   # 1.5m away (inside d_safe)
        (5.0, 3.0),   # 2m away (at d_safe)
        (5.0, 2.0),   # 3m away (outside d_safe)
        (5.0, 0.0),   # 5m away (far)
    ]

    for fx, fy in test_positions:
        follower_pos = np.array([fx, fy])
        distance = controller.get_separation_distance(leader_pos, follower_pos)
        velocity = controller.compute_velocity(leader_pos, follower_pos)
        F_att, F_rep, d = controller.compute_forces(leader_pos, follower_pos)

        print(f"\n  Follower at ({fx}, {fy}), distance={distance:.2f}m:")
        print(f"    F_att: ({F_att[0]:.3f}, {F_att[1]:.3f})")
        print(f"    F_rep: ({F_rep[0]:.3f}, {F_rep[1]:.3f})")
        print(f"    velocity: ({velocity[0]:.3f}, {velocity[1]:.3f}) m/s")
        print(f"    safe: {controller.is_safe(leader_pos, follower_pos)}")

    print(f"\n  Min distance observed: {controller.get_min_distance_observed():.2f}m")
