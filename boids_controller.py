"""
Reynolds Boids swarm controller for multi-dog coordination.

Computes velocity commands for a boid (dog) based on three classic rules:
  1. Separation — steer away from nearby flockmates
  2. Alignment  — match the leader's velocity direction
  3. Cohesion   — steer toward the centroid of nearby agents
"""

from dataclasses import dataclass
from typing import List, Tuple
import numpy as np


@dataclass
class BoidsConfig:
    """Configuration for Boids controller."""
    w_separation: float = 2.0        # Separation force weight
    w_alignment: float = 1.0         # Alignment force weight
    w_cohesion: float = 1.0          # Cohesion force weight
    separation_radius: float = 2.5   # Distance within which separation activates (meters)
    neighbor_radius: float = 10.0    # Max distance to consider another agent a neighbor
    max_velocity: float = 1.5        # Maximum output velocity (m/s)
    velocity_deadband: float = 0.1   # Ignore velocities below this (m/s)
    smoothing: float = 0.7           # Exponential smoothing factor (0-1)


class BoidsController:
    """
    Reynolds Boids controller for a single boid (dog).

    Each dog gets its own controller instance so smoothing state
    is independent.

    Steering forces:
        F_sep = sum( (self - neighbor) / dist^2 )   for neighbors within separation_radius
        F_ali = normalize(leader_vel)                match leader heading
        F_coh = centroid - self_pos                   steer toward group center

        velocity = w_sep * F_sep + w_ali * F_ali + w_coh * F_coh
    """

    def __init__(self, config: BoidsConfig = None):
        self.config = config or BoidsConfig()
        self._prev_velocity = np.array([0.0, 0.0])
        self._min_distance_observed = float('inf')

    def compute_velocity(
        self,
        agent_pos: np.ndarray,
        neighbors: List[Tuple[np.ndarray, np.ndarray]],
        leader_pos: np.ndarray,
        leader_vel: np.ndarray,
    ) -> np.ndarray:
        """
        Compute velocity command for this boid.

        Args:
            agent_pos: [x, y] position of this boid
            neighbors: list of (pos_2d, vel_2d) tuples for other boids
            leader_pos: [x, y] drone ground-projected position
            leader_vel: [vx, vy] drone velocity (for alignment)

        Returns:
            velocity: [vx, vy] command in world frame
        """
        cfg = self.config

        agent_2d = np.array(agent_pos[:2], dtype=np.float64)
        leader_2d = np.array(leader_pos[:2], dtype=np.float64)
        leader_v2d = np.array(leader_vel[:2], dtype=np.float64)

        # --- Separation ---
        F_sep = np.array([0.0, 0.0])
        all_neighbors = [(np.array(p[:2], dtype=np.float64), np.array(v[:2], dtype=np.float64))
                         for p, v in neighbors]
        # Include drone ground position as a separation source
        all_separation_positions = [p for p, _ in all_neighbors] + [leader_2d]

        for n_pos in all_separation_positions:
            diff = agent_2d - n_pos
            dist = np.linalg.norm(diff)
            self._min_distance_observed = min(self._min_distance_observed, dist)
            if 0.001 < dist < cfg.separation_radius:
                F_sep += diff / (dist * dist)

        # --- Alignment ---
        F_ali = np.array([0.0, 0.0])
        leader_speed = np.linalg.norm(leader_v2d)
        if leader_speed > 0.01:
            F_ali = leader_v2d / leader_speed

        # --- Cohesion ---
        # Leader weighted 2x relative to other boids
        weight_leader = 2.0
        weight_boid = 1.0
        centroid = weight_leader * leader_2d
        total_weight = weight_leader
        for n_pos, _ in all_neighbors:
            dist = np.linalg.norm(agent_2d - n_pos)
            if dist < cfg.neighbor_radius:
                centroid += weight_boid * n_pos
                total_weight += weight_boid
        centroid /= total_weight
        F_coh = centroid - agent_2d

        # --- Weighted sum ---
        velocity = (cfg.w_separation * F_sep +
                    cfg.w_alignment * F_ali +
                    cfg.w_cohesion * F_coh)

        # --- Smoothing ---
        velocity = cfg.smoothing * velocity + (1.0 - cfg.smoothing) * self._prev_velocity
        self._prev_velocity = velocity.copy()

        # --- Clamp ---
        speed = np.linalg.norm(velocity)
        if speed > cfg.max_velocity:
            velocity = velocity / speed * cfg.max_velocity

        # --- Deadband ---
        if speed < cfg.velocity_deadband:
            velocity = np.array([0.0, 0.0])

        return velocity

    def get_separation_distance(
        self,
        pos_a: np.ndarray,
        pos_b: np.ndarray,
    ) -> float:
        """Compute distance between two 2D positions."""
        a = np.array(pos_a[:2])
        b = np.array(pos_b[:2])
        return float(np.linalg.norm(a - b))

    def get_min_distance_observed(self) -> float:
        """Get minimum separation distance observed so far."""
        return self._min_distance_observed

    def reset(self):
        """Reset controller state."""
        self._prev_velocity = np.array([0.0, 0.0])
        self._min_distance_observed = float('inf')

    def __repr__(self) -> str:
        return (
            f"BoidsController("
            f"w_sep={self.config.w_separation}, "
            f"w_ali={self.config.w_alignment}, "
            f"w_coh={self.config.w_cohesion}, "
            f"sep_r={self.config.separation_radius}m, "
            f"max_vel={self.config.max_velocity}m/s)"
        )


if __name__ == "__main__":
    config = BoidsConfig(
        w_separation=2.0,
        w_alignment=1.0,
        w_cohesion=1.0,
        separation_radius=2.5,
        max_velocity=1.5,
    )
    controller = BoidsController(config)
    print(f"Controller: {controller}")

    print("\nTesting Boids at various configurations:")

    leader_pos = np.array([5.0, 5.0])
    leader_vel = np.array([1.0, 0.0])

    test_cases = [
        ("Dog far from leader, no other boids",
         np.array([0.0, 0.0]), [], leader_pos, leader_vel),
        ("Dog near leader, no other boids",
         np.array([4.0, 5.0]), [], leader_pos, leader_vel),
        ("Two dogs, well separated",
         np.array([3.0, 3.0]),
         [(np.array([3.0, 7.0]), np.array([0.5, 0.0]))],
         leader_pos, leader_vel),
        ("Two dogs, close together (separation should activate)",
         np.array([3.0, 5.0]),
         [(np.array([3.5, 5.0]), np.array([0.5, 0.0]))],
         leader_pos, leader_vel),
    ]

    for desc, agent_pos, neighbors, l_pos, l_vel in test_cases:
        controller.reset()
        vel = controller.compute_velocity(agent_pos, neighbors, l_pos, l_vel)
        print(f"\n  {desc}")
        print(f"    Agent at ({agent_pos[0]:.1f}, {agent_pos[1]:.1f})")
        print(f"    Velocity: ({vel[0]:.3f}, {vel[1]:.3f}) m/s")
        print(f"    Min dist observed: {controller.get_min_distance_observed():.2f}m")

    print(f"\n  Final min distance: {controller.get_min_distance_observed():.2f}m")
