"""
Lawnmower trajectory planner for drone leader.

Generates a deterministic lawnmower (boustrophedon) sweep pattern
for full coverage of a square workspace.
"""

from dataclasses import dataclass
from typing import List, Tuple
import numpy as np


@dataclass
class LawnmowerConfig:
    """Configuration for lawnmower trajectory."""
    workspace_size: float = 10.0      # Square side length (meters)
    sweep_spacing: float = 2.0        # Distance between rows (meters)
    drone_speed: float = 1.0          # Traversal speed (m/s)
    drone_altitude: float = 3.0       # Fixed altitude above ground (meters)
    start_corner: Tuple[float, float] = (0.0, 0.0)  # Starting position (x, y)


class LawnmowerPlanner:
    """
    Generates lawnmower sweep trajectory for area coverage.

    The pattern alternates direction on each row:
    Row 0: left to right  (y=0)
    Row 1: right to left  (y=spacing)
    Row 2: left to right  (y=2*spacing)
    ...
    """

    def __init__(self, config: LawnmowerConfig = None):
        self.config = config or LawnmowerConfig()
        self.waypoints = self._generate_waypoints()
        self.total_distance = self._compute_total_distance()
        self.total_time = self.total_distance / self.config.drone_speed
        self._current_segment = 0

    def _generate_waypoints(self) -> List[np.ndarray]:
        """Generate waypoints for lawnmower pattern."""
        waypoints = []
        cfg = self.config

        x_start = cfg.start_corner[0]
        y_start = cfg.start_corner[1]
        x_end = x_start + cfg.workspace_size

        num_rows = int(np.ceil(cfg.workspace_size / cfg.sweep_spacing)) + 1

        for row in range(num_rows):
            y = y_start + row * cfg.sweep_spacing
            # Clamp to workspace boundary
            y = min(y, y_start + cfg.workspace_size)

            if row % 2 == 0:
                # Even rows: left to right
                waypoints.append(np.array([x_start, y, cfg.drone_altitude]))
                waypoints.append(np.array([x_end, y, cfg.drone_altitude]))
            else:
                # Odd rows: right to left
                waypoints.append(np.array([x_end, y, cfg.drone_altitude]))
                waypoints.append(np.array([x_start, y, cfg.drone_altitude]))

        # Remove duplicate consecutive waypoints
        filtered = [waypoints[0]]
        for wp in waypoints[1:]:
            if not np.allclose(wp, filtered[-1]):
                filtered.append(wp)

        return filtered

    def _compute_total_distance(self) -> float:
        """Compute total path length."""
        total = 0.0
        for i in range(len(self.waypoints) - 1):
            total += np.linalg.norm(self.waypoints[i + 1] - self.waypoints[i])
        return total

    def get_position(self, t: float) -> np.ndarray:
        """
        Get drone position at time t.

        Args:
            t: Time since start (seconds)

        Returns:
            position: [x, y, z] position in world frame
        """
        if t <= 0:
            return self.waypoints[0].copy()

        if t >= self.total_time:
            return self.waypoints[-1].copy()

        # Find which segment we're in
        distance_traveled = t * self.config.drone_speed
        cumulative_dist = 0.0

        for i in range(len(self.waypoints) - 1):
            segment_start = self.waypoints[i]
            segment_end = self.waypoints[i + 1]
            segment_length = np.linalg.norm(segment_end - segment_start)

            if cumulative_dist + segment_length >= distance_traveled:
                # We're in this segment
                segment_progress = distance_traveled - cumulative_dist
                if segment_length > 0:
                    alpha = segment_progress / segment_length
                else:
                    alpha = 0.0
                return segment_start + alpha * (segment_end - segment_start)

            cumulative_dist += segment_length

        return self.waypoints[-1].copy()

    def get_velocity(self, t: float) -> np.ndarray:
        """
        Get desired drone velocity at time t.

        Args:
            t: Time since start (seconds)

        Returns:
            velocity: [vx, vy, vz] velocity in world frame
        """
        if t < 0 or t >= self.total_time:
            return np.array([0.0, 0.0, 0.0])

        # Find which segment we're in
        distance_traveled = t * self.config.drone_speed
        cumulative_dist = 0.0

        for i in range(len(self.waypoints) - 1):
            segment_start = self.waypoints[i]
            segment_end = self.waypoints[i + 1]
            segment_length = np.linalg.norm(segment_end - segment_start)

            if cumulative_dist + segment_length >= distance_traveled:
                # We're in this segment
                if segment_length > 0:
                    direction = (segment_end - segment_start) / segment_length
                    return direction * self.config.drone_speed
                else:
                    return np.array([0.0, 0.0, 0.0])

            cumulative_dist += segment_length

        return np.array([0.0, 0.0, 0.0])

    def is_complete(self, t: float) -> bool:
        """Check if traversal is complete."""
        return t >= self.total_time

    def get_progress(self, t: float) -> float:
        """Get coverage progress as fraction [0, 1]."""
        if t <= 0:
            return 0.0
        if t >= self.total_time:
            return 1.0
        return t / self.total_time

    def get_waypoints(self) -> List[np.ndarray]:
        """Get all waypoints for visualization."""
        return self.waypoints.copy()

    def __repr__(self) -> str:
        return (
            f"LawnmowerPlanner("
            f"workspace={self.config.workspace_size}m, "
            f"rows={len(self.waypoints)//2}, "
            f"total_distance={self.total_distance:.1f}m, "
            f"total_time={self.total_time:.1f}s)"
        )


if __name__ == "__main__":
    # Test the planner
    config = LawnmowerConfig(
        workspace_size=10.0,
        sweep_spacing=2.0,
        drone_speed=1.0,
        drone_altitude=3.0,
    )
    planner = LawnmowerPlanner(config)

    print(f"Planner: {planner}")
    print(f"\nWaypoints:")
    for i, wp in enumerate(planner.waypoints):
        print(f"  {i}: ({wp[0]:.1f}, {wp[1]:.1f}, {wp[2]:.1f})")

    print(f"\nTrajectory samples:")
    for t in np.linspace(0, planner.total_time, 10):
        pos = planner.get_position(t)
        vel = planner.get_velocity(t)
        print(f"  t={t:.1f}s: pos=({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}), "
              f"vel=({vel[0]:.2f}, {vel[1]:.2f}, {vel[2]:.2f}), "
              f"progress={planner.get_progress(t)*100:.0f}%")
