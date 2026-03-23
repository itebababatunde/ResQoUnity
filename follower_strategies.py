"""
Follower strategies for leader-follower coordination.

Each strategy computes attractor targets for the two dogs given
the current drone state. The APF controller then drives each dog
toward its assigned target.

Usage:
    strategy = get_strategy('lateral_offset', workspace_size=10.0, sweep_spacing=2.0)
    target0, target1 = strategy.get_targets(drone_pos, drone_vel, dog0_pos, dog1_pos)
"""

from abc import ABC, abstractmethod
from collections import deque
import numpy as np


class FollowerStrategy(ABC):
    """Base class for follower target strategies."""

    name: str = 'base'

    @abstractmethod
    def get_targets(
        self,
        drone_pos: np.ndarray,   # [x, y, z]
        drone_vel: np.ndarray,   # [vx, vy, vz]
        dog0_pos: np.ndarray,    # [x, y, z]
        dog1_pos: np.ndarray,    # [x, y, z]
    ) -> tuple:
        """
        Returns:
            (target0, target1): 2D attractor positions [x, y] for each dog.
        """
        ...

    def __repr__(self):
        return f'{self.__class__.__name__}()'


class NaiveFollow(FollowerStrategy):
    """
    Both dogs follow the drone position directly.
    This is the original behavior — produces high overlap.
    """

    name = 'naive'

    def get_targets(self, drone_pos, drone_vel, dog0_pos, dog1_pos):
        target = drone_pos[:2]
        return target.copy(), target.copy()


class LateralOffset(FollowerStrategy):
    """
    Dogs follow offset targets perpendicular to the drone's travel direction.
    Dog0 goes left, Dog1 goes right (relative to drone heading).
    This spreads coverage across parallel strips.
    """

    name = 'lateral_offset'

    def __init__(self, offset_distance: float = 2.0,
                 workspace_origin: tuple = (0.0, 0.0),
                 workspace_size: float = 10.0):
        self.offset_distance = offset_distance
        self.ws_min = np.array(workspace_origin)
        self.ws_max = np.array(workspace_origin) + workspace_size
        self._last_perp = np.array([0.0, 1.0])

    def get_targets(self, drone_pos, drone_vel, dog0_pos, dog1_pos):
        drone_2d = drone_pos[:2]
        vel_2d = drone_vel[:2]
        speed = np.linalg.norm(vel_2d)

        if speed > 0.05:
            direction = vel_2d / speed
            # Perpendicular: rotate 90 degrees CCW
            perp = np.array([-direction[1], direction[0]])
            self._last_perp = perp
        else:
            perp = self._last_perp

        offset = self.offset_distance * perp
        target0 = drone_2d + offset     # left
        target1 = drone_2d - offset     # right

        # Clamp to workspace bounds
        target0 = np.clip(target0, self.ws_min, self.ws_max)
        target1 = np.clip(target1, self.ws_min, self.ws_max)

        return target0, target1

    def __repr__(self):
        return f'LateralOffset(offset={self.offset_distance}m)'


class TrailingOffset(FollowerStrategy):
    """
    Dog0 follows the drone's current position.
    Dog1 follows where the drone was N seconds ago (trailing).
    Reduces overlap on straight segments.
    """

    name = 'trailing'

    def __init__(self, trail_delay: float = 3.0, dt: float = 0.05):
        self.trail_delay = trail_delay
        self.dt = dt
        buffer_size = int(trail_delay / dt) + 1
        self._history = deque(maxlen=buffer_size)

    def get_targets(self, drone_pos, drone_vel, dog0_pos, dog1_pos):
        drone_2d = drone_pos[:2].copy()
        self._history.append(drone_2d)

        target0 = drone_2d
        # Dog1 follows the oldest position in the buffer
        target1 = self._history[0]

        return target0, target1

    def __repr__(self):
        return f'TrailingOffset(delay={self.trail_delay}s)'


# ------------------------------------------------------------------
# Registry
# ------------------------------------------------------------------
STRATEGIES = {
    'naive': NaiveFollow,
    'lateral_offset': LateralOffset,
    'trailing': TrailingOffset,
}


def get_strategy(name: str, **kwargs) -> FollowerStrategy:
    """Instantiate a strategy by name.

    Args:
        name: One of 'naive', 'lateral_offset', 'trailing'
        **kwargs: Passed to the strategy constructor

    Returns:
        FollowerStrategy instance
    """
    if name not in STRATEGIES:
        available = ', '.join(STRATEGIES.keys())
        raise ValueError(f'Unknown strategy "{name}". Available: {available}')
    return STRATEGIES[name](**kwargs)
