"""
maze_astar.py — A* path planner for the aerial-assisted maze navigation use case.

Operates on the 13x13 expanded occupancy grid produced by maze_generator.get_occupancy_grid().
Plans a path from the start cell to the end cell and converts grid indices to world waypoints.

Expanded grid index conventions (mirrored from maze_generator.py):
  - Cell (r, c) in maze space -> expanded index (2r+1, 2c+1)
  - Even indices: walls / passages between cells
  - 0 = free, 1 = wall

Usage (standalone test):
    python maze_astar.py [seed]
"""

import heapq
import numpy as np

from maze_generator import (
    generate_maze_grid,
    get_occupancy_grid,
    get_cell_center_world,
    ROWS, COLS,
)


def _heuristic(a, b):
    """Manhattan distance heuristic."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def astar_on_grid(occ, start_exp, goal_exp):
    """
    Run A* on the expanded occupancy grid.

    Args:
        occ: np.ndarray (H, W) with 0=free, 1=wall
        start_exp: (row, col) in expanded grid coordinates
        goal_exp:  (row, col) in expanded grid coordinates

    Returns:
        List of (row, col) expanded grid indices from start to goal (inclusive),
        or None if no path found.
    """
    H, W = occ.shape
    open_heap = []
    g = {start_exp: 0}
    parent = {start_exp: None}
    heapq.heappush(open_heap, (0 + _heuristic(start_exp, goal_exp), start_exp))
    closed = set()

    while open_heap:
        _, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        if current == goal_exp:
            # Reconstruct path
            path = []
            node = current
            while node is not None:
                path.append(node)
                node = parent[node]
            path.reverse()
            return path
        closed.add(current)

        cr, cc = current
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nr, nc = cr + dr, cc + dc
            if 0 <= nr < H and 0 <= nc < W and occ[nr][nc] == 0:
                neighbor = (nr, nc)
                if neighbor in closed:
                    continue
                ng = g[current] + 1
                if ng < g.get(neighbor, float('inf')):
                    g[neighbor] = ng
                    parent[neighbor] = current
                    f = ng + _heuristic(neighbor, goal_exp)
                    heapq.heappush(open_heap, (f, neighbor))

    return None  # No path found


def expanded_to_world(exp_r, exp_c, rows=ROWS, cols=COLS):
    """
    Convert expanded grid index (exp_r, exp_c) to world (x, y) coordinates.

    Cell centers are at odd indices; passage midpoints at even indices.
    """
    # Map expanded index back to continuous maze coordinate
    # exp = 2*maze_idx + 1 for cell centers; 0.5-step for passages
    maze_r = (exp_r - 1) / 2.0
    maze_c = (exp_c - 1) / 2.0
    half_w = (cols - 1) / 2.0
    half_h = (rows - 1) / 2.0
    x = maze_c - half_w
    y = maze_r - half_h
    return (x, y)


class MazeAstar:
    """
    A* planner wrapper. Holds reference to the occupancy grid and provides
    plan() and smooth_path() methods that return 3D world waypoints.
    """

    DOG_Z = 0.3  # height of waypoints for dog navigation (slightly above ground)

    def __init__(self, occ=None, rows=ROWS, cols=COLS):
        """
        Args:
            occ: 13x13 np.ndarray (0=free, 1=wall). If None, generates a fresh maze.
            rows, cols: maze dimensions (default 6x6).
        """
        self.rows = rows
        self.cols = cols
        if occ is None:
            grid = generate_maze_grid(rows, cols)
            occ = get_occupancy_grid(grid, rows, cols)
        self.occ = occ

    def plan(self, start_cell=(0, 0), end_cell=None):
        """
        Plan a path from start_cell to end_cell.

        Args:
            start_cell: (row, col) in maze cell space
            end_cell:   (row, col) in maze cell space; defaults to (rows-1, cols-1)

        Returns:
            List of np.ndarray([x, y, z]) world waypoints, or None if no path.
        """
        if end_cell is None:
            end_cell = (self.rows - 1, self.cols - 1)

        start_exp = (2 * start_cell[0] + 1, 2 * start_cell[1] + 1)
        goal_exp  = (2 * end_cell[0]  + 1, 2 * end_cell[1]  + 1)

        exp_path = astar_on_grid(self.occ, start_exp, goal_exp)
        if exp_path is None:
            return None

        waypoints = []
        for (er, ec) in exp_path:
            x, y = expanded_to_world(er, ec, self.rows, self.cols)
            waypoints.append(np.array([x, y, self.DOG_Z]))

        return waypoints

    def smooth_path(self, waypoints):
        """
        Line-of-sight pruning: remove intermediate waypoints that lie on a
        straight navigable line between non-adjacent waypoints.

        Args:
            waypoints: list of np.ndarray([x, y, z])

        Returns:
            Pruned list of waypoints (always includes start and end).
        """
        if len(waypoints) <= 2:
            return waypoints

        pruned = [waypoints[0]]
        i = 0
        while i < len(waypoints) - 1:
            # Find the furthest waypoint reachable in a straight line from i
            j = len(waypoints) - 1
            while j > i + 1:
                if self._line_of_sight(waypoints[i], waypoints[j]):
                    break
                j -= 1
            pruned.append(waypoints[j])
            i = j

        return pruned

    def _line_of_sight(self, wp_a, wp_b, samples=20):
        """
        Bresenham-style check: sample points along the segment and verify
        none fall in a wall cell in the occupancy grid.
        """
        for t in np.linspace(0.0, 1.0, samples):
            x = wp_a[0] + t * (wp_b[0] - wp_a[0])
            y = wp_a[1] + t * (wp_b[1] - wp_a[1])
            # Convert world -> expanded grid
            half_w = (self.cols - 1) / 2.0
            half_h = (self.rows - 1) / 2.0
            maze_c = x + half_w
            maze_r = y + half_h
            exp_r = round(maze_r * 2 + 1)
            exp_c = round(maze_c * 2 + 1)
            H, W = self.occ.shape
            if not (0 <= exp_r < H and 0 <= exp_c < W):
                return False
            if self.occ[exp_r][exp_c] == 1:
                return False
        return True


# ---------------------------------------------------------------------------
# Standalone test
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    import sys

    seed = int(sys.argv[1]) if len(sys.argv) > 1 else 42
    print(f"Testing A* planner with seed={seed} ...")

    grid = generate_maze_grid(ROWS, COLS, seed=seed)
    occ = get_occupancy_grid(grid, ROWS, COLS)

    planner = MazeAstar(occ=occ)
    waypoints = planner.plan(start_cell=(0, 0), end_cell=(ROWS - 1, COLS - 1))

    if waypoints is None:
        print("ERROR: No path found!")
        sys.exit(1)

    print(f"Raw path: {len(waypoints)} waypoints")
    smoothed = planner.smooth_path(waypoints)
    print(f"Smoothed path: {len(smoothed)} waypoints")

    # Print path as ASCII overlay on the occupancy grid
    exp_path_set = set()
    for wp in waypoints:
        half_w = (COLS - 1) / 2.0
        half_h = (ROWS - 1) / 2.0
        maze_c = wp[0] + half_w
        maze_r = wp[1] + half_h
        exp_r = round(maze_r * 2 + 1)
        exp_c = round(maze_c * 2 + 1)
        exp_path_set.add((exp_r, exp_c))

    print("\nOccupancy grid with path (* = path, # = wall, . = free):")
    H, W = occ.shape
    for r in range(H):
        row_str = ""
        for c in range(W):
            if (r, c) in exp_path_set:
                row_str += "* "
            elif occ[r][c] == 1:
                row_str += "# "
            else:
                row_str += ". "
        print(row_str)

    print("\nSmoothed waypoints (x, y, z):")
    for i, wp in enumerate(smoothed):
        print(f"  [{i}] ({wp[0]:.2f}, {wp[1]:.2f}, {wp[2]:.2f})")

    print("\nAll tests passed.")
