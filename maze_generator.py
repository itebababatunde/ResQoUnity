"""
maze_generator.py — Procedural maze generation for the aerial-assisted maze navigation use case.

Generates a 6x6 cell maze using recursive-backtracking (DFS), spawns walls as USD Cube prims
in Isaac Sim, and provides coordinate utilities for the planner and dog controller.

Maze layout:
  - 6x6 cells, 1m cell size (center-to-center spacing)
  - Wall thickness: 0.1m, wall height: 0.4m
  - Centered at origin: cells span [-2.5, 2.5] x [-2.5, 2.5]
  - Start cell: (row=0, col=0) -> world (-2.5, -2.5)
  - End cell:   (row=5, col=5) -> world (+2.5, +2.5)

Expanded (13x13) occupancy grid:
  - Each maze cell (r,c) maps to expanded index (2r+1, 2c+1)
  - Wall passages map to even-indexed coordinates
  - 0 = free, 1 = wall
"""

import os
import random
import numpy as np

# Maze dimensions — configurable via env vars, defaults to 6×6
ROWS = int(os.environ.get('MAZE_ROWS', 6))
COLS = int(os.environ.get('MAZE_COLS', 6))
CELL_SIZE = 2.0       # meters between cell centers (keep fixed — dog needs 2m corridors)
WALL_THICKNESS = 0.2  # meters
WALL_HEIGHT = 0.8     # meters
ENTRANCE_GAP = CELL_SIZE        # entrance/exit gap width = exactly 1 cell → single valid route

# Directions: (row_delta, col_delta, wall_flag_name)
DIRECTIONS = [
    (-1, 0, 'N'),
    (0,  1, 'E'),
    (1,  0, 'S'),
    (0, -1, 'W'),
]
OPPOSITE = {'N': 'S', 'S': 'N', 'E': 'W', 'W': 'E'}


def generate_maze_grid(rows=ROWS, cols=COLS, seed=None):
    """
    Generate a maze using recursive-backtracking DFS.

    Returns a 2D list of dicts, each dict has keys 'N', 'E', 'S', 'W'
    with value True if that wall is OPEN (passage exists), False if CLOSED (wall).

    grid[row][col]['N'] = True means the northern wall of cell (row, col) is open.
    """
    import sys
    sys.setrecursionlimit(max(1000, rows * cols * 4))

    rng = random.Random(seed)

    # Initialize all walls closed
    grid = [[{'N': False, 'E': False, 'S': False, 'W': False}
              for _ in range(cols)] for _ in range(rows)]
    visited = [[False] * cols for _ in range(rows)]

    def carve(r, c):
        visited[r][c] = True
        dirs = DIRECTIONS[:]
        rng.shuffle(dirs)
        for dr, dc, wall in dirs:
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols and not visited[nr][nc]:
                grid[r][c][wall] = True          # open wall in this cell
                grid[nr][nc][OPPOSITE[wall]] = True  # open matching wall in neighbor
                carve(nr, nc)

    carve(0, 0)
    return grid


def get_occupancy_grid(grid=None, rows=ROWS, cols=COLS):
    """
    Convert the maze wall grid into a (2*rows+1) x (2*cols+1) binary occupancy array.

    Expanded grid conventions:
      - odd (i, j): cell interior — always 0 (free)
      - even (i, j): boundary/corner — 1 if wall, 0 if open passage
      - Index mapping: cell (r, c) -> expanded (2r+1, 2c+1)

    Returns np.ndarray of shape (13, 13) for a 6x6 maze (dtype uint8, 0=free, 1=wall).
    """
    if grid is None:
        grid = generate_maze_grid(rows, cols)

    exp_rows = 2 * rows + 1
    exp_cols = 2 * cols + 1
    occ = np.ones((exp_rows, exp_cols), dtype=np.uint8)  # start all walls

    for r in range(rows):
        for c in range(cols):
            er, ec = 2 * r + 1, 2 * c + 1
            occ[er][ec] = 0  # cell interior is free

            if grid[r][c]['S'] and r + 1 < rows:   # south passage
                occ[er + 1][ec] = 0
            if grid[r][c]['E'] and c + 1 < cols:   # east passage
                occ[er][ec + 1] = 0

    return occ


def get_cell_center_world(row, col, rows=ROWS, cols=COLS):
    """
    Return (x, y) world coordinates for the center of maze cell (row, col).

    World origin at maze center. Row 0 = most negative Y, Col 0 = most negative X.
    """
    half_w = (cols - 1) * CELL_SIZE / 2.0
    half_h = (rows - 1) * CELL_SIZE / 2.0
    x = col * CELL_SIZE - half_w
    y = row * CELL_SIZE - half_h
    return (x, y)


def print_ascii_maze(grid, rows=ROWS, cols=COLS):
    """Print a human-readable ASCII representation of the maze for debugging.

    Entrance opening is shown above the start cell (row=0, col=0).
    Exit opening is shown below the end cell (row=rows-1, col=cols-1).
    Note: 'top' in ASCII = world-south (row=0 maps to most-negative y).
    """
    lines = []
    # Top border — open above start cell col=0 (entrance)
    top = '+'
    for c in range(cols):
        top += ('   ' if c == 0 else '---') + '+'
    lines.append(top)
    for r in range(rows):
        # Cell row: west walls and cell contents
        row_str = '|'
        for c in range(cols):
            cell_label = '   '
            if r == 0 and c == 0:
                cell_label = ' S '  # Start
            elif r == rows - 1 and c == cols - 1:
                cell_label = ' E '  # End
            east_wall = ' ' if grid[r][c]['E'] else '|'
            row_str += cell_label + east_wall
        lines.append(row_str)
        # South walls — open below end cell col=cols-1 on last row (exit)
        south_str = '+'
        for c in range(cols):
            if r == rows - 1 and c == cols - 1:
                south_wall = '   '  # exit opening
            else:
                south_wall = '   ' if grid[r][c]['S'] else '---'
            south_str += south_wall + '+'
        lines.append(south_str)
    print('\n'.join(lines))


def spawn_walls_in_stage(grid, stage, rows=ROWS, cols=COLS):
    """
    Spawn maze walls as USD Cube prims with physics colliders in Isaac Sim.

    Each wall segment is a thin box at the correct world position.
    Requires: omni.usd, pxr.UsdGeom, pxr.UsdPhysics, pxr.Gf, pxr.Sdf

    Args:
        grid: 2D list from generate_maze_grid()
        stage: omni.usd.get_context().get_stage()
    """
    try:
        from pxr import UsdGeom, UsdPhysics, Gf, Sdf
        import omni.usd
    except ImportError:
        print("[MazeGenerator] USD libraries not available — skipping wall spawn (standalone mode).")
        return

    maze_root = "/World/Maze"
    root_prim = stage.DefinePrim(maze_root, "Xform")

    wall_idx = 0
    half_w = (cols - 1) * CELL_SIZE / 2.0
    half_h = (rows - 1) * CELL_SIZE / 2.0

    def _spawn_box(name, tx, ty, tz, sx, sy, sz):
        prim_path = f"{maze_root}/{name}"
        cube = UsdGeom.Cube.Define(stage, prim_path)
        xform = UsdGeom.Xformable(cube.GetPrim())
        xform_ops = xform.AddXformOp(UsdGeom.XformOp.TypeTranslate)
        xform_ops.Set(Gf.Vec3d(tx, ty, tz))
        scale_op = xform.AddXformOp(UsdGeom.XformOp.TypeScale)
        scale_op.Set(Gf.Vec3d(sx, sy, sz))
        # Add collision
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        rigid = UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
        rigid.CreateKinematicEnabledAttr(True)

    def _wall_pos(row, col, direction):
        nonlocal wall_idx
        cx = col * CELL_SIZE - half_w
        cy = row * CELL_SIZE - half_h
        tz = WALL_HEIGHT / 2.0

        if direction == 'N':
            tx, ty = cx, cy - CELL_SIZE / 2.0
            sx, sy = CELL_SIZE + WALL_THICKNESS, WALL_THICKNESS
        elif direction == 'S':
            tx, ty = cx, cy + CELL_SIZE / 2.0
            sx, sy = CELL_SIZE + WALL_THICKNESS, WALL_THICKNESS
        elif direction == 'W':
            tx, ty = cx - CELL_SIZE / 2.0, cy
            sx, sy = WALL_THICKNESS, CELL_SIZE + WALL_THICKNESS
        elif direction == 'E':
            tx, ty = cx + CELL_SIZE / 2.0, cy
            sx, sy = WALL_THICKNESS, CELL_SIZE + WALL_THICKNESS

        name = f"wall_{wall_idx:04d}"
        _spawn_box(name, tx, ty, tz, sx, sy, WALL_HEIGHT)
        wall_idx += 1

    # Spawn outer border
    border_half_x = cols * CELL_SIZE / 2.0
    border_half_y = rows * CELL_SIZE / 2.0
    tz = WALL_HEIGHT / 2.0

    gap_half = ENTRANCE_GAP / 2.0  # half the entrance/exit opening width

    # ---- South border: entrance gap centred on start cell (row=0, col=0) ----
    entrance_x = get_cell_center_world(0, 0, rows, cols)[0]
    s_gap_l = max(entrance_x - gap_half, -border_half_x)
    s_gap_r = min(entrance_x + gap_half,  border_half_x)
    # Left segment
    seg_len = s_gap_l - (-border_half_x)
    if seg_len > WALL_THICKNESS:
        _spawn_box("border_S_left", -border_half_x + seg_len / 2.0, -border_half_y, tz,
                   seg_len, WALL_THICKNESS, WALL_HEIGHT)
    # Right segment
    seg_len = border_half_x - s_gap_r
    if seg_len > WALL_THICKNESS:
        _spawn_box("border_S_right", s_gap_r + seg_len / 2.0, -border_half_y, tz,
                   seg_len, WALL_THICKNESS, WALL_HEIGHT)

    # ---- North border: exit gap centred on end cell (row=rows-1, col=cols-1) ----
    exit_x = get_cell_center_world(rows - 1, cols - 1, rows, cols)[0]
    n_gap_l = max(exit_x - gap_half, -border_half_x)
    n_gap_r = min(exit_x + gap_half,  border_half_x)
    # Left segment
    seg_len = n_gap_l - (-border_half_x)
    if seg_len > WALL_THICKNESS:
        _spawn_box("border_N_left", -border_half_x + seg_len / 2.0, border_half_y, tz,
                   seg_len, WALL_THICKNESS, WALL_HEIGHT)
    # Right segment
    seg_len = border_half_x - n_gap_r
    if seg_len > WALL_THICKNESS:
        _spawn_box("border_N_right", n_gap_r + seg_len / 2.0, border_half_y, tz,
                   seg_len, WALL_THICKNESS, WALL_HEIGHT)

    # East / West borders — exact height (no corner overhang that would seal the gaps)
    _spawn_box("border_E", border_half_x, 0.0, tz,
               WALL_THICKNESS, rows * CELL_SIZE, WALL_HEIGHT)
    _spawn_box("border_W", -border_half_x, 0.0, tz,
               WALL_THICKNESS, rows * CELL_SIZE, WALL_HEIGHT)

    # Internal walls: only spawn S and E walls to avoid duplicates
    for r in range(rows):
        for c in range(cols):
            if not grid[r][c]['S'] and r + 1 < rows:
                _wall_pos(r, c, 'S')
            if not grid[r][c]['E'] and c + 1 < cols:
                _wall_pos(r, c, 'E')

    print(f"[MazeGenerator] Spawned {wall_idx} wall segments.")


def spawn_markers(stage, rows=ROWS, cols=COLS):
    """
    Spawn visual markers: green cylinder at start (0,0), red cylinder at end (5,5).
    """
    try:
        from pxr import UsdGeom, Gf
    except ImportError:
        print("[MazeGenerator] USD libraries not available — skipping markers (standalone mode).")
        return

    start_x, start_y = get_cell_center_world(0, 0, rows, cols)
    end_x, end_y = get_cell_center_world(rows - 1, cols - 1, rows, cols)

    for name, tx, ty, color in [
        ("marker_start", start_x, start_y, (0.0, 1.0, 0.0)),
        ("marker_end",   end_x,   end_y,   (1.0, 0.0, 0.0)),
    ]:
        prim_path = f"/World/Maze/{name}"
        cyl = UsdGeom.Cylinder.Define(stage, prim_path)
        cyl.GetHeightAttr().Set(0.05)
        cyl.GetRadiusAttr().Set(0.15)
        xform = UsdGeom.Xformable(cyl.GetPrim())
        trans = xform.AddXformOp(UsdGeom.XformOp.TypeTranslate)
        trans.Set(Gf.Vec3d(tx, ty, 0.03))
        # Set display color
        cyl.GetDisplayColorAttr().Set([Gf.Vec3f(*color)])

    print("[MazeGenerator] Spawned start (green) and end (red) markers.")


# ---------------------------------------------------------------------------
# Standalone test
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    import sys

    seed = int(sys.argv[1]) if len(sys.argv) > 1 else 42
    print(f"Generating maze with seed={seed} ...")
    grid = generate_maze_grid(ROWS, COLS, seed=seed)
    print_ascii_maze(grid)

    occ = get_occupancy_grid(grid)
    print(f"\nOccupancy grid shape: {occ.shape}")
    print("Occupancy grid (0=free, 1=wall):")
    for row in occ:
        print(' '.join(str(v) for v in row))

    # Verify start and end cells are free
    assert occ[1][1] == 0, "Start cell should be free"
    assert occ[2 * ROWS - 1][2 * COLS - 1] == 0, "End cell should be free"

    # Print world coordinates of each cell corner
    print(f"\nCell (0,0) center: {get_cell_center_world(0, 0)}")
    print(f"Cell (5,5) center: {get_cell_center_world(5, 5)}")
    print("\nAll tests passed.")
