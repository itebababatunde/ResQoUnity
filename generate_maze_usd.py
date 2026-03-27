"""
generate_maze_usd.py — Offline maze USD generator.

Run with Isaac Sim's Python (which has pxr):
    ~/.local/share/ov/pkg/isaac-sim-2023.1.1/python.sh generate_maze_usd.py [seed]

Produces: envs/maze.usda
Loaded in sim the same way as warehouse.usd / office.usd.
"""

import sys
import os
import random

from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf

# ── Maze parameters ────────────────────────────────────────────────────────────
ROWS          = 6
COLS          = 6
CELL_SIZE     = 2.0    # m between cell centres
WALL_T        = 0.2    # wall thickness (m)
WALL_H        = 0.8    # wall height (m)
ENTRANCE_GAP  = CELL_SIZE          # opening width = exactly 1 cell → single valid route

BORDER_HALF_X = COLS * CELL_SIZE / 2.0   # 6.0
BORDER_HALF_Y = ROWS * CELL_SIZE / 2.0   # 6.0

# ── Maze generation (DFS) ──────────────────────────────────────────────────────
DIRECTIONS = [(-1,0,'N'),(0,1,'E'),(1,0,'S'),(0,-1,'W')]
OPPOSITE   = {'N':'S','S':'N','E':'W','W':'E'}

def generate_grid(seed):
    rng  = random.Random(seed)
    grid = [[{'N':False,'E':False,'S':False,'W':False}
              for _ in range(COLS)] for _ in range(ROWS)]
    vis  = [[False]*COLS for _ in range(ROWS)]

    def carve(r, c):
        vis[r][c] = True
        dirs = DIRECTIONS[:]
        rng.shuffle(dirs)
        for dr, dc, wall in dirs:
            nr, nc = r+dr, c+dc
            if 0 <= nr < ROWS and 0 <= nc < COLS and not vis[nr][nc]:
                grid[r][c][wall] = True
                grid[nr][nc][OPPOSITE[wall]] = True
                carve(nr, nc)
    carve(0, 0)
    return grid

def cell_centre(row, col):
    """World (x, y) of cell centre."""
    x = col * CELL_SIZE - (COLS-1)*CELL_SIZE/2.0
    y = row * CELL_SIZE - (ROWS-1)*CELL_SIZE/2.0
    return x, y

# ── USD helpers ────────────────────────────────────────────────────────────────
def add_box(stage, path, cx, cy, cz, sx, sy, sz):
    """Add a static kinematic box (wall) at world position (cx,cy,cz) with size (sx,sy,sz)."""
    cube = UsdGeom.Cube.Define(stage, path)
    xf   = UsdGeom.Xformable(cube.GetPrim())
    xf.AddTranslateOp().Set(Gf.Vec3d(cx, cy, cz))
    xf.AddScaleOp().Set(Gf.Vec3d(sx, sy, sz))
    cube.GetPrim().GetAttribute('size').Set(1.0)   # unit cube, scaled by xform
    # Physics: static collider
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    rb = UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
    rb.CreateKinematicEnabledAttr(True)
    return cube

# ── Main USD build ─────────────────────────────────────────────────────────────
def build_maze_usd(seed, out_path):
    grid = generate_grid(seed)

    stage = Usd.Stage.CreateNew(out_path)
    stage.SetMetadata('metersPerUnit', 1.0)
    stage.SetMetadata('upAxis', 'Z')

    root = stage.DefinePrim('/Maze', 'Xform')
    stage.SetDefaultPrim(root)          # required for UsdFileCfg reference loading
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

    idx  = [0]
    tz   = WALL_H / 2.0

    def wall(cx, cy, sx, sy, label="w"):
        path = f'/Maze/{label}_{idx[0]:04d}'
        add_box(stage, path, cx, cy, tz, sx, sy, WALL_H)
        idx[0] += 1

    gap_half = ENTRANCE_GAP / 2.0

    # ── South border: entrance gap centred on cell (0,0) ──────────────────────
    entrance_x, _ = cell_centre(0, 0)
    s_gap_l = max(entrance_x - gap_half, -BORDER_HALF_X)
    s_gap_r = min(entrance_x + gap_half,  BORDER_HALF_X)

    seg = s_gap_l - (-BORDER_HALF_X)
    if seg > WALL_T:
        wall(-BORDER_HALF_X + seg/2, -BORDER_HALF_Y, seg, WALL_T, "bS_L")

    seg = BORDER_HALF_X - s_gap_r
    if seg > WALL_T:
        wall(s_gap_r + seg/2, -BORDER_HALF_Y, seg, WALL_T, "bS_R")

    # ── North border: exit gap centred on cell (rows-1, cols-1) ───────────────
    exit_x, _ = cell_centre(ROWS-1, COLS-1)
    n_gap_l = max(exit_x - gap_half, -BORDER_HALF_X)
    n_gap_r = min(exit_x + gap_half,  BORDER_HALF_X)

    seg = n_gap_l - (-BORDER_HALF_X)
    if seg > WALL_T:
        wall(-BORDER_HALF_X + seg/2, BORDER_HALF_Y, seg, WALL_T, "bN_L")

    seg = BORDER_HALF_X - n_gap_r
    if seg > WALL_T:
        wall(n_gap_r + seg/2, BORDER_HALF_Y, seg, WALL_T, "bN_R")

    # ── East / West borders (exact height, no corner overhang) ────────────────
    wall( BORDER_HALF_X, 0.0, WALL_T, ROWS*CELL_SIZE, "bE")
    wall(-BORDER_HALF_X, 0.0, WALL_T, ROWS*CELL_SIZE, "bW")

    # ── Internal walls ─────────────────────────────────────────────────────────
    half_w = (COLS-1)*CELL_SIZE/2.0
    half_h = (ROWS-1)*CELL_SIZE/2.0

    for r in range(ROWS):
        for c in range(COLS):
            cx = c*CELL_SIZE - half_w
            cy = r*CELL_SIZE - half_h

            # South wall of cell — at cy + CELL_SIZE/2 (shared boundary with row r+1)
            if not grid[r][c]['S'] and r+1 < ROWS:
                wall(cx, cy + CELL_SIZE/2,
                     CELL_SIZE + WALL_T, WALL_T, "iS")

            # East wall of cell
            if not grid[r][c]['E'] and c+1 < COLS:
                wall(cx + CELL_SIZE/2, cy,
                     WALL_T, CELL_SIZE + WALL_T, "iE")

    # ── Start / End visual markers ─────────────────────────────────────────────
    def add_cylinder(path, cx, cy, radius, height, color):
        cyl = UsdGeom.Cylinder.Define(stage, path)
        cyl.GetHeightAttr().Set(height)
        cyl.GetRadiusAttr().Set(radius)
        xf  = UsdGeom.Xformable(cyl.GetPrim())
        xf.AddTranslateOp().Set(Gf.Vec3d(cx, cy, height/2))
        cyl.GetDisplayColorAttr().Set([Gf.Vec3f(*color)])

    sx, sy = cell_centre(0, 0)
    ex, ey = cell_centre(ROWS-1, COLS-1)
    add_cylinder('/Maze/marker_start', sx, sy, 0.25, 0.05, (0,1,0))
    add_cylinder('/Maze/marker_end',   ex, ey, 0.25, 0.05, (1,0,0))

    stage.GetRootLayer().Save()
    print(f"[MazeUSD] Saved: {out_path}")
    print(f"[MazeUSD] Walls: {idx[0]}  seed={seed}")
    print(f"[MazeUSD] Entrance gap: x=[{s_gap_l:.2f}, {s_gap_r:.2f}]  width={s_gap_r-s_gap_l:.2f}m")
    print(f"[MazeUSD] Exit gap:     x=[{n_gap_l:.2f}, {n_gap_r:.2f}]  width={n_gap_r-n_gap_l:.2f}m")
    print(f"[MazeUSD] Start cell world pos: ({sx:.1f}, {sy:.1f})")
    print(f"[MazeUSD] End cell world pos:   ({ex:.1f}, {ey:.1f})")


if __name__ == '__main__':
    seed     = int(sys.argv[1]) if len(sys.argv) > 1 else 42
    script_dir = os.path.dirname(os.path.abspath(__file__))
    out_path = os.path.join(script_dir, 'envs', 'maze.usda')
    build_maze_usd(seed, out_path)
