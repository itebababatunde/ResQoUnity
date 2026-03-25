"""
debug_maze_2d.py — 2D visualisation of the exact walls that spawn_walls_in_stage creates.

Mirrors spawn_walls_in_stage() logic exactly (no Isaac Sim needed).
Run:  python3 debug_maze_2d.py [seed]
"""

import sys
import math
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

from maze_generator import (
    generate_maze_grid, get_cell_center_world,
    ROWS, COLS, CELL_SIZE, WALL_THICKNESS, WALL_HEIGHT, ENTRANCE_GAP,
)

# ── reproduce spawn_walls_in_stage geometry ──────────────────────────────────

def collect_walls(grid, rows=ROWS, cols=COLS):
    """
    Return a list of dicts describing every wall box that would be spawned,
    each with keys: cx, cy, sx, sy, label
    (centre position and half-extents in X/Y; label for debugging)
    """
    walls = []
    half_w = (cols - 1) * CELL_SIZE / 2.0
    half_h = (rows - 1) * CELL_SIZE / 2.0
    border_half_x = cols * CELL_SIZE / 2.0
    border_half_y = rows * CELL_SIZE / 2.0
    gap_half = ENTRANCE_GAP / 2.0       # ← mirrors spawn_walls_in_stage

    def add(cx, cy, sx, sy, label="wall"):
        walls.append(dict(cx=cx, cy=cy, sx=sx, sy=sy, label=label))

    # ── South border: entrance gap centred on start cell (row=0, col=0) ───────
    entrance_x = get_cell_center_world(0, 0, rows, cols)[0]
    s_gap_l = max(entrance_x - gap_half, -border_half_x)
    s_gap_r = min(entrance_x + gap_half,  border_half_x)

    seg = s_gap_l - (-border_half_x)
    if seg > WALL_THICKNESS:
        add(-border_half_x + seg/2, -border_half_y, seg, WALL_THICKNESS, "border_S_left")

    seg = border_half_x - s_gap_r
    if seg > WALL_THICKNESS:
        add(s_gap_r + seg/2, -border_half_y, seg, WALL_THICKNESS, "border_S_right")

    # ── North border: exit gap centred on end cell (row=rows-1, col=cols-1) ──
    exit_x = get_cell_center_world(rows-1, cols-1, rows, cols)[0]
    n_gap_l = max(exit_x - gap_half, -border_half_x)
    n_gap_r = min(exit_x + gap_half,  border_half_x)

    seg = n_gap_l - (-border_half_x)
    if seg > WALL_THICKNESS:
        add(-border_half_x + seg/2, border_half_y, seg, WALL_THICKNESS, "border_N_left")

    seg = border_half_x - n_gap_r
    if seg > WALL_THICKNESS:
        add(n_gap_r + seg/2, border_half_y, seg, WALL_THICKNESS, "border_N_right")

    # ── East / West borders — no corner overhang ──────────────────────────────
    add( border_half_x, 0.0, WALL_THICKNESS, rows*CELL_SIZE, "border_E")
    add(-border_half_x, 0.0, WALL_THICKNESS, rows*CELL_SIZE, "border_W")

    # ── Internal walls ────────────────────────────────────────────────────────
    for r in range(rows):
        for c in range(cols):
            cx_cell = c * CELL_SIZE - half_w
            cy_cell = r * CELL_SIZE - half_h

            if not grid[r][c]['S'] and r + 1 < rows:      # south wall of cell
                add(cx_cell, cy_cell - CELL_SIZE/2,
                    CELL_SIZE + WALL_THICKNESS, WALL_THICKNESS, f"S_{r}_{c}")

            if not grid[r][c]['E'] and c + 1 < cols:      # east wall of cell
                add(cx_cell + CELL_SIZE/2, cy_cell,
                    WALL_THICKNESS, CELL_SIZE + WALL_THICKNESS, f"E_{r}_{c}")

    return walls, border_half_x, border_half_y, entrance_x, exit_x, s_gap_l, s_gap_r, n_gap_l, n_gap_r


# ── draw ─────────────────────────────────────────────────────────────────────

def draw(seed=42):
    grid = generate_maze_grid(ROWS, COLS, seed=seed)
    walls, bhx, bhy, ent_x, ext_x, s_gl, s_gr, n_gl, n_gr = collect_walls(grid)

    fig, ax = plt.subplots(figsize=(10, 10))
    ax.set_aspect('equal')
    ax.set_facecolor('#e8e8e8')
    ax.set_title(f'Maze 2-D layout (seed={seed})\n'
                 f'CELL={CELL_SIZE}m  WALL_T={WALL_THICKNESS}m  '
                 f'entrance gap x=[{s_gl:.1f},{s_gr:.1f}]  '
                 f'exit gap x=[{n_gl:.1f},{n_gr:.1f}]', fontsize=9)

    # wall rectangles
    for w in walls:
        rect = patches.Rectangle(
            (w['cx'] - w['sx']/2, w['cy'] - w['sy']/2),
            w['sx'], w['sy'],
            linewidth=0.5, edgecolor='black',
            facecolor='steelblue' if 'border' in w['label'] else 'dimgray',
            alpha=0.85, zorder=2
        )
        ax.add_patch(rect)

    # cell centres (light grid)
    for r in range(ROWS):
        for c in range(COLS):
            cx, cy = get_cell_center_world(r, c, ROWS, COLS)
            ax.plot(cx, cy, 'k+', markersize=4, alpha=0.3, zorder=1)

    # entrance gap arrow
    ax.annotate('', xy=(ent_x, -bhy + 0.1),
                xytext=(ent_x, -bhy - CELL_SIZE),
                arrowprops=dict(arrowstyle='->', color='green', lw=2))
    ax.text(ent_x, -bhy - CELL_SIZE - 0.3, 'ENTRY', color='green',
            ha='center', va='top', fontsize=9, fontweight='bold')

    # exit gap arrow
    ax.annotate('', xy=(ext_x, bhy - 0.1),
                xytext=(ext_x, bhy + CELL_SIZE),
                arrowprops=dict(arrowstyle='->', color='red', lw=2))
    ax.text(ext_x, bhy + CELL_SIZE + 0.3, 'EXIT', color='red',
            ha='center', va='bottom', fontsize=9, fontweight='bold')

    # start / end markers
    sx, sy = get_cell_center_world(0, 0, ROWS, COLS)
    ex, ey = get_cell_center_world(ROWS-1, COLS-1, ROWS, COLS)
    ax.plot(sx, sy, 'go', markersize=12, label='Start cell (0,0)', zorder=5)
    ax.plot(ex, ey, 'r^', markersize=12, label='End cell (5,5)',   zorder=5)

    # dog spawn position
    dog_x = sx
    dog_y = -(ROWS * CELL_SIZE / 2.0) - CELL_SIZE / 2.0
    ax.plot(dog_x, dog_y, 'bs', markersize=12, label=f'Dog spawn ({dog_x:.1f},{dog_y:.1f})', zorder=5)

    # drone spawn
    drone_x = dog_x + CELL_SIZE
    drone_y = dog_y
    ax.plot(drone_x, drone_y, 'm^', markersize=12, label=f'Drone spawn ({drone_x:.1f},{drone_y:.1f})', zorder=5)

    # gap highlight bars
    ax.plot([s_gl, s_gr], [-bhy, -bhy], 'g-', lw=4, alpha=0.6, label=f'S-gap [{s_gl:.1f},{s_gr:.1f}]')
    ax.plot([n_gl, n_gr], [ bhy,  bhy], 'r-', lw=4, alpha=0.6, label=f'N-gap [{n_gl:.1f},{n_gr:.1f}]')

    # axis limits with margin
    margin = CELL_SIZE * 1.5
    ax.set_xlim(-bhx - margin, bhx + margin)
    ax.set_ylim(-bhy - margin, bhy + margin)
    ax.set_xlabel('World X (m)')
    ax.set_ylabel('World Y (m)')
    ax.axhline(0, color='gray', lw=0.5, alpha=0.4)
    ax.axvline(0, color='gray', lw=0.5, alpha=0.4)
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(True, alpha=0.2)

    # print gap analysis
    print(f"\n=== Gap Analysis ===")
    print(f"Border half-x: {bhx}m  half-y: {bhy}m")
    print(f"South border gap: x=[{s_gl:.2f}, {s_gr:.2f}]  width={s_gr-s_gl:.2f}m")
    print(f"  gap starts at west border? {abs(s_gl - (-bhx)) < 0.01}")
    print(f"  gap ends at east border?   {abs(s_gr -  bhx) < 0.01}")
    print(f"North border gap: x=[{n_gl:.2f}, {n_gr:.2f}]  width={n_gr-n_gl:.2f}m")
    print(f"  gap starts at west border? {abs(n_gl - (-bhx)) < 0.01}")
    print(f"  gap ends at east border?   {abs(n_gr -  bhx) < 0.01}")
    print(f"Dog spawn: ({dog_x:.2f}, {dog_y:.2f})")
    print(f"  Dog x within south gap? {s_gl <= dog_x <= s_gr}")
    print(f"  Dog outside south border (y < {-bhy:.1f})? {dog_y < -bhy}")
    print(f"Total walls spawned: {len(walls)}")

    plt.tight_layout()
    out = f"/tmp/maze_debug_seed{seed}.png"
    plt.savefig(out, dpi=150)
    print(f"\nSaved: {out}")
    plt.show()


if __name__ == "__main__":
    seed = int(sys.argv[1]) if len(sys.argv) > 1 else 42
    draw(seed)
