"""
maze_report.py — Offline experiment report generator for the maze navigation use case.

Reads a mission CSV log and the snapshot directory from one run, then produces a
single multi-panel PNG summarising the experiment for team sharing.

Usage:
    python3 maze_report.py                          # auto-picks latest CSV
    python3 maze_report.py logs/maze_<ts>.csv
    python3 maze_report.py logs/maze_<ts>.csv --seed 42 --snapshots logs/snapshots

Output: logs/reports/maze_report_<ts>.png
"""

import argparse
import os
import sys
import glob
import math
from datetime import datetime

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.gridspec as gridspec
from matplotlib.lines import Line2D

# ---------------------------------------------------------------------------
# Project imports
# ---------------------------------------------------------------------------
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from maze_generator import (
    generate_maze_grid, get_cell_center_world, ROWS, COLS, CELL_SIZE,
)
from maze_astar import MazeAstar
from debug_maze_2d import collect_walls   # reuse wall geometry — don't duplicate


# ---------------------------------------------------------------------------
# State colour map
# ---------------------------------------------------------------------------
STATE_COLORS = {
    'INIT':            '#aaaaaa',
    'ARMING':          '#f0a500',
    'CLIMBING':        '#f0c040',
    'CENTERING':       '#80c0ff',
    'ALTITUDE_SURVEY': '#4080ff',
    'PERCEIVING':      '#a040ff',
    'PLANNING':        '#ff80c0',
    'SHOWING_PATH':    '#ff4080',
    'GUIDING_DOG':     '#40c040',
    'DOG_STUCK':       '#ff4040',
    'SUCCESS':         '#00aa00',
    'FAILURE':         '#cc0000',
}


# ---------------------------------------------------------------------------
# CSV loader
# ---------------------------------------------------------------------------
def load_csv(path):
    """Return dict of column-name → numpy array."""
    import csv
    rows = []
    with open(path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)
    if not rows:
        raise ValueError(f'Empty CSV: {path}')
    data = {k: [] for k in rows[0]}
    for row in rows:
        for k, v in row.items():
            data[k].append(v)
    float_cols = ['t', 'dog_x', 'dog_y', 'drone_z', 'wp_idx', 'wp_total',
                  'vel_x', 'ang_z', 'dist_to_wp']
    for col in float_cols:
        if col in data:
            data[col] = np.array(data[col], dtype=float)
    return data


# ---------------------------------------------------------------------------
# Panel helpers
# ---------------------------------------------------------------------------

def _draw_maze_walls(ax, seed):
    """Draw maze wall rectangles on a matplotlib axes."""
    grid = generate_maze_grid(ROWS, COLS, seed=seed)
    walls, *_ = collect_walls(grid)
    for w in walls:
        rect = mpatches.Rectangle(
            (w['cx'] - w['sx'] / 2, w['cy'] - w['sy'] / 2),
            w['sx'], w['sy'],
            linewidth=0.4, edgecolor='black',
            facecolor='steelblue' if 'border' in w['label'] else '#444455',
            alpha=0.7, zorder=2,
        )
        ax.add_patch(rect)


def panel_trajectory(ax, data, seed):
    """Panel 1: top-down trajectory over maze layout."""
    _draw_maze_walls(ax, seed)

    # Dog trajectory (GUIDING_DOG rows only)
    mask = np.array(data['state']) == 'GUIDING_DOG'
    if mask.any():
        ax.plot(data['dog_x'][mask], data['dog_y'][mask],
                color='royalblue', lw=1.5, zorder=4, label='Dog path')

    # Planned path (re-run planner from seed)
    try:
        gt_grid = MazeAstar(occ=None)   # generates fresh maze — use seed below
        from maze_generator import get_occupancy_grid
        gt_occ = get_occupancy_grid(generate_maze_grid(seed=seed))
        planner = MazeAstar(occ=gt_occ)
        wps = planner.plan((0, 0), (ROWS - 1, COLS - 1))
        sm = planner.smooth_path(wps)
        xs = [w[0] for w in sm]
        ys = [w[1] for w in sm]
        ax.plot(xs, ys, 'g--', lw=1.5, zorder=5, label='Planned path')
        for wp in sm:
            ax.plot(wp[0], wp[1], 'go', ms=5, zorder=6)
    except Exception:
        pass

    # Stuck event markers: rows where dist_to_wp barely changes for >10 ticks
    if mask.any():
        d = data['dist_to_wp'][mask]
        t = data['t'][mask]
        x = data['dog_x'][mask]
        y = data['dog_y'][mask]
        stuck_xs, stuck_ys = [], []
        for i in range(10, len(d)):
            if abs(d[i] - d[i - 10]) < 0.05 and d[i] > 0.3:
                stuck_xs.append(x[i])
                stuck_ys.append(y[i])
        if stuck_xs:
            ax.scatter(stuck_xs, stuck_ys, marker='x', color='red', s=40,
                       zorder=7, label=f'Stuck ({len(stuck_xs)} pts)')

    # Start / end markers
    sx, sy = get_cell_center_world(0, 0)
    ex, ey = get_cell_center_world(ROWS - 1, COLS - 1)
    ax.plot(sx, sy, 'go', ms=10, zorder=8, label='Start')
    ax.plot(ex, ey, 'r^', ms=10, zorder=8, label='Goal')

    half = COLS * CELL_SIZE / 2.0 + CELL_SIZE
    ax.set_xlim(-half, half)
    ax.set_ylim(-half, half)
    ax.set_aspect('equal')
    ax.set_title('Dog Trajectory on Maze')
    ax.set_xlabel('World X (m)')
    ax.set_ylabel('World Y (m)')
    ax.legend(fontsize=7, loc='upper left')
    ax.grid(True, alpha=0.2)


def panel_state_timeline(ax, data):
    """Panel 2: Gantt-style state timeline."""
    states = data['state']
    t = data['t']
    # Detect transitions
    segments = []
    i = 0
    while i < len(states):
        j = i
        while j < len(states) and states[j] == states[i]:
            j += 1
        segments.append((states[i], float(t[i]), float(t[j - 1])))
        i = j

    unique_states = list(dict.fromkeys(s for s, _, _ in segments))
    y_map = {s: k for k, s in enumerate(unique_states)}

    for s, t0, t1 in segments:
        color = STATE_COLORS.get(s, '#888888')
        ax.barh(y_map[s], t1 - t0, left=t0, height=0.6,
                color=color, edgecolor='white', linewidth=0.3)

    ax.set_yticks(list(y_map.values()))
    ax.set_yticklabels(list(y_map.keys()), fontsize=7)
    ax.set_xlabel('Time (s)')
    ax.set_title('State Timeline')
    ax.grid(axis='x', alpha=0.3)


def panel_distance_to_goal(ax, data):
    """Panel 3: distance to goal over time during navigation."""
    ex, ey = get_cell_center_world(ROWS - 1, COLS - 1)
    dist = np.sqrt((data['dog_x'] - ex) ** 2 + (data['dog_y'] - ey) ** 2)
    mask = np.array(data['state']) == 'GUIDING_DOG'
    if mask.any():
        ax.plot(data['t'][mask], dist[mask], color='darkorange', lw=1.5)
        ax.axhline(0.3, color='green', ls='--', lw=1, label='Arrival radius (0.3m)')
        ax.legend(fontsize=7)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Distance to goal (m)')
    ax.set_title('Distance to Goal')
    ax.grid(True, alpha=0.3)


def panel_velocity(ax, data):
    """Panel 4: velocity profile during navigation."""
    mask = np.array(data['state']) == 'GUIDING_DOG'
    if mask.any():
        ax.plot(data['t'][mask], data['vel_x'][mask],
                color='royalblue', lw=1, label='linear_x (m/s)')
        ax.plot(data['t'][mask], data['ang_z'][mask],
                color='tomato', lw=1, label='angular_z (rad/s)')
        ax.axhline(0, color='gray', lw=0.5)
        ax.legend(fontsize=7)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Velocity')
    ax.set_title('Velocity Profile')
    ax.grid(True, alpha=0.3)


def panel_snapshots(ax, snapshot_dir, run_ts):
    """Panel 5: grid of key snapshots."""
    ax.axis('off')
    if not os.path.isdir(snapshot_dir):
        ax.text(0.5, 0.5, 'No snapshots directory found',
                ha='center', va='center', transform=ax.transAxes)
        return

    pattern = os.path.join(snapshot_dir, f'{run_ts}_*.png')
    files = sorted(glob.glob(pattern))
    if not files:
        # Fallback: all PNGs in dir
        files = sorted(glob.glob(os.path.join(snapshot_dir, '*.png')))

    if not files:
        ax.text(0.5, 0.5, 'No snapshots found', ha='center', va='center',
                transform=ax.transAxes)
        return

    import matplotlib.image as mpimg
    n = len(files)
    cols = min(n, 4)
    rows_g = math.ceil(n / cols)
    for idx, fpath in enumerate(files[:cols * rows_g]):
        sub = ax.inset_axes([
            (idx % cols) / cols + 0.01,
            1.0 - ((idx // cols) + 1) / rows_g + 0.02,
            1.0 / cols - 0.02,
            1.0 / rows_g - 0.04,
        ])
        try:
            img = mpimg.imread(fpath)
            sub.imshow(img)
        except Exception:
            sub.text(0.5, 0.5, 'err', ha='center', va='center')
        label = os.path.basename(fpath).replace(run_ts + '_', '').replace('.png', '')
        sub.set_title(label, fontsize=5, pad=1)
        sub.axis('off')
    ax.set_title('Drone Snapshots', pad=14)


def panel_metrics(ax, data, seed, csv_path):
    """Panel 6: text metrics table."""
    ax.axis('off')

    t = data['t']
    states = np.array(data['state'])

    # Time per state
    time_per_state = {}
    i = 0
    while i < len(states):
        j = i
        while j < len(states) and states[j] == states[i]:
            j += 1
        s = states[i]
        dur = float(t[j - 1]) - float(t[i])
        time_per_state[s] = time_per_state.get(s, 0.0) + dur
        i = j

    total_time = float(t[-1]) if len(t) else 0.0
    outcome = states[-1] if len(states) else 'N/A'

    # Survey altitude
    surv_mask = states == 'ALTITUDE_SURVEY'
    survey_alt = float(np.max(data['drone_z'][surv_mask])) if surv_mask.any() else 0.0

    # Planned path length
    try:
        from maze_generator import get_occupancy_grid
        gt_occ = get_occupancy_grid(generate_maze_grid(seed=seed))
        planner = MazeAstar(occ=gt_occ)
        wps = planner.plan((0, 0), (ROWS - 1, COLS - 1))
        sm = planner.smooth_path(wps)
        planned_len = sum(
            math.hypot(sm[i + 1][0] - sm[i][0], sm[i + 1][1] - sm[i][1])
            for i in range(len(sm) - 1)
        )
        n_wps = len(sm)
    except Exception:
        planned_len = 0.0
        n_wps = 0

    # Actual travel distance
    nav_mask = states == 'GUIDING_DOG'
    if nav_mask.any():
        dx = np.diff(data['dog_x'][nav_mask])
        dy = np.diff(data['dog_y'][nav_mask])
        actual_dist = float(np.sum(np.sqrt(dx ** 2 + dy ** 2)))
    else:
        actual_dist = 0.0

    efficiency = (planned_len / actual_dist * 100) if actual_dist > 0 else 0.0

    # Waypoints
    wp_total = int(np.max(data['wp_total'])) if len(data['wp_total']) else 0
    wp_reached = int(np.max(data['wp_idx'])) if len(data['wp_idx']) else 0

    # Stuck events (state transitions into GUIDING_DOG after DOG_STUCK)
    stuck_count = int(np.sum(
        (states[1:] == 'GUIDING_DOG') & (states[:-1] == 'PERCEIVING')
    ))

    lines = [
        ('Outcome',              outcome),
        ('Total mission time',   f'{total_time:.1f} s'),
        ('Survey altitude',      f'{survey_alt:.1f} m'),
        ('Survey time',          f'{time_per_state.get("ALTITUDE_SURVEY", 0):.1f} s'),
        ('Perception time',      f'{time_per_state.get("PERCEIVING", 0):.1f} s'),
        ('Navigation time',      f'{time_per_state.get("GUIDING_DOG", 0):.1f} s'),
        ('Planned path length',  f'{planned_len:.2f} m  ({n_wps} waypoints)'),
        ('Actual travel dist',   f'{actual_dist:.2f} m'),
        ('Path efficiency',      f'{efficiency:.1f} %'),
        ('Waypoints reached',    f'{wp_reached} / {wp_total}'),
        ('Stuck / re-plan events', str(stuck_count)),
        ('Maze seed',            str(seed)),
        ('CSV log',              os.path.basename(csv_path)),
    ]

    col_x = [0.02, 0.52]
    row_h = 1.0 / (len(lines) + 2)
    ax.text(0.5, 1.0 - row_h * 0.6, 'Mission Metrics',
            ha='center', va='center', fontsize=9, fontweight='bold',
            transform=ax.transAxes)
    ax.plot([0, 1], [1.0 - row_h * 1.1] * 2, color='gray', lw=0.5,
            transform=ax.transAxes, clip_on=False)

    for i, (label, value) in enumerate(lines):
        y = 1.0 - row_h * (i + 1.8)
        ax.text(col_x[0], y, label, ha='left', va='center',
                fontsize=7.5, transform=ax.transAxes, color='#333333')
        ax.text(col_x[1], y, value, ha='left', va='center',
                fontsize=7.5, fontweight='bold', transform=ax.transAxes)
        if i % 2 == 0:
            rect = mpatches.FancyBboxPatch(
                (0, y - row_h * 0.45), 1.0, row_h * 0.9,
                boxstyle='square,pad=0', linewidth=0,
                facecolor='#f5f5f5', transform=ax.transAxes, zorder=0
            )
            ax.add_patch(rect)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def build_report(csv_path, seed=42, snapshot_dir=None, out_path=None):
    data = load_csv(csv_path)

    # Infer run timestamp from CSV filename
    base = os.path.basename(csv_path)          # maze_20260401_173904.csv
    run_ts = base.replace('maze_', '').replace('.csv', '')  # 20260401_173904

    if snapshot_dir is None:
        snapshot_dir = os.path.join(os.path.dirname(csv_path), 'snapshots')

    if out_path is None:
        report_dir = os.path.join(os.path.dirname(csv_path), 'reports')
        os.makedirs(report_dir, exist_ok=True)
        out_path = os.path.join(report_dir, f'maze_report_{run_ts}.png')

    fig = plt.figure(figsize=(18, 14))
    fig.suptitle(
        f'Maze Navigation Experiment Report  |  seed={seed}  |  {run_ts}',
        fontsize=13, fontweight='bold', y=0.98
    )

    gs = gridspec.GridSpec(3, 2, figure=fig, hspace=0.42, wspace=0.32,
                           left=0.07, right=0.97, top=0.94, bottom=0.05)

    ax1 = fig.add_subplot(gs[0, 0])
    ax2 = fig.add_subplot(gs[0, 1])
    ax3 = fig.add_subplot(gs[1, 0])
    ax4 = fig.add_subplot(gs[1, 1])
    ax5 = fig.add_subplot(gs[2, 0])
    ax6 = fig.add_subplot(gs[2, 1])

    panel_trajectory(ax1, data, seed)
    panel_state_timeline(ax2, data)
    panel_distance_to_goal(ax3, data)
    panel_velocity(ax4, data)
    panel_snapshots(ax5, snapshot_dir, run_ts)
    panel_metrics(ax6, data, seed, csv_path)

    fig.savefig(out_path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'[MazeReport] Saved: {out_path}')
    return out_path


def main():
    parser = argparse.ArgumentParser(description='Generate maze experiment report PNG.')
    parser.add_argument('csv', nargs='?', help='CSV log path (default: latest in logs/)')
    parser.add_argument('--seed', type=int, default=42, help='Maze seed')
    parser.add_argument('--snapshots', default=None, help='Snapshot directory path')
    parser.add_argument('--out', default=None, help='Output PNG path')
    args = parser.parse_args()

    csv_path = args.csv
    if csv_path is None:
        log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'logs')
        candidates = sorted(glob.glob(os.path.join(log_dir, 'maze_*.csv')))
        if not candidates:
            print('[MazeReport] No CSV log found in logs/. Run the mission first.')
            sys.exit(1)
        csv_path = candidates[-1]
        print(f'[MazeReport] Auto-selected: {csv_path}')

    build_report(csv_path, seed=args.seed,
                 snapshot_dir=args.snapshots, out_path=args.out)


if __name__ == '__main__':
    main()
