#!/usr/bin/env python3
"""
Post-mission analysis for leader-follower coordination.

Reads a CSV log produced by leader_follower_ros2_node and generates
a 6-subplot dashboard quantifying coverage and coordination quality.

Usage:
    python3 analyze_mission.py                     # uses latest log
    python3 analyze_mission.py logs/mission_XYZ.csv  # specific file
"""

import sys
import os
import glob
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap

# ------------------------------------------------------------------
# Configuration
# ------------------------------------------------------------------
WORKSPACE_ORIGIN = (0.0, 0.0)
WORKSPACE_SIZE = 10.0
CELL_SIZE = 0.5          # grid resolution for coverage (meters)
SENSING_RADIUS = 1.5     # each dog "covers" cells within this radius
RATE_WINDOW = 40         # steps for moving-average coverage rate


def load_csv(path: str) -> pd.DataFrame:
    df = pd.read_csv(path)
    for col in df.columns:
        if col != 'step':
            df[col] = pd.to_numeric(df[col], errors='coerce')
    return df


def build_coverage_grid(df: pd.DataFrame):
    """
    Returns:
        grid_all:  2D int array — total visit count per cell (either dog)
        grid_dog0: 2D bool — cells covered by dog0
        grid_dog1: 2D bool — cells covered by dog1
        xs, ys:    cell center coordinates
        coverage_over_time: array of cumulative coverage fraction per step
    """
    x0, y0 = WORKSPACE_ORIGIN
    nx = int(np.ceil(WORKSPACE_SIZE / CELL_SIZE))
    ny = int(np.ceil(WORKSPACE_SIZE / CELL_SIZE))

    xs = x0 + (np.arange(nx) + 0.5) * CELL_SIZE
    ys = y0 + (np.arange(ny) + 0.5) * CELL_SIZE
    cx, cy = np.meshgrid(xs, ys)  # (ny, nx)
    cx_flat = cx.ravel()  # (nx*ny,)
    cy_flat = cy.ravel()

    total_cells = nx * ny
    r2 = SENSING_RADIUS ** 2

    # Vectorized: compute which cells each position covers
    dog0_x = df['dog0_x'].values
    dog0_y = df['dog0_y'].values
    dog1_x = df['dog1_x'].values
    dog1_y = df['dog1_y'].values

    grid_dog0 = np.zeros(total_cells, dtype=bool)
    grid_dog1 = np.zeros(total_cells, dtype=bool)
    grid_all = np.zeros(total_cells, dtype=int)
    coverage_over_time = np.zeros(len(df))

    for i in range(len(df)):
        d0 = (cx_flat - dog0_x[i])**2 + (cy_flat - dog0_y[i])**2 <= r2
        d1 = (cx_flat - dog1_x[i])**2 + (cy_flat - dog1_y[i])**2 <= r2

        grid_dog0 |= d0
        grid_dog1 |= d1
        grid_all += d0.astype(int) + d1.astype(int)

        coverage_over_time[i] = (grid_dog0 | grid_dog1).sum() / total_cells

    grid_dog0 = grid_dog0.reshape(ny, nx)
    grid_dog1 = grid_dog1.reshape(ny, nx)
    grid_all = grid_all.reshape(ny, nx)

    return grid_all, grid_dog0, grid_dog1, xs, ys, coverage_over_time


def compute_separations(df: pd.DataFrame):
    drone = df[['drone_x', 'drone_y']].values
    dog0 = df[['dog0_x', 'dog0_y']].values
    dog1 = df[['dog1_x', 'dog1_y']].values

    sep_drone_dog0 = np.linalg.norm(drone - dog0, axis=1)
    sep_drone_dog1 = np.linalg.norm(drone - dog1, axis=1)
    sep_dog0_dog1 = np.linalg.norm(dog0 - dog1, axis=1)

    return sep_drone_dog0, sep_drone_dog1, sep_dog0_dog1


def compute_angular_spread(df: pd.DataFrame):
    """Angle between drone->dog0 and drone->dog1 vectors."""
    drone = df[['drone_x', 'drone_y']].values
    dog0 = df[['dog0_x', 'dog0_y']].values
    dog1 = df[['dog1_x', 'dog1_y']].values

    v0 = dog0 - drone
    v1 = dog1 - drone

    # Normalize
    n0 = np.linalg.norm(v0, axis=1, keepdims=True)
    n1 = np.linalg.norm(v1, axis=1, keepdims=True)
    n0 = np.maximum(n0, 1e-6)
    n1 = np.maximum(n1, 1e-6)

    cos_angle = np.sum((v0 / n0) * (v1 / n1), axis=1)
    cos_angle = np.clip(cos_angle, -1.0, 1.0)
    angles_deg = np.degrees(np.arccos(cos_angle))

    return angles_deg


def compute_coverage_rate(coverage_over_time, dt_per_step):
    """New coverage fraction per second, smoothed with moving average."""
    dcov = np.diff(coverage_over_time, prepend=0.0)
    rate = dcov / dt_per_step  # fraction/sec
    # Moving average
    kernel = np.ones(RATE_WINDOW) / RATE_WINDOW
    rate_smooth = np.convolve(rate, kernel, mode='same')
    return rate_smooth


def parse_strategy_name(csv_path: str) -> str:
    """Extract strategy name from CSV filename like mission_naive_20260220_102033.csv"""
    basename = os.path.basename(csv_path)
    # Strip 'mission_' prefix and '_YYYYMMDD_HHMMSS.csv' suffix
    parts = basename.replace('mission_', '', 1).rsplit('_', 2)
    if len(parts) >= 3:
        return parts[0]
    return 'unknown'


def print_summary(df, coverage_over_time, grid_dog0, grid_dog1,
                  sep_drone_dog0, sep_drone_dog1, sep_dog0_dog1, angles_deg,
                  strategy_name='unknown'):
    total_cells = grid_dog0.size
    covered_dog0 = grid_dog0.sum()
    covered_dog1 = grid_dog1.sum()
    covered_both = (grid_dog0 & grid_dog1).sum()
    covered_any = (grid_dog0 | grid_dog1).sum()

    print('=' * 60)
    print('           MISSION ANALYSIS SUMMARY')
    print('=' * 60)
    print(f'  Strategy            : {strategy_name}')
    print(f'  Duration            : {df["t"].iloc[-1]:.1f}s  ({len(df)} steps)')
    print(f'  Grid                : {CELL_SIZE}m cells, {SENSING_RADIUS}m sensing radius')
    print()
    print(f'  --- Coverage ---')
    print(f'  Final coverage      : {coverage_over_time[-1]*100:.1f}%  ({covered_any}/{total_cells} cells)')
    print(f'  Dog0 coverage       : {covered_dog0/total_cells*100:.1f}%')
    print(f'  Dog1 coverage       : {covered_dog1/total_cells*100:.1f}%')
    print(f'  Overlap (both dogs) : {covered_both/max(covered_any,1)*100:.1f}% of covered area')
    print()
    print(f'  --- Separations ---')
    print(f'  Drone-Dog0          : mean={np.mean(sep_drone_dog0):.2f}m, '
          f'min={np.min(sep_drone_dog0):.2f}m, max={np.max(sep_drone_dog0):.2f}m')
    print(f'  Drone-Dog1          : mean={np.mean(sep_drone_dog1):.2f}m, '
          f'min={np.min(sep_drone_dog1):.2f}m, max={np.max(sep_drone_dog1):.2f}m')
    print(f'  Dog0-Dog1           : mean={np.mean(sep_dog0_dog1):.2f}m, '
          f'min={np.min(sep_dog0_dog1):.2f}m, max={np.max(sep_dog0_dog1):.2f}m')
    print()
    print(f'  --- Angular spread ---')
    print(f'  Mean angle          : {np.mean(angles_deg):.1f} deg')
    print(f'  Median angle        : {np.median(angles_deg):.1f} deg')
    print('=' * 60)


def plot_dashboard(df, coverage_over_time, grid_all, grid_dog0, grid_dog1,
                   xs, ys, sep_drone_dog0, sep_drone_dog1, sep_dog0_dog1,
                   angles_deg, coverage_rate, output_path,
                   strategy_name='unknown'):

    t = df['t'].values
    fig, axes = plt.subplots(2, 3, figsize=(18, 10))
    fig.suptitle(f'Leader-Follower Mission Analysis — strategy: {strategy_name}',
                 fontsize=16, fontweight='bold')

    # --- A: Coverage vs time ---
    ax = axes[0, 0]
    ax.plot(t, coverage_over_time * 100, color='tab:blue', linewidth=2)
    # Ideal linear reference
    ax.plot([t[0], t[-1]], [0, 100], '--', color='gray', alpha=0.5, label='Ideal linear')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Coverage (%)')
    ax.set_title('A. Cumulative Coverage')
    ax.set_ylim(0, 105)
    ax.legend()
    ax.grid(True, alpha=0.3)

    # --- B: Coverage heatmap ---
    ax = axes[0, 1]
    x0, y0 = WORKSPACE_ORIGIN
    extent = [x0, x0 + WORKSPACE_SIZE, y0, y0 + WORKSPACE_SIZE]

    # Custom colormap: black (unvisited) -> blue -> yellow (many visits)
    cmap = LinearSegmentedColormap.from_list('cov', ['#1a1a2e', '#16213e', '#0f3460', '#e94560', '#ffcc00'])
    uncovered_mask = ~(grid_dog0 | grid_dog1)
    display = grid_all.astype(float)
    display[uncovered_mask] = -1

    im = ax.imshow(display, origin='lower', extent=extent, cmap=cmap,
                   vmin=0, aspect='equal', interpolation='nearest')
    # Mark uncovered cells
    uncov_display = np.ma.masked_where(~uncovered_mask, np.ones_like(display))
    ax.imshow(uncov_display, origin='lower', extent=extent, cmap='Greys',
              vmin=0, vmax=2, alpha=0.7, aspect='equal', interpolation='nearest')
    fig.colorbar(im, ax=ax, label='Visit count', shrink=0.8)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('B. Coverage Heatmap')

    # --- C: Agent trajectories ---
    ax = axes[0, 2]
    ax.plot(df['drone_x'].values, df['drone_y'].values, '-', color='cyan', linewidth=1.0,
            alpha=0.7, label='Drone')
    ax.plot(df['dog0_x'].values, df['dog0_y'].values, '-', color='tab:green', linewidth=1.0,
            alpha=0.7, label='Dog0')
    ax.plot(df['dog1_x'].values, df['dog1_y'].values, '-', color='tab:purple', linewidth=1.0,
            alpha=0.7, label='Dog1')
    # Start/end markers
    ax.plot(df['drone_x'].iloc[0], df['drone_y'].iloc[0], 'o', color='white',
            markersize=8, zorder=5)
    ax.plot(df['drone_x'].iloc[-1], df['drone_y'].iloc[-1], 's', color='white',
            markersize=8, zorder=5)
    # Workspace boundary
    bx = [x0, x0 + WORKSPACE_SIZE, x0 + WORKSPACE_SIZE, x0, x0]
    by = [y0, y0, y0 + WORKSPACE_SIZE, y0 + WORKSPACE_SIZE, y0]
    ax.plot(bx, by, '--', color='white', linewidth=1.5, alpha=0.8)
    ax.set_xlim(x0 - 1, x0 + WORKSPACE_SIZE + 1)
    ax.set_ylim(y0 - 1, y0 + WORKSPACE_SIZE + 1)
    ax.set_aspect('equal')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('C. Agent Trajectories')
    ax.legend(fontsize=8)
    ax.set_facecolor('#1a1a2e')
    ax.grid(True, alpha=0.2, color='gray')

    # --- D: Separations over time ---
    ax = axes[1, 0]
    ax.plot(t, sep_drone_dog0, color='tab:green', linewidth=1, alpha=0.8, label='Drone-Dog0')
    ax.plot(t, sep_drone_dog1, color='tab:purple', linewidth=1, alpha=0.8, label='Drone-Dog1')
    ax.plot(t, sep_dog0_dog1, color='tab:orange', linewidth=1, alpha=0.8, label='Dog0-Dog1')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Distance (m)')
    ax.set_title('D. Agent Separations')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # --- E: Angular spread ---
    ax = axes[1, 1]
    ax.plot(t, angles_deg, color='tab:red', linewidth=0.8, alpha=0.6)
    # Smoothed
    kernel = np.ones(RATE_WINDOW) / RATE_WINDOW
    smooth = np.convolve(angles_deg, kernel, mode='same')
    ax.plot(t, smooth, color='tab:red', linewidth=2, label=f'Smoothed ({RATE_WINDOW}-step)')
    ax.axhline(180, color='gray', linestyle='--', alpha=0.4, label='Max spread (180 deg)')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Angle (deg)')
    ax.set_title('E. Inter-Dog Angular Spread')
    ax.set_ylim(0, 200)
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # --- F: Coverage rate ---
    ax = axes[1, 2]
    ax.plot(t, coverage_rate * 100, color='tab:blue', linewidth=1.5)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Coverage Rate (%/s)')
    ax.set_title('F. Coverage Rate (smoothed)')
    ax.set_ylim(bottom=0)
    ax.grid(True, alpha=0.3)

    plt.tight_layout(rect=[0, 0, 1, 0.95])
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f'Dashboard saved: {output_path}')
    if os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'):
        plt.show()
    else:
        print('No display detected, skipping interactive window.')
        plt.close(fig)


def main():
    # Find CSV file
    if len(sys.argv) > 1:
        csv_path = sys.argv[1]
    else:
        log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'logs')
        csvs = sorted(glob.glob(os.path.join(log_dir, 'mission_*.csv')))
        if not csvs:
            print(f'No mission logs found in {log_dir}/')
            print('Run a mission first, or pass a CSV path as argument.')
            sys.exit(1)
        csv_path = csvs[-1]
        print(f'Using latest log: {csv_path}')

    strategy_name = parse_strategy_name(csv_path)
    df = load_csv(csv_path)
    print(f'Strategy: {strategy_name}')
    print(f'Loaded {len(df)} rows, t=[{df["t"].iloc[0]:.1f}, {df["t"].iloc[-1]:.1f}]s')

    if len(df) < 2:
        print('Not enough data to analyze.')
        sys.exit(1)

    # Compute metrics
    grid_all, grid_dog0, grid_dog1, xs, ys, coverage_over_time = build_coverage_grid(df)
    sep_drone_dog0, sep_drone_dog1, sep_dog0_dog1 = compute_separations(df)
    angles_deg = compute_angular_spread(df)

    dt_per_step = (df['t'].iloc[-1] - df['t'].iloc[0]) / max(len(df) - 1, 1)
    coverage_rate = compute_coverage_rate(coverage_over_time, max(dt_per_step, 1e-6))

    # Print text summary
    print_summary(df, coverage_over_time, grid_dog0, grid_dog1,
                  sep_drone_dog0, sep_drone_dog1, sep_dog0_dog1, angles_deg,
                  strategy_name=strategy_name)

    # Plot dashboard
    output_path = csv_path.replace('.csv', '_dashboard.png')
    plot_dashboard(df, coverage_over_time, grid_all, grid_dog0, grid_dog1,
                   xs, ys, sep_drone_dog0, sep_drone_dog1, sep_dog0_dog1,
                   angles_deg, coverage_rate, output_path,
                   strategy_name=strategy_name)


if __name__ == '__main__':
    main()
