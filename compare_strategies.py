#!/usr/bin/env python3
"""
Compare multiple leader-follower mission runs side by side.

Reads CSV logs from different strategy runs and produces an overlay
dashboard for direct comparison.

Usage:
    python3 compare_strategies.py logs/mission_naive_*.csv logs/mission_lateral_offset_*.csv
    python3 compare_strategies.py logs/mission_naive_20260220_102033.csv logs/mission_lateral_offset_20260220_103000.csv
"""

import sys
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# Reuse metric functions from analyze_mission
from analyze_mission import (
    load_csv, build_coverage_grid, compute_separations,
    compute_angular_spread, compute_coverage_rate, parse_strategy_name,
    WORKSPACE_ORIGIN, WORKSPACE_SIZE, CELL_SIZE, SENSING_RADIUS, RATE_WINDOW,
)

COLORS = ['tab:blue', 'tab:orange', 'tab:green', 'tab:red', 'tab:purple']


def main():
    if len(sys.argv) < 3:
        print('Usage: python3 compare_strategies.py <csv1> <csv2> [csv3 ...]')
        print('  Accepts glob patterns: logs/mission_naive_*.csv')
        sys.exit(1)

    csv_paths = sys.argv[1:]
    runs = []

    for path in csv_paths:
        if not os.path.exists(path):
            print(f'Warning: {path} not found, skipping')
            continue
        strategy = parse_strategy_name(path)
        df = load_csv(path)
        grid_all, g0, g1, xs, ys, cov = build_coverage_grid(df)
        sep0, sep1, sep_dd = compute_separations(df)
        angles = compute_angular_spread(df)
        dt = (df['t'].iloc[-1] - df['t'].iloc[0]) / max(len(df) - 1, 1)
        rate = compute_coverage_rate(cov, max(dt, 1e-6))

        covered_both = (g0 & g1).sum()
        covered_any = (g0 | g1).sum()
        overlap = covered_both / max(covered_any, 1)

        runs.append(dict(
            path=path, strategy=strategy, df=df,
            coverage=cov, grid_all=grid_all, grid_dog0=g0, grid_dog1=g1,
            sep_drone_dog0=sep0, sep_drone_dog1=sep1, sep_dog_dog=sep_dd,
            angles=angles, rate=rate, overlap=overlap,
        ))
        print(f'  [{strategy}] {len(df)} steps, '
              f'coverage={cov[-1]*100:.1f}%, overlap={overlap*100:.1f}%')

    if len(runs) < 2:
        print('Need at least 2 runs to compare.')
        sys.exit(1)

    # --- Print comparison table ---
    print()
    print('=' * 72)
    print(f'{"Strategy":<20} {"Coverage":>10} {"Overlap":>10} '
          f'{"Dog-Dog sep":>12} {"Ang spread":>12}')
    print('-' * 72)
    for r in runs:
        print(f'{r["strategy"]:<20} '
              f'{r["coverage"][-1]*100:>9.1f}% '
              f'{r["overlap"]*100:>9.1f}% '
              f'{np.mean(r["sep_dog_dog"]):>10.2f}m '
              f'{np.mean(r["angles"]):>10.1f} deg')
    print('=' * 72)

    # --- Plot comparison dashboard ---
    fig, axes = plt.subplots(2, 3, figsize=(18, 10))
    fig.suptitle('Strategy Comparison', fontsize=16, fontweight='bold')

    # A: Coverage curves overlay
    ax = axes[0, 0]
    for i, r in enumerate(runs):
        t = r['df']['t'].values
        ax.plot(t, r['coverage'] * 100, color=COLORS[i % len(COLORS)],
                linewidth=2, label=r['strategy'])
    ax.plot([0, max(r['df']['t'].max() for r in runs)], [0, 100],
            '--', color='gray', alpha=0.4, label='Ideal')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Coverage (%)')
    ax.set_title('A. Coverage vs Time')
    ax.set_ylim(0, 105)
    ax.legend()
    ax.grid(True, alpha=0.3)

    # B: Final metrics bar chart
    ax = axes[0, 1]
    names = [r['strategy'] for r in runs]
    x = np.arange(len(names))
    width = 0.35
    final_cov = [r['coverage'][-1] * 100 for r in runs]
    overlaps = [r['overlap'] * 100 for r in runs]
    bars1 = ax.bar(x - width/2, final_cov, width, label='Coverage %',
                   color=[COLORS[i % len(COLORS)] for i in range(len(runs))])
    bars2 = ax.bar(x + width/2, overlaps, width, label='Overlap %',
                   color=[COLORS[i % len(COLORS)] for i in range(len(runs))], alpha=0.5)
    ax.set_xticks(x)
    ax.set_xticklabels(names, rotation=15, ha='right')
    ax.set_ylabel('Percent (%)')
    ax.set_title('B. Final Coverage & Overlap')
    ax.legend()
    ax.grid(True, alpha=0.3, axis='y')
    for bar, val in zip(bars1, final_cov):
        ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 1,
                f'{val:.1f}', ha='center', va='bottom', fontsize=9)

    # C: Trajectories side by side (up to 3 runs)
    ax = axes[0, 2]
    x0, y0 = WORKSPACE_ORIGIN
    bx = [x0, x0 + WORKSPACE_SIZE, x0 + WORKSPACE_SIZE, x0, x0]
    by = [y0, y0, y0 + WORKSPACE_SIZE, y0 + WORKSPACE_SIZE, y0]
    ax.plot(bx, by, '--', color='white', linewidth=1.5, alpha=0.8)
    for i, r in enumerate(runs):
        c = COLORS[i % len(COLORS)]
        df = r['df']
        ax.plot(df['dog0_x'].values, df['dog0_y'].values, '-', color=c,
                linewidth=0.8, alpha=0.5)
        ax.plot(df['dog1_x'].values, df['dog1_y'].values, '--', color=c,
                linewidth=0.8, alpha=0.5, label=f'{r["strategy"]}')
    ax.set_xlim(x0 - 1, x0 + WORKSPACE_SIZE + 1)
    ax.set_ylim(y0 - 1, y0 + WORKSPACE_SIZE + 1)
    ax.set_aspect('equal')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('C. Dog Trajectories (solid=dog0, dashed=dog1)')
    ax.legend(fontsize=8)
    ax.set_facecolor('#1a1a2e')
    ax.grid(True, alpha=0.2, color='gray')

    # D: Dog-dog separation overlay
    ax = axes[1, 0]
    for i, r in enumerate(runs):
        t = r['df']['t'].values
        ax.plot(t, r['sep_dog_dog'], color=COLORS[i % len(COLORS)],
                linewidth=1, alpha=0.7, label=r['strategy'])
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Dog0-Dog1 Distance (m)')
    ax.set_title('D. Inter-Dog Separation')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # E: Angular spread overlay
    ax = axes[1, 1]
    kernel = np.ones(RATE_WINDOW) / RATE_WINDOW
    for i, r in enumerate(runs):
        t = r['df']['t'].values
        smooth = np.convolve(r['angles'], kernel, mode='same')
        ax.plot(t, smooth, color=COLORS[i % len(COLORS)],
                linewidth=2, label=r['strategy'])
    ax.axhline(180, color='gray', linestyle='--', alpha=0.4)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Angle (deg)')
    ax.set_title('E. Angular Spread (smoothed)')
    ax.set_ylim(0, 200)
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # F: Coverage rate overlay
    ax = axes[1, 2]
    for i, r in enumerate(runs):
        t = r['df']['t'].values
        ax.plot(t, r['rate'] * 100, color=COLORS[i % len(COLORS)],
                linewidth=1.5, label=r['strategy'])
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Coverage Rate (%/s)')
    ax.set_title('F. Coverage Rate')
    ax.set_ylim(bottom=0)
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    plt.tight_layout(rect=[0, 0, 1, 0.95])

    output_path = os.path.join(os.path.dirname(csv_paths[0]), 'comparison_dashboard.png')
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f'\nComparison dashboard saved: {output_path}')

    if os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'):
        plt.show()
    else:
        plt.close(fig)


if __name__ == '__main__':
    main()
