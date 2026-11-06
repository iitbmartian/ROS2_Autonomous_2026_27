"""Figures from benchmark results: success rate, time to goal and unsafe entries per world and
config, and every run's ground-truth trajectory over the world's terrain.

    python3 fig_benchmark.py OUT_NAME RESULTS_DIR [RESULTS_DIR ...] [TAG=2d|3d|swerve ...]

Each RESULTS_DIR is a benchmark --out directory (runs.csv, poses/). Configs keep a fixed colour
(figstyle.CONFIG_COLOR, matched on the tag prefix 2d, 3d or swerve). Writes
~/rover_3d_nav_results/figures/<OUT_NAME>_summary.{png,svg}, <OUT_NAME>_traj_<world>.{png,svg}
and <OUT_NAME>_table.csv.
"""
import csv
import os
import sys
from collections import defaultdict

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import figstyle as fs  # noqa: E402

HOME = os.path.expanduser('~')
OUT = os.path.join(HOME, 'rover_3d_nav_results/figures')
GT = os.path.join(HOME, '.ros/rover_3d_nav/ground_truth')
GOALS = '/home/ram/ROS2_Autonomous_2026_27/src/rover_nav2/rover_nav2/benchmark/goals'


TAG_KIND = {}   # tag -> 2d, 3d or swerve, from TAG=KIND arguments


def color_of(cfg):
    if cfg in TAG_KIND:
        return fs.CONFIG_COLOR[TAG_KIND[cfg]]
    for key in ('swerve', '3d', '2d'):
        if key in cfg:
            return fs.CONFIG_COLOR[key]
    return fs.CATEGORICAL[3]


def load(dirs):
    rows = []
    for d in dirs:
        with open(os.path.join(d, 'runs.csv')) as f:
            for r in csv.DictReader(f):
                r['_dir'] = d
                rows.append(r)
    return rows


def world_of(r):
    return r['world'].replace('.sdf', '')


def fnum(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return float('nan')


def summary(rows, name):
    # Startup failures say nothing about navigation, they are left out and counted separately.
    scored = [r for r in rows if r['status'] != 'startup_failed']
    worlds = sorted({world_of(r) for r in scored})
    cfgs = sorted({r['config'] for r in scored})
    stat = defaultdict(list)
    for r in scored:
        stat[(world_of(r), r['config'])].append(r)
    with open(os.path.join(OUT, f'{name}_table.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['world', 'config', 'runs', 'success_rate', 'median_time_to_goal_s',
                    'unsafe_entries', 'recoveries_mean', 'startup_failed'])
        for wd in worlds:
            for c in cfgs:
                rs = stat[(wd, c)]
                if not rs:
                    continue
                ok = [r for r in rs if r['success'] == '1']
                sf = sum(1 for r in rows if world_of(r) == wd and r['config'] == c
                         and r['status'] == 'startup_failed')
                w.writerow([wd, c, len(rs), f'{len(ok) / len(rs):.3f}',
                            f"{np.median([fnum(r['time_to_goal_s']) for r in ok]):.1f}" if ok else '',
                            sum(int(fnum(r['unsafe_entries']) or 0) for r in rs
                                if not np.isnan(fnum(r['unsafe_entries']))),
                            f"{np.nanmean([fnum(r['recoveries']) for r in rs]):.2f}", sf])

    fs.apply()
    metrics = [('success rate', lambda rs: np.mean([r['success'] == '1' for r in rs]), (0, 1.05)),
               ('median time to goal [s], successes',
                lambda rs: np.median([fnum(r['time_to_goal_s']) for r in rs if r['success'] == '1'])
                if any(r['success'] == '1' for r in rs) else np.nan, None),
               ('unsafe entries (sum)',
                lambda rs: np.nansum([fnum(r['unsafe_entries']) for r in rs]), None)]
    fig, axes = fs.plt.subplots(len(metrics), 1, figsize=(max(6, 1.3 * len(worlds) * len(cfgs)),
                                                          2.6 * len(metrics)), sharex=True)
    width = 0.8 / max(len(cfgs), 1)
    xs = np.arange(len(worlds))
    for ax, (label, fn, ylim) in zip(np.atleast_1d(axes), metrics):
        for k, c in enumerate(cfgs):
            vals = [fn(stat[(wd, c)]) if stat[(wd, c)] else np.nan for wd in worlds]
            bx = xs + (k - (len(cfgs) - 1) / 2) * width
            ax.bar(bx, vals, width * 0.92,
                   color=color_of(c), label=c, edgecolor=fs.SURFACE, linewidth=1)
            # A zero is a result, so it gets a label. No data stays blank.
            for xb, v in zip(bx, vals):
                if v == 0:
                    ax.annotate('0', (xb, 0), xytext=(0, 3), textcoords='offset points',
                                ha='center', fontsize=8, color=fs.INK2)
        ax.set_ylabel(label)
        ax.grid(axis='x', visible=False)
        if ylim:
            ax.set_ylim(*ylim)
        else:
            top = ax.get_ylim()[1]
            ax.set_ylim(0, top if top > 0 else 1)
    axes[0].legend(ncol=len(cfgs), loc='lower left', bbox_to_anchor=(0, 1.02))
    axes[-1].set_xticks(xs, worlds)
    fig.tight_layout()
    fs.save(fig, os.path.join(OUT, f'{name}_summary'))


def trajectories(rows, name):
    fs.apply()
    for wd in sorted({world_of(r) for r in rows}):
        gt_path = os.path.join(GT, f'{wd}.npz')
        if not os.path.exists(gt_path):
            continue
        d = np.load(gt_path)
        z, x0, y1, res = d['z'], float(d['x_min']), float(d['y_max']), float(d['res'])
        ext = (x0, x0 + z.shape[1] * res, y1 - z.shape[0] * res, y1)
        with open(os.path.join(GOALS, f'{wd}.yaml')) as f:
            gs = yaml.safe_load(f)
        fig, ax = fs.plt.subplots(figsize=(7, 6))
        im = ax.imshow(z, extent=ext, cmap=fs.SEQ_BLUE, origin='upper', alpha=0.85)
        allx, ally = [gs['spawn']['x']], [gs['spawn']['y']]
        seen = set()
        for r in rows:
            if world_of(r) != wd:
                continue
            p = os.path.join(r['_dir'], 'poses', f"{r['run_id']}.csv")
            if not os.path.exists(p):
                continue
            a = np.genfromtxt(p, delimiter=',', names=True)
            if a.size < 2:
                continue
            c = color_of(r['config'])
            lbl = r['config'] if r['config'] not in seen else None
            seen.add(r['config'])
            ax.plot(a['x'], a['y'], color=c, lw=1.4, alpha=0.9 if r['success'] == '1' else 0.45,
                    ls='-' if r['success'] == '1' else '--', label=lbl)
            allx += list(a['x'])
            ally += list(a['y'])
        for g in gs['goals']:
            ax.plot(g['x'], g['y'], marker='*', ms=13, color=fs.INK, mec=fs.SURFACE, mew=1.5)
            ax.annotate(g['id'], (g['x'], g['y']), xytext=(5, 5), textcoords='offset points',
                        fontsize=8, color=fs.INK, path_effects=fs.halo())
            allx.append(g['x'])
            ally.append(g['y'])
        ax.plot(gs['spawn']['x'], gs['spawn']['y'], 'o', ms=8, color=fs.INK, mec=fs.SURFACE, mew=2)
        ax.set_xlim(min(allx) - 2, max(allx) + 2)
        ax.set_ylim(min(ally) - 2, max(ally) + 2)
        ax.set_aspect('equal')
        ax.grid(False)
        ax.set_title(f'{wd}: ground-truth trajectories', loc='left', pad=16)
        ax.text(0, 1.01, 'solid: success, dashed: failure', transform=ax.transAxes,
                fontsize=8, color=fs.INK2, va='bottom')
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        from mpl_toolkits.axes_grid1 import make_axes_locatable
        cax = make_axes_locatable(ax).append_axes('right', size='3%', pad=0.08)
        cb = fig.colorbar(im, cax=cax)
        cb.set_label('terrain height [m]', color=fs.INK2)
        cb.outline.set_visible(False)
        if seen:
            ax.legend(loc='upper left')
        fs.save(fig, os.path.join(OUT, f'{name}_traj_{wd}'))


def main():
    name = sys.argv[1]
    dirs = []
    for arg in sys.argv[2:]:
        if '=' in arg and not os.path.isdir(os.path.expanduser(arg)):
            tag, kind = arg.split('=', 1)
            TAG_KIND[tag] = kind
        else:
            dirs.append(os.path.expanduser(arg))
    os.makedirs(OUT, exist_ok=True)
    rows = load(dirs)
    summary(rows, name)
    trajectories(rows, name)
    print(f'{OUT}/{name}_summary.png, {name}_traj_*.png, {name}_table.csv')


if __name__ == '__main__':
    main()
