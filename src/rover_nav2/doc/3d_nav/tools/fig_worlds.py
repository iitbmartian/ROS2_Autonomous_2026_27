"""Figure: every test world's ground-truth terrain with its benchmark goals.

    python3 fig_worlds.py [OUT_DIR]     (default ~/rover_3d_nav_results/figures)

Reads ground_truth/<world>.npz (ros2 run rover_nav2 ground_truth_heightmap) and the goal sets in
rover_nav2/benchmark/goals. One panel per world, height in one blue ramp, spawn and goals marked.
"""
import os
import sys

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import figstyle as fs  # noqa: E402

HOME = os.path.expanduser('~')
GT = os.path.join(HOME, '.ros/rover_3d_nav/ground_truth')
GOALS = '/home/ram/ROS2_Autonomous_2026_27/src/rover_nav2/rover_nav2/benchmark/goals'
WORLDS = ['marsyard', 'ledge', 'bars', 'slopes', 'crater_field', 'rock_field', 'urc_mix']


def panel(ax, world):
    d = np.load(os.path.join(GT, f'{world}.npz'))
    z, x0, y1, r = d['z'], float(d['x_min']), float(d['y_max']), float(d['res'])
    ext = (x0, x0 + z.shape[1] * r, y1 - z.shape[0] * r, y1)
    with open(os.path.join(GOALS, f'{world}.yaml')) as f:
        gs = yaml.safe_load(f)
    pts = [(gs['spawn']['x'], gs['spawn']['y'])] + [(g['x'], g['y']) for g in gs['goals']]
    xs, ys = zip(*pts)
    # Frame the drive area, not the whole padded ground-truth extent.
    pad = 3.0
    xl = (max(ext[0], min(xs) - pad), min(ext[1], max(xs) + pad))
    yl = (max(ext[2], min(ys) - pad), min(ext[3], max(ys) + pad))
    im = ax.imshow(z, extent=ext, cmap=fs.SEQ_BLUE, origin='upper', interpolation='nearest')
    ax.plot(gs['spawn']['x'], gs['spawn']['y'], marker='o', ms=8, color=fs.INK,
            mec=fs.SURFACE, mew=2, ls='none', label='spawn')
    for g in gs['goals']:
        ax.plot(g['x'], g['y'], marker='*', ms=11, color=fs.CATEGORICAL[1], mec=fs.SURFACE,
                mew=1.5, ls='none')
        ax.annotate(g['id'], (g['x'], g['y']), xytext=(4, 6), textcoords='offset points',
                    fontsize=7, color=fs.INK, path_effects=fs.halo(), rotation=20)
    ax.set_xlim(*xl)
    ax.set_ylim(*yl)
    ax.set_title(world)
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.grid(False)
    cb = ax.figure.colorbar(im, ax=ax, fraction=0.046, pad=0.02)
    cb.set_label('height [m]', color=fs.INK2)
    cb.outline.set_visible(False)


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HOME, 'rover_3d_nav_results/figures')
    os.makedirs(out, exist_ok=True)
    fs.apply()
    worlds = [w for w in WORLDS if os.path.exists(os.path.join(GT, f'{w}.npz'))]
    cols = 4
    rows = (len(worlds) + cols - 1) // cols
    fig, axes = fs.plt.subplots(rows, cols, figsize=(4.2 * cols, 3.8 * rows))
    for ax in axes.flat[len(worlds):]:
        ax.set_visible(False)
    for ax, w in zip(axes.flat, worlds):
        panel(ax, w)
    fig.suptitle('Test worlds: ground-truth terrain, spawn (dot) and benchmark goals (star)',
                 color=fs.INK, fontsize=12)
    fig.tight_layout()
    fs.save(fig, os.path.join(out, 'worlds_ground_truth'))
    print(os.path.join(out, 'worlds_ground_truth.png'))


if __name__ == '__main__':
    main()
