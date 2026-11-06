#!/usr/bin/env python3
"""Height range under each benchmark goal's footprint (plus a margin) in the ground-truth grids,
so goals that sit on rocks or crater rims are caught before a benchmark scores them.
Usage: goal_audit.py [margin_m]"""
import glob, math, os, re, sys
import numpy as np, yaml
REPO = os.path.expanduser('~/ROS2_Autonomous_2026_27'); G = os.path.expanduser('~/rover_3d_nav_results/ground_truth')
def gs_world(gf):
    w = yaml.safe_load(open(gf))['world']
    return f'{REPO}/src/rover_gazebo/worlds/{w}'


def models(path):
    """(name, x, y, height) of each model with a box size or ellipsoid radii."""
    out = []
    for mm in re.finditer(r'<model name="([^"]+)">(.*?)</model>', open(path).read(), re.S):
        p = re.search(r'<pose>([^<]+)</pose>', mm.group(2))
        sz = re.search(r'<size>([\d.]+) ([\d.]+) ([\d.]+)</size>', mm.group(2))
        rd = re.search(r'<radii>([\d.]+) ([\d.]+) ([\d.]+)</radii>', mm.group(2))
        if not p or not (sz or rd):
            continue
        x, y = map(float, p.group(1).split()[:2])
        h = float(sz.group(3)) if sz else 2 * float(rd.group(3))
        out.append((mm.group(1), x, y, h))
    return out


m = float(sys.argv[1]) if len(sys.argv) > 1 else 0.0
L, W = 1.1 + 2 * m, 0.9 + 2 * m
for gf in sorted(glob.glob(f'{REPO}/src/rover_nav2/rover_nav2/benchmark/goals/*.yaml')):
    name = os.path.basename(gf)[:-5]; p = f'{G}/{name}.npz'
    if not os.path.exists(p):
        print(name, 'no ground truth'); continue
    d = np.load(p); z = d['z']; x0, y1, r = float(d['x_min']), float(d['y_max']), float(d['res'])
    for g in yaml.safe_load(open(gf))['goals']:
        c, s = math.cos(g['yaw']), math.sin(g['yaw']); zs = []
        for u in np.arange(-L / 2, L / 2 + 1e-9, r):
            for v in np.arange(-W / 2, W / 2 + 1e-9, r):
                x, y = g['x'] + c * u - s * v, g['y'] + s * u + c * v
                i, j = int(round((y1 - y) / r)), int(round((x - x0) / r))
                if 0 <= i < z.shape[0] and 0 <= j < z.shape[1]: zs.append(z[i, j])
        zs = np.array(zs); zs = zs[np.isfinite(zs)]
        rng = float(zs.max() - zs.min()) if zs.size else float('nan')
        # Some grids (urc_mix) hold the terrain mesh only, so rocks come from the world file too.
        rocks = [f'{n} {h:.2f} m' for n, x, y, h in models(gs_world(gf)) if abs(c * (x - g['x']) + s * (y - g['y'])) < L / 2 + 0.15
                 and abs(-s * (x - g['x']) + c * (y - g['y'])) < W / 2 + 0.15 and h > 0.10]
        bad = rng > 0.10 or rocks
        print(f"{name:13s} {g['id']:16s} ({g['x']:6.1f},{g['y']:6.1f}) range {rng:5.2f} m"
              f"{' rocks ' + ','.join(rocks) if rocks else ''}{'  <-- check' if bad else ''}")
