"""Build the ideal occupancy grid of a Gazebo world: what a perfect mapper should produce.

    ros2 run rover_testing make_reference_map <world.sdf> <out_stem> --spawn X Y

writes out_stem.pgm + out_stem.yaml (Nav2 map_server format, world frame). The
mapping benchmarks compare RTAB-Map's /map against these.

"Ideal" means traversability, not raw geometry, because that is what the 2D map is
for: Nav2 plans in it. So a ramp the rover can climb is free, and a wall is
occupied. The world is rasterised into a 2.5D height field (the top surface of
every static box and cylinder collision, at each cell centre), then

    steep        a cell on a surface steeper than max_slope_deg. Measured over two
                 0.25 m half-baselines that must both exceed it, so a single step
                 (a ledge or a wall face) does not make the ground beside it steep.
    reachable    flood fill from the spawn across 8-neighbours that are not steep
                 and differ in height by at most max_step
    free         reachable
    occupied     not reachable and either steep, raised above the spawn ground by
                 more than max_step, or the higher side of a step out of the
                 reachable area
    unknown      not reachable, flat, at ground level: outside the arena

max_step defaults to 0.15 m, rover_bringup's rtabmap Grid/MaxGroundHeight and
Nav2's min_obstacle_height, and above the 10 cm ledge rover_gazebo's rover is
built to climb. max_slope_deg defaults to 30.

Only <box> and <cylinder> collisions of static models are read (poses composed
model -> link -> collision, rotation included); the ground plane is the zero
height. Meshes, heightmaps and <include>s are skipped with a warning.
"""

import argparse
import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np
from scipy import ndimage

from rover_testing import map_compare


def pose_matrix(text):
    v = [float(t) for t in (text or "0 0 0 0 0 0").split()]
    v += [0.0] * (6 - len(v))
    x, y, z, roll, pitch, yaw = v
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                              math.sin(pitch), math.cos(yaw), math.sin(yaw))
    m = np.eye(4)
    m[:3, :3] = [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                 [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                 [-sp, cp * sr, cp * cr]]
    m[:3, 3] = (x, y, z)
    return m


def shapes(sdf_path):
    """[(kind, world 4x4, params)] for every static box / cylinder collision."""
    root = ET.parse(sdf_path).getroot()
    world = root.find("world")
    if world.findall("include"):
        print(f"warning: {sdf_path}: <include> models are not read", file=sys.stderr)
    out = []
    for model in world.findall("model"):
        if (model.findtext("static") or "false").strip() not in ("true", "1"):
            continue
        m_model = pose_matrix(model.findtext("pose"))
        for link in model.findall("link"):
            m_link = m_model @ pose_matrix(link.findtext("pose"))
            for collision in link.findall("collision"):
                m = m_link @ pose_matrix(collision.findtext("pose"))
                geometry = collision.find("geometry")
                if geometry.find("box") is not None:
                    size = [float(t) for t in geometry.find("box").findtext("size").split()]
                    out.append(("box", m, np.array(size) / 2.0))
                elif geometry.find("cylinder") is not None:
                    c = geometry.find("cylinder")
                    out.append(("cylinder", m, (float(c.findtext("radius")),
                                                float(c.findtext("length")) / 2.0)))
                elif geometry.find("plane") is None:
                    kind = geometry[0].tag if len(geometry) else "?"
                    print(f"warning: {model.get('name')}: <{kind}> collision skipped",
                          file=sys.stderr)
    return out


def top_surface(kind, m, params, x, y):
    """Highest z where the vertical line through (x, y) meets the shape, or -inf."""
    rot, t = m[:3, :3], m[:3, 3]
    # Line p(s) = (x, y, 0) + s * (0, 0, 1) in the shape's frame.
    d = rot.T @ np.array([0.0, 0.0, 1.0])
    px = rot.T[0, 0] * (x - t[0]) + rot.T[0, 1] * (y - t[1]) + rot.T[0, 2] * (0 - t[2])
    py = rot.T[1, 0] * (x - t[0]) + rot.T[1, 1] * (y - t[1]) + rot.T[1, 2] * (0 - t[2])
    pz = rot.T[2, 0] * (x - t[0]) + rot.T[2, 1] * (y - t[1]) + rot.T[2, 2] * (0 - t[2])
    p = (px, py, pz)
    lo = np.full(x.shape, -np.inf)
    hi = np.full(x.shape, np.inf)
    if kind == "box":
        axes = range(3)
        half = params
    else:   # cylinder along local z: slab on z, circle in xy
        axes = (2,)
        half = (None, None, params[1])
    for a in axes:
        if abs(d[a]) < 1e-12:
            outside = np.abs(p[a]) > half[a]
            lo[outside], hi[outside] = np.inf, -np.inf
        else:
            s1, s2 = (-half[a] - p[a]) / d[a], (half[a] - p[a]) / d[a]
            lo, hi = np.maximum(lo, np.minimum(s1, s2)), np.minimum(hi, np.maximum(s1, s2))
    if kind == "cylinder":
        r = params[0]
        a2 = d[0] ** 2 + d[1] ** 2
        if a2 < 1e-12:   # upright: inside the circle or not
            outside = px ** 2 + py ** 2 > r * r
            lo[outside], hi[outside] = np.inf, -np.inf
        else:
            b = 2 * (px * d[0] + py * d[1])
            c = px ** 2 + py ** 2 - r * r
            disc = b * b - 4 * a2 * c
            ok = disc >= 0
            sq = np.sqrt(np.where(ok, disc, 0.0))
            lo = np.where(ok, np.maximum(lo, (-b - sq) / (2 * a2)), np.inf)
            hi = np.where(ok, np.minimum(hi, (-b + sq) / (2 * a2)), -np.inf)
    return np.where(hi >= lo, hi, -np.inf)


def height_field(sdf_path, resolution, margin):
    found = shapes(sdf_path)
    if not found:
        raise ValueError(f"{sdf_path}: no box or cylinder collisions to map")
    # Bounds: every shape's bounding box, plus a margin.
    corners = []
    for kind, m, params in found:
        if kind == "box":
            hx, hy, hz = params
        else:
            hx = hy = params[0]
            hz = params[1]
        for sx in (-1, 1):
            for sy in (-1, 1):
                for sz in (-1, 1):
                    corners.append((m @ np.array([sx * hx, sy * hy, sz * hz, 1.0]))[:2])
    corners = np.array(corners)
    x0, y0 = np.floor((corners.min(axis=0) - margin) / resolution) * resolution
    x1, y1 = np.ceil((corners.max(axis=0) + margin) / resolution) * resolution
    cols, rows = int(round((x1 - x0) / resolution)), int(round((y1 - y0) / resolution))
    grid = {"data": np.zeros((rows, cols), dtype=np.int8), "resolution": resolution,
            "origin": (float(x0), float(y0), 0.0)}
    x, y = map_compare.cell_centers_world(grid)
    h = np.zeros((rows, cols))
    for kind, m, params in found:
        h = np.maximum(h, top_surface(kind, m, params, x, y))
    return h, grid


def classify(h, grid, spawn, max_step=0.15, max_slope_deg=30.0, baseline=0.25):
    res = grid["resolution"]
    k = max(1, int(round(baseline / res)))
    tan_max = math.tan(math.radians(max_slope_deg))
    steep = np.zeros(h.shape, dtype=bool)
    padded = np.pad(h, k, mode="edge")
    rows, cols = h.shape
    for di, dj in ((0, 1), (1, 0), (1, 1), (1, -1)):
        run = k * res * math.hypot(di, dj)
        ahead = padded[k + di * k:k + di * k + rows, k + dj * k:k + dj * k + cols]
        behind = padded[k - di * k:k - di * k + rows, k - dj * k:k - dj * k + cols]
        up, down = (ahead - h) / run, (h - behind) / run
        # Both halves sloping the same way and both too steep: a slope, not a step.
        steep |= (np.sign(up) == np.sign(down)) & (np.minimum(np.abs(up), np.abs(down)) > tan_max)

    # Flood fill from the spawn across small steps between non-steep cells.
    i0 = int((spawn[1] - grid["origin"][1]) / res)
    j0 = int((spawn[0] - grid["origin"][0]) / res)
    if not (0 <= i0 < rows and 0 <= j0 < cols):
        raise ValueError(f"spawn {spawn} is outside the mapped area")
    if steep[i0, j0]:
        raise ValueError(f"spawn {spawn} is on a steep cell")
    reachable = np.zeros(h.shape, dtype=bool)
    reachable[i0, j0] = True
    frontier = [(i0, j0)]
    neighbours = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
    while frontier:
        nxt = []
        for i, j in frontier:
            for di, dj in neighbours:
                a, b = i + di, j + dj
                if 0 <= a < rows and 0 <= b < cols and not reachable[a, b] \
                        and not steep[a, b] and abs(h[a, b] - h[i, j]) <= max_step:
                    reachable[a, b] = True
                    nxt.append((a, b))
        frontier = nxt

    ground = h[i0, j0]
    near_reachable = ndimage.binary_dilation(reachable, structure=np.ones((3, 3), dtype=bool))
    edge = near_reachable & ~reachable   # just outside the reachable area: a step up or down
    occupied = ~reachable & (steep | (h > ground + max_step) | (edge & (h > ground)))
    data = np.full(h.shape, map_compare.UNKNOWN, dtype=np.int8)
    data[reachable] = map_compare.FREE
    data[occupied] = map_compare.OCCUPIED
    return data


def build(sdf_path, out_stem, spawn, resolution=0.05, margin=0.5, max_step=0.15,
          max_slope_deg=30.0):
    h, grid = height_field(sdf_path, resolution, margin)
    grid["data"] = classify(h, grid, spawn, max_step, max_slope_deg)
    os.makedirs(os.path.dirname(out_stem) or ".", exist_ok=True)
    path = map_compare.save_map_yaml(grid, out_stem)
    free = int(np.count_nonzero(grid["data"] == map_compare.FREE))
    occ = int(np.count_nonzero(grid["data"] == map_compare.OCCUPIED))
    return (f"wrote {path}: {grid['data'].shape[1]}x{grid['data'].shape[0]} cells, "
            f"{free * resolution ** 2:.1f} m2 free, {occ * resolution ** 2:.1f} m2 occupied")


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("world", help=".sdf world file")
    parser.add_argument("out_stem", help="output path without extension")
    parser.add_argument("--spawn", nargs=2, type=float, default=(0.0, 0.0), metavar=("X", "Y"),
                        help="world position the rover starts at; seeds the reachable area")
    parser.add_argument("--resolution", type=float, default=0.05)
    parser.add_argument("--max-step", type=float, default=0.15)
    parser.add_argument("--max-slope-deg", type=float, default=30.0)
    args = parser.parse_args()
    print(build(args.world, args.out_stem, tuple(args.spawn), args.resolution,
                max_step=args.max_step, max_slope_deg=args.max_slope_deg))


if __name__ == "__main__":
    main()
