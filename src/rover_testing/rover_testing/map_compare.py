"""Score an occupancy grid against a reference ("ideal") map of the same world.

Maps are dicts:

    data        int8 [rows, cols], row 0 at origin y: -1 unknown, 0 free, 100 occupied
    resolution  m per cell
    origin      (x, y, yaw) of cell (0, 0)'s corner, in the map's own frame

Reference maps (maps/*.yaml, made by make_reference_map.py) are in the world frame.
RTAB-Map's /map is in the map frame, which starts at the rover's spawn pose, so
the actual map is moved into the world with the spawn pose before comparing. No
registration is done on purpose: a map that is right in shape but drifted or
rotated is a wrong map for a rover that plans in it.

Everything is evaluated on the reference grid, the actual map resampled into it
(nearest cell). With d = match_distance:

    precision   share of the actual occupied cells that lie within d of a reference
                obstacle. Low: phantom obstacles (e.g. a climbable slope mapped as a
                wall, a ghost ring from pitching sensors, drift smearing walls).
    recall      share of the reference obstacle *outline* cells (those bordering
                reference free space; no sensor sees an obstacle's inside) that the
                rover observed (some actual known cell within d) and that have an
                actual occupied cell within d. Low: missed or misplaced obstacles.
    coverage    share of the reference free cells the actual map knows (free or
                occupied). How much of the reachable world got mapped.
    free_agreement   share of the actual free cells that are not inside a reference
                obstacle (more than d from its edge). Low: free space painted
                through walls.
    obstacle_error_mean / _p95   distance from actual occupied cells to the nearest
                reference obstacle, m, over the cells inside the reference's known
                area. The geometric accuracy of what was mapped.

Actual cells that land on reference *unknown* cells (outside the arena, say) are
counted in outside_reference_cells and left out of every ratio. With region
(world x_min, y_min, x_max, y_max), reference cells outside it count as unknown:
the score covers only the area a route is meant to map.
"""

import math
import os

import numpy as np
import yaml
from scipy import ndimage

UNKNOWN, FREE, OCCUPIED = -1, 0, 100


# -- loading and saving --------------------------------------------------------

def load_map_yaml(path):
    """A Nav2 map_server map (yaml + pgm/png), trinary mode."""
    from PIL import Image
    with open(path) as f:
        meta = yaml.safe_load(f)
    image = meta["image"]
    if not os.path.isabs(image):
        image = os.path.join(os.path.dirname(path), image)
    pixels = np.asarray(Image.open(image).convert("L"), dtype=np.float64)
    p = pixels / 255.0 if meta.get("negate", 0) else (255.0 - pixels) / 255.0
    data = np.full(pixels.shape, UNKNOWN, dtype=np.int8)
    data[p > meta.get("occupied_thresh", 0.65)] = OCCUPIED
    data[p < meta.get("free_thresh", 0.196)] = FREE
    origin = meta.get("origin", [0.0, 0.0, 0.0])
    # Image row 0 is the top (max y); grids keep row 0 at the origin.
    return {"data": np.flipud(data), "resolution": float(meta["resolution"]),
            "origin": (float(origin[0]), float(origin[1]), float(origin[2]))}


def save_map_yaml(grid, stem):
    """Write grid as stem.pgm + stem.yaml (map_server format). Returns the yaml path."""
    from PIL import Image
    pixels = np.full(grid["data"].shape, 205, dtype=np.uint8)
    pixels[grid["data"] == FREE] = 254
    pixels[grid["data"] == OCCUPIED] = 0
    Image.fromarray(np.flipud(pixels)).save(stem + ".pgm")
    meta = {"image": os.path.basename(stem) + ".pgm", "mode": "trinary",
            "resolution": grid["resolution"], "origin": [float(v) for v in grid["origin"]],
            "negate": 0, "occupied_thresh": 0.65,
            # map_server's standard thresholds: 205 (unknown) is p = 0.196, which
            # must not read back as free.
            "free_thresh": 0.196}
    with open(stem + ".yaml", "w") as f:
        yaml.safe_dump(meta, f, sort_keys=False)
    return stem + ".yaml"


def grid_from_msg(msg):
    info = msg.info
    data = np.asarray(msg.data, dtype=np.int16).reshape(info.height, info.width)
    out = np.full(data.shape, UNKNOWN, dtype=np.int8)
    out[(data >= 0) & (data < 50)] = FREE
    out[data >= 50] = OCCUPIED
    q = info.origin.orientation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return {"data": out, "resolution": float(info.resolution),
            "origin": (info.origin.position.x, info.origin.position.y, yaw)}


def save_png(rgb, path, scale=None):
    from PIL import Image
    image = Image.fromarray(np.flipud(rgb))
    if scale is None:   # aim for roughly 700 px on the long side
        scale = max(1, int(700 / max(rgb.shape[:2])))
    if scale > 1:
        image = image.resize((image.width * scale, image.height * scale), Image.NEAREST)
    image.save(path)
    return path


# -- geometry ------------------------------------------------------------------

def cell_centers_world(grid):
    """World (x, y) of every cell centre of a world-frame grid."""
    rows, cols = grid["data"].shape
    r = grid["resolution"]
    ox, oy, oyaw = grid["origin"]
    j, i = np.meshgrid(np.arange(cols), np.arange(rows))
    lx, ly = (j + 0.5) * r, (i + 0.5) * r
    c, s = math.cos(oyaw), math.sin(oyaw)
    return ox + c * lx - s * ly, oy + s * lx + c * ly


def sample(grid, x, y, frame_pose=(0.0, 0.0, 0.0)):
    """Value of grid at world points (x, y); grid lives in a frame at frame_pose."""
    fx, fy, fyaw = frame_pose
    c, s = math.cos(-fyaw), math.sin(-fyaw)
    mx, my = c * (x - fx) - s * (y - fy), s * (x - fx) + c * (y - fy)
    ox, oy, oyaw = grid["origin"]
    c, s = math.cos(-oyaw), math.sin(-oyaw)
    gx, gy = c * (mx - ox) - s * (my - oy), s * (mx - ox) + c * (my - oy)
    j = np.floor(gx / grid["resolution"]).astype(np.int64)
    i = np.floor(gy / grid["resolution"]).astype(np.int64)
    rows, cols = grid["data"].shape
    inside = (i >= 0) & (i < rows) & (j >= 0) & (j < cols)
    out = np.full(x.shape, UNKNOWN, dtype=np.int8)
    out[inside] = grid["data"][i[inside], j[inside]]
    return out


# -- the comparison ------------------------------------------------------------

def compare(actual, reference, spawn, match_distance=0.15, region=None):
    """Metrics plus an RGB overlay (uint8 [rows, cols, 3], row 0 at origin y)."""
    res = reference["resolution"]
    wx, wy = cell_centers_world(reference)
    ref = reference["data"].copy()
    if region is not None:
        x0, y0, x1, y1 = region
        ref[(wx < x0) | (wx > x1) | (wy < y0) | (wy > y1)] = UNKNOWN
        reference = {**reference, "data": ref}
    act = sample(actual, wx, wy, frame_pose=spawn)
    # Resampling only sees the actual map at reference cell centres; count what
    # falls outside the reference's known area separately, at native resolution.
    ax, ay = cell_centers_world({**actual, "origin": _compose(spawn, actual["origin"])})
    act_occ_native = actual["data"] == OCCUPIED
    ref_at_actual = sample(reference, ax, ay)
    outside = int(np.count_nonzero(act_occ_native & (ref_at_actual == UNKNOWN)))

    ref_known = ref != UNKNOWN
    ref_occ = ref == OCCUPIED
    ref_free = ref == FREE
    act_known = act != UNKNOWN
    act_occ = act == OCCUPIED
    act_free = act == FREE

    # Distances in metres to the nearest reference / actual obstacle cell.
    d_ref = ndimage.distance_transform_edt(~ref_occ) * res if ref_occ.any() \
        else np.full(ref.shape, np.inf)
    d_act = ndimage.distance_transform_edt(~act_occ) * res if act_occ.any() \
        else np.full(ref.shape, np.inf)
    d_known = ndimage.distance_transform_edt(~act_known) * res if act_known.any() \
        else np.full(ref.shape, np.inf)
    d_ref_free = ndimage.distance_transform_edt(~ref_free) * res if ref_free.any() \
        else np.full(ref.shape, np.inf)

    scored_occ = act_occ & ref_known
    true_pos = scored_occ & (d_ref <= match_distance)
    false_pos = scored_occ & (d_ref > match_distance)
    outline = ref_occ & ndimage.binary_dilation(ref_free, structure=np.ones((3, 3), bool))
    observed_ref_occ = outline & (d_known <= match_distance)
    found = observed_ref_occ & (d_act <= match_distance)
    missed = observed_ref_occ & ~found
    scored_free = act_free & ref_known
    # Free is wrong only inside an obstacle, past the match distance from its edge.
    free_ok = scored_free & ~(ref_occ & (d_ref_free > match_distance))

    def ratio(a, b):
        nb = int(np.count_nonzero(b))
        return float(np.count_nonzero(a)) / nb if nb else 0.0

    errors = d_ref[scored_occ]
    errors = errors[np.isfinite(errors)]
    area = res * res
    metrics = {
        "precision": ratio(true_pos, scored_occ),
        "recall": ratio(found, observed_ref_occ),
        "coverage": ratio(ref_free & act_known, ref_free),
        "free_agreement": ratio(free_ok, scored_free),
        "obstacle_error_mean": float(errors.mean()) if errors.size else None,
        "obstacle_error_p95": float(np.percentile(errors, 95)) if errors.size else None,
        "match_distance": match_distance,
        "false_obstacle_area_m2": float(np.count_nonzero(false_pos)) * area,
        "missed_obstacle_area_m2": float(np.count_nonzero(missed)) * area,
        "mapped_area_m2": float(np.count_nonzero(act_known & ref_known)) * area,
        "reference_free_area_m2": float(np.count_nonzero(ref_free)) * area,
        "outside_reference_cells": outside,
        "actual_resolution": actual["resolution"],
        "reference_resolution": res,
        "region": list(region) if region is not None else None,
    }

    # Overlay. Reference as the backdrop, the actual map's verdicts on top.
    rgb = np.full(ref.shape + (3,), 255, dtype=np.uint8)          # reference unknown
    rgb[ref_free] = (236, 236, 236)                                # reference free
    rgb[ref_occ] = (150, 150, 150)                                 # reference obstacle
    rgb[scored_free & ~free_ok] = (250, 200, 120)                  # free through an obstacle
    rgb[scored_free & free_ok] = (205, 230, 205)                   # mapped free, agrees
    rgb[missed] = (60, 110, 230)                                   # missed obstacle
    rgb[true_pos] = (30, 150, 60)                                  # obstacle found
    rgb[false_pos] = (220, 40, 40)                                 # phantom obstacle
    metrics["overlay"] = rgb
    metrics["legend"] = {
        "grey": "reference obstacle", "light grey": "reference free",
        "pale green": "mapped free, agrees", "green": "obstacle found",
        "red": "phantom obstacle (not in reference)",
        "blue": "missed obstacle (observed area, nothing mapped)",
        "orange": "mapped free inside a reference obstacle",
    }
    return metrics


def _compose(a, b):
    ax, ay, ayaw = a
    bx, by, byaw = b
    c, s = math.cos(ayaw), math.sin(ayaw)
    return (ax + c * bx - s * by, ay + s * bx + c * by, ayaw + byaw)
