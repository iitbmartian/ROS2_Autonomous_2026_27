"""Ground-truth terrain height for a Gazebo world: the top surface on a regular grid.

    ros2 run rover_nav2 ground_truth_heightmap marsyard.sdf --out ~/gt/marsyard.npz

Reads the world SDF itself, so the result is exactly the terrain Gazebo simulates. For every
grid cell a vertical ray is cast from above against every static collision shape, analytically:
heightmap, mesh (STL), box, ellipsoid, sphere, cylinder and plane. Other mesh formats are skipped
with a warning. The
height of a cell is the highest hit, NaN where nothing is hit.

Output .npz: z (rows, cols) in metres, row 0 at y_max and column 0 at x_min (image order), plus
x_min, y_max, res. Heights are sampled at cell centres.
"""

import argparse
import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np


def rpy_matrix(roll, pitch, yaw):
    """SDF rotation: fixed-axis roll about x, then pitch about y, then yaw about z."""
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                              math.sin(pitch), math.cos(yaw), math.sin(yaw))
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]])


class Pose:

    def __init__(self, R=None, t=None):
        self.R = np.eye(3) if R is None else R
        self.t = np.zeros(3) if t is None else np.asarray(t, dtype=float)

    @staticmethod
    def parse(elem):
        if elem is None or not (elem.text or '').strip():
            return Pose()
        v = [float(x) for x in elem.text.split()]
        v += [0.0] * (6 - len(v))
        return Pose(rpy_matrix(v[3], v[4], v[5]), v[:3])

    def __mul__(self, other):
        return Pose(self.R @ other.R, self.R @ other.t + self.t)


# --- Ray casts. Each takes cell centres (N, 2) and returns the top hit z (N,), NaN on a miss.
# Rays start above everything and point down (0, 0, -1).

Z_TOP = 1.0e4


def _local_rays(pose, xy):
    """Ray origins and the common direction in the shape's frame."""
    o = np.column_stack([xy, np.full(len(xy), Z_TOP)]) - pose.t
    return o @ pose.R, pose.R.T @ np.array([0.0, 0.0, -1.0])


def cast_box(pose, size, xy):
    o, d = _local_rays(pose, xy)
    half = np.asarray(size) / 2.0
    t_in = np.full(len(xy), -np.inf)
    t_out = np.full(len(xy), np.inf)
    for k in range(3):
        if abs(d[k]) < 1e-12:
            inside = np.abs(o[:, k]) <= half[k]
            t_in = np.where(inside, t_in, np.inf)
            continue
        t1 = (-half[k] - o[:, k]) / d[k]
        t2 = (half[k] - o[:, k]) / d[k]
        t_in = np.maximum(t_in, np.minimum(t1, t2))
        t_out = np.minimum(t_out, np.maximum(t1, t2))
    hit = (t_in <= t_out) & np.isfinite(t_in)
    return np.where(hit, Z_TOP - t_in, np.nan)


def cast_ellipsoid(pose, radii, xy):
    o, d = _local_rays(pose, xy)
    r = np.asarray(radii, dtype=float)
    o, d = o / r, d / r
    a = d @ d
    b = 2.0 * (o @ d)
    c = np.einsum('ij,ij->i', o, o) - 1.0
    disc = b * b - 4 * a * c
    t = (-b - np.sqrt(np.maximum(disc, 0.0))) / (2 * a)
    return np.where(disc >= 0, Z_TOP - t, np.nan)


def cast_cylinder(pose, radius, length, xy):
    o, d = _local_rays(pose, xy)
    n = len(xy)
    # Side: x^2 + y^2 = r^2.
    a = d[0] ** 2 + d[1] ** 2
    if a < 1e-12:      # axis vertical in practice: inside the circle hits the top cap
        inside = o[:, 0] ** 2 + o[:, 1] ** 2 <= radius ** 2
        t_side_in = np.where(inside, -np.inf, np.inf)
        t_side_out = np.where(inside, np.inf, -np.inf)
    else:
        b = 2 * (o[:, 0] * d[0] + o[:, 1] * d[1])
        c = o[:, 0] ** 2 + o[:, 1] ** 2 - radius ** 2
        disc = b * b - 4 * a * c
        s = np.sqrt(np.maximum(disc, 0.0))
        t_side_in = np.where(disc >= 0, (-b - s) / (2 * a), np.inf)
        t_side_out = np.where(disc >= 0, (-b + s) / (2 * a), -np.inf)
    # Caps: |z| <= length / 2.
    if abs(d[2]) < 1e-12:
        inside = np.abs(o[:, 2]) <= length / 2
        t_cap_in = np.where(inside, -np.inf, np.inf)
        t_cap_out = np.where(inside, np.inf, -np.inf)
    else:
        t1 = (-length / 2 - o[:, 2]) / d[2]
        t2 = (length / 2 - o[:, 2]) / d[2]
        t_cap_in, t_cap_out = np.minimum(t1, t2), np.maximum(t1, t2)
    t_in = np.maximum(t_side_in, t_cap_in)
    t_out = np.minimum(t_side_out, t_cap_out)
    hit = (t_in <= t_out) & np.isfinite(t_in) & np.ones(n, bool)
    return np.where(hit, Z_TOP - t_in, np.nan)


def cast_plane(pose, normal, xy):
    n = pose.R @ np.asarray(normal, dtype=float)
    if abs(n[2]) < 1e-9:
        return np.full(len(xy), np.nan)
    # n . (p - t) = 0 on the vertical line through (x, y).
    z = pose.t[2] - (n[0] * (xy[:, 0] - pose.t[0]) + n[1] * (xy[:, 1] - pose.t[1])) / n[2]
    return z


def load_heightmap_image(path):
    from PIL import Image
    img = Image.open(path)
    a = np.array(img, dtype=float)
    if a.ndim == 3:
        a = a[..., 0]
    full = 65535.0 if (img.mode in ('I;16', 'I') or a.max() > 255) else 255.0
    return a / full


def cast_heightmap(pose, size, pos, image, xy):
    """Gazebo heightmap: an N x N image over size[0] x size[1] centred on pos, height
    pos.z + value * size[2]. Row 0 is the +y edge, column 0 the -x edge. Bilinear between
    pixel centres, which sit on a regular grid including the edges."""
    n_rows, n_cols = image.shape
    # Shape frame: the heightmap is defined in its link frame, usually the identity.
    local = (np.column_stack([xy, np.zeros(len(xy))]) - pose.t) @ pose.R
    u = (local[:, 0] - (pos[0] - size[0] / 2)) / size[0] * (n_cols - 1)
    v = ((pos[1] + size[1] / 2) - local[:, 1]) / size[1] * (n_rows - 1)
    inside = (u >= 0) & (u <= n_cols - 1) & (v >= 0) & (v <= n_rows - 1)
    u = np.clip(u, 0, n_cols - 1)
    v = np.clip(v, 0, n_rows - 1)
    j0 = np.minimum(np.floor(u).astype(int), n_cols - 2)
    i0 = np.minimum(np.floor(v).astype(int), n_rows - 2)
    fu, fv = u - j0, v - i0
    h = (image[i0, j0] * (1 - fu) * (1 - fv) + image[i0, j0 + 1] * fu * (1 - fv)
         + image[i0 + 1, j0] * (1 - fu) * fv + image[i0 + 1, j0 + 1] * fu * fv)
    z = pos[2] + h * size[2] + pose.t[2]
    return np.where(inside, z, np.nan)


def load_stl(path):
    """Triangles (N, 3, 3) from a binary or ASCII STL."""
    with open(path, 'rb') as f:
        data = f.read()
    if len(data) >= 84:
        n = int(np.frombuffer(data[80:84], dtype='<u4')[0])
        if len(data) == 84 + 50 * n:
            rec = np.dtype([('normal', '<f4', 3), ('v', '<f4', (3, 3)), ('attr', '<u2')])
            return np.frombuffer(data, dtype=rec, count=n, offset=84)['v'].astype(float)
    verts = [[float(x) for x in line.split()[1:4]]
             for line in data.decode('ascii', 'replace').splitlines()
             if line.strip().startswith('vertex')]
    return np.array(verts, dtype=float).reshape(-1, 3, 3)


class MeshCaster:
    """Top hit of vertical rays on a triangle soup. Triangles are binned into square buckets
    in xy, so each ray only tests the few triangles over its bucket. Vertical triangles have no
    xy area and are never hit by a vertical ray, so they are dropped."""

    def __init__(self, tris):
        a, b, c = tris[:, 0], tris[:, 1], tris[:, 2]
        area = (b[:, 0] - a[:, 0]) * (c[:, 1] - a[:, 1]) - (b[:, 1] - a[:, 1]) * (c[:, 0] - a[:, 0])
        keep = np.abs(area) > 1e-12
        self.tris, self.area = tris[keep], area[keep]
        lo, hi = self.tris[:, :, :2].min(axis=1), self.tris[:, :, :2].max(axis=1)
        self.lo = lo.min(axis=0) if len(lo) else np.zeros(2)
        self.hi = hi.max(axis=0) if len(hi) else np.zeros(2)
        ext = np.median(hi - lo, axis=0).max() if len(lo) else 1.0
        self.bucket = max(float(ext), 1e-6)
        i0 = np.floor((lo - self.lo) / self.bucket).astype(int)
        i1 = np.floor((hi - self.lo) / self.bucket).astype(int)
        self.nx = int(np.floor((self.hi[0] - self.lo[0]) / self.bucket)) + 1
        span = i1 - i0 + 1
        bids, tids = [], []
        small = (span <= 4).all(axis=1)
        for dx in range(4):
            for dy in range(4):
                m = small & (dx < span[:, 0]) & (dy < span[:, 1])
                bids.append((i0[m, 1] + dy) * self.nx + i0[m, 0] + dx)
                tids.append(np.nonzero(m)[0])
        for t in np.nonzero(~small)[0]:
            gx, gy = np.meshgrid(np.arange(i0[t, 0], i1[t, 0] + 1), np.arange(i0[t, 1], i1[t, 1] + 1))
            bids.append((gy * self.nx + gx).ravel())
            tids.append(np.full(gx.size, t))
        bids, tids = np.concatenate(bids), np.concatenate(tids)
        order = np.argsort(bids, kind='stable')
        self.bids, self.tids = bids[order], tids[order]

    def bounds(self):
        return self.lo, self.hi

    def cast(self, xy):
        z = np.full(len(xy), np.nan)
        g = np.floor((xy - self.lo) / self.bucket).astype(int)
        ok = (xy >= self.lo).all(axis=1) & (xy <= self.hi).all(axis=1)
        q = np.where(ok, g[:, 1] * self.nx + g[:, 0], -1)
        start = np.searchsorted(self.bids, q, 'left')
        end = np.where(ok, np.searchsorted(self.bids, q, 'right'), start)
        count = end - start
        for k in range(int(count.max()) if len(count) else 0):
            m = np.nonzero(count > k)[0]
            t = self.tids[start[m] + k]
            a, b, c = self.tris[t, 0], self.tris[t, 1], self.tris[t, 2]
            p = xy[m]
            # Barycentric weights from signed sub-areas, normalised by the triangle's area.
            wa = ((b[:, 0] - p[:, 0]) * (c[:, 1] - p[:, 1])
                  - (b[:, 1] - p[:, 1]) * (c[:, 0] - p[:, 0])) / self.area[t]
            wb = ((c[:, 0] - p[:, 0]) * (a[:, 1] - p[:, 1])
                  - (c[:, 1] - p[:, 1]) * (a[:, 0] - p[:, 0])) / self.area[t]
            wc = 1.0 - wa - wb
            inside = (wa >= -1e-9) & (wb >= -1e-9) & (wc >= -1e-9)
            h = wa * a[:, 2] + wb * b[:, 2] + wc * c[:, 2]
            z[m] = np.fmax(z[m], np.where(inside, h, np.nan))
        return z


def mesh_world_triangles(tris, pose, scale):
    """Mesh vertices in the world frame: scaled in the mesh frame, then posed."""
    v = tris.reshape(-1, 3) * np.asarray(scale, dtype=float)
    return (v @ pose.R.T + pose.t).reshape(-1, 3, 3)


# --- SDF -----------------------------------------------------------------------------------

def resolve_uri(uri, world_dir):
    if uri.startswith('file://'):
        return uri[len('file://'):]
    if uri.startswith('model://'):
        rel = uri[len('model://'):]
        for root in os.environ.get('GZ_SIM_RESOURCE_PATH', '').split(':'):
            if root and os.path.exists(os.path.join(root, rel)):
                return os.path.join(root, rel)
        raise FileNotFoundError(f'{uri} not found on GZ_SIM_RESOURCE_PATH')
    return uri if os.path.isabs(uri) else os.path.join(world_dir, uri)


def floats(elem, default=None):
    if elem is None:
        return default
    return [float(x) for x in elem.text.split()]


class Shape:

    def __init__(self, kind, pose, params, name):
        self.kind, self.pose, self.params, self.name = kind, pose, params, name

    def cast(self, xy):
        p = self.params
        if self.kind == 'box':
            return cast_box(self.pose, p['size'], xy)
        if self.kind == 'ellipsoid':
            return cast_ellipsoid(self.pose, p['radii'], xy)
        if self.kind == 'sphere':
            return cast_ellipsoid(self.pose, [p['radius']] * 3, xy)
        if self.kind == 'cylinder':
            return cast_cylinder(self.pose, p['radius'], p['length'], xy)
        if self.kind == 'plane':
            return cast_plane(self.pose, p['normal'], xy)
        if self.kind == 'heightmap':
            return cast_heightmap(self.pose, p['size'], p['pos'], p['image'], xy)
        if self.kind == 'mesh':
            return p['caster'].cast(xy)
        raise ValueError(self.kind)

    def xy_bounds(self):
        """Planar bounding box, or None for unbounded shapes."""
        p = self.params
        if self.kind == 'plane':
            return None
        if self.kind == 'heightmap':
            c = self.pose.t[:2] + np.asarray(p['pos'][:2])
            h = np.asarray(p['size'][:2]) / 2
            return c - h, c + h
        if self.kind == 'mesh':
            return p['caster'].bounds()
        if self.kind == 'box':
            r = np.linalg.norm(p['size']) / 2
        elif self.kind == 'ellipsoid':
            r = max(p['radii'])
        elif self.kind == 'sphere':
            r = p['radius']
        else:
            r = math.hypot(p['radius'], p['length'] / 2)
        return self.pose.t[:2] - r, self.pose.t[:2] + r


def load_shapes(world_path, warn=print):
    world_dir = os.path.dirname(os.path.abspath(world_path))
    root = ET.parse(world_path).getroot()
    world = root.find('world')
    shapes = []
    for model in world.findall('model'):
        mpose = Pose.parse(model.find('pose'))
        for link in model.findall('link'):
            lpose = mpose * Pose.parse(link.find('pose'))
            for col in link.findall('collision'):
                cpose = lpose * Pose.parse(col.find('pose'))
                geom = col.find('geometry')
                if geom is None or len(geom) == 0:
                    continue
                g = geom[0]
                name = f"{model.get('name')}/{col.get('name')}"
                if g.tag == 'box':
                    shapes.append(Shape('box', cpose, {'size': floats(g.find('size'))}, name))
                elif g.tag == 'ellipsoid':
                    shapes.append(Shape('ellipsoid', cpose,
                                        {'radii': floats(g.find('radii'))}, name))
                elif g.tag == 'sphere':
                    shapes.append(Shape('sphere', cpose,
                                        {'radius': floats(g.find('radius'))[0]}, name))
                elif g.tag == 'cylinder':
                    shapes.append(Shape('cylinder', cpose, {
                        'radius': floats(g.find('radius'))[0],
                        'length': floats(g.find('length'))[0]}, name))
                elif g.tag == 'plane':
                    shapes.append(Shape('plane', cpose,
                                        {'normal': floats(g.find('normal'), [0, 0, 1])}, name))
                elif g.tag == 'heightmap':
                    img = load_heightmap_image(resolve_uri(g.find('uri').text.strip(), world_dir))
                    shapes.append(Shape('heightmap', cpose, {
                        'size': floats(g.find('size'), [img.shape[1], img.shape[0], 1]),
                        'pos': floats(g.find('pos'), [0, 0, 0]),
                        'image': img}, name))
                elif g.tag == 'mesh' and g.find('uri').text.strip().lower().endswith('.stl'):
                    tris = load_stl(resolve_uri(g.find('uri').text.strip(), world_dir))
                    tris = mesh_world_triangles(tris, cpose, floats(g.find('scale'), [1, 1, 1]))
                    shapes.append(Shape('mesh', cpose, {'caster': MeshCaster(tris)}, name))
                else:
                    warn(f'ground_truth: skipping {g.tag} collision {name}')
    return shapes


def ground_truth(shapes, x_min, x_max, y_min, y_max, res):
    """Top-surface height over the grid, cell centres, image order (row 0 at y_max)."""
    cols = int(round((x_max - x_min) / res))
    rows = int(round((y_max - y_min) / res))
    xs = x_min + (np.arange(cols) + 0.5) * res
    ys = y_max - (np.arange(rows) + 0.5) * res
    X, Y = np.meshgrid(xs, ys)
    xy = np.column_stack([X.ravel(), Y.ravel()])
    z = np.full(len(xy), np.nan)
    for s in shapes:
        b = s.xy_bounds()
        if b is None:
            idx = np.arange(len(xy))
        else:
            lo, hi = b
            idx = np.nonzero((xy[:, 0] >= lo[0]) & (xy[:, 0] <= hi[0])
                             & (xy[:, 1] >= lo[1]) & (xy[:, 1] <= hi[1]))[0]
        if len(idx) == 0:
            continue
        h = s.cast(xy[idx])
        z[idx] = np.fmax(z[idx], h)
    return z.reshape(rows, cols)


def default_extent(shapes, margin=1.0, around_origin=10.0):
    """Bounds of every bounded shape plus a margin, and always at least +-around_origin about
    the spawn point, so worlds of small obstacles on a plane still cover the drive area."""
    los, his = [np.array([-around_origin, -around_origin])], [np.array([around_origin] * 2)]
    for s in shapes:
        b = s.xy_bounds()
        if b is not None:
            los.append(b[0])
            his.append(b[1])
    lo, hi = np.min(los, axis=0) - margin, np.max(his, axis=0) + margin
    return lo[0], hi[0], lo[1], hi[1]


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('world', help='world file, or a name in rover_gazebo/worlds')
    p.add_argument('--out', required=True, help='.npz path')
    p.add_argument('--res', type=float, default=0.05)
    p.add_argument('--extent', type=float, nargs=4, metavar=('XMIN', 'XMAX', 'YMIN', 'YMAX'))
    p.add_argument('--png', action='store_true', help='also write a preview next to --out')
    a = p.parse_args(sys.argv[1:] if argv is None else argv)
    world = a.world
    if not os.path.exists(world):
        from ament_index_python.packages import get_package_share_directory
        world = os.path.join(get_package_share_directory('rover_gazebo'), 'worlds', world)
    shapes = load_shapes(world)
    ext = a.extent or default_extent(shapes)
    z = ground_truth(shapes, *ext, a.res)
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    np.savez_compressed(a.out, z=z.astype(np.float32), x_min=ext[0], y_max=ext[3], res=a.res,
                        world=os.path.basename(world))
    fin = z[np.isfinite(z)]
    print(f'{os.path.basename(world)}: {len(shapes)} shapes, grid {z.shape[1]} x {z.shape[0]} '
          f'at {a.res} m over x {ext[0]:.2f}..{ext[1]:.2f}, y {ext[2]:.2f}..{ext[3]:.2f}, '
          f'z {fin.min():.3f}..{fin.max():.3f} m -> {a.out}')
    if a.png:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(8, 8 * z.shape[0] / z.shape[1]))
        im = ax.imshow(z, cmap='terrain', extent=(ext[0], ext[1], ext[2], ext[3]))
        fig.colorbar(im, ax=ax, label='z [m]')
        ax.set_title(f'ground truth {os.path.basename(world)}')
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        fig.tight_layout()
        fig.savefig(os.path.splitext(a.out)[0] + '.png', dpi=90)
    return 0


if __name__ == '__main__':
    sys.exit(main())
