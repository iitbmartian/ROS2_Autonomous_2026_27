import math

import numpy as np
import pytest
from PIL import Image

from rover_nav2.benchmark import ground_truth as gt


def P(x=0, y=0, z=0, r=0, p=0, yw=0):
    return gt.Pose(gt.rpy_matrix(r, p, yw), [x, y, z])


def pts(*xy):
    return np.array(xy, dtype=float)


def test_box_top_and_miss():
    z = gt.cast_box(P(z=0.1), [1.0, 1.0, 0.2], pts((0.0, 0.0), (0.49, -0.49), (0.6, 0.0)))
    assert z[0] == pytest.approx(0.2) and z[1] == pytest.approx(0.2)
    assert math.isnan(z[2])


def test_ledge_ramp_box_from_ledge_sdf():
    # ledge.sdf: <pose>2.01 0.218 -0.1497 0 -0.05 0</pose>, size 2.0 0.5 0.4. Its comment says
    # the top face runs from the ground at x 1.0 up to 10 cm at x 3.0. Top face centre:
    # c + 0.2 * (sin(-0.05), 0, cos(-0.05)) = (2.0000, 0.218, 0.04995), slope tan(0.05).
    pose = P(2.01, 0.218, -0.1497, 0, -0.05, 0)
    # The top face ends at x = 2.000 - cos(0.05) = 1.0012, so x 1.0 itself is a miss.
    z = gt.cast_box(pose, [2.0, 0.5, 0.4], pts((2.0, 0.218), (1.01, 0.218), (2.9, 0.218)))
    x0, z0 = 2.01 - 0.2 * math.sin(0.05), -0.1497 + 0.2 * math.cos(0.05)
    expect = [z0 + math.tan(0.05) * (x - x0) for x in (2.0, 1.01, 2.9)]
    assert z == pytest.approx(expect, abs=1e-9)
    assert z[1] == pytest.approx(0.0, abs=1e-3) and z[2] == pytest.approx(0.0955, abs=1e-3)
    assert math.isnan(gt.cast_box(pose, [2.0, 0.5, 0.4], pts((1.0, 0.218)))[0])


def test_ellipsoid():
    # radii (0.2, 0.1, 0.05) at (1, 1, 0): top 0.05, at dx 0.1 it is 0.05 sqrt(1 - 0.25).
    z = gt.cast_ellipsoid(P(1, 1, 0), [0.2, 0.1, 0.05], pts((1, 1), (1.1, 1), (1.3, 1)))
    assert z[0] == pytest.approx(0.05)
    assert z[1] == pytest.approx(0.05 * math.sqrt(0.75))
    assert math.isnan(z[2])


def test_vertical_cylinder():
    z = gt.cast_cylinder(P(z=0.15), 0.1, 0.3, pts((0.05, 0.05), (0.2, 0.0)))
    assert z[0] == pytest.approx(0.3)
    assert math.isnan(z[1])


def test_horizontal_cylinder():
    # Roll 90 deg lays the axis along y. Radius 0.1 centred at z 0.1: at dx 0.05 the top is
    # 0.1 + sqrt(0.01 - 0.0025).
    z = gt.cast_cylinder(P(z=0.1, r=math.pi / 2), 0.1, 1.0, pts((0.05, 0.0), (0.05, 0.6)))
    assert z[0] == pytest.approx(0.1 + math.sqrt(0.0075))
    assert math.isnan(z[1])


def test_plane():
    z = gt.cast_plane(P(z=-0.5), [0, 0, 1], pts((3, 4), (-100, 7)))
    assert z == pytest.approx([-0.5, -0.5])


def write_png(path, values):
    Image.fromarray(np.array(values, dtype=np.uint16), mode='I;16').save(path)


def test_heightmap_bilinear(tmp_path):
    # 3 x 3, centre 65535 and edges 0, over 2 x 2 m with height range 1 m centred at origin.
    write_png(tmp_path / 'h.png', [[0, 0, 0], [0, 65535, 0], [0, 0, 0]])
    img = gt.load_heightmap_image(str(tmp_path / 'h.png'))
    z = gt.cast_heightmap(P(), [2.0, 2.0, 1.0], [0.0, 0.0, 0.0], img,
                          pts((0, 0), (0.5, 0), (1.0, 1.0), (1.5, 0)))
    assert z[0] == pytest.approx(1.0)
    assert z[1] == pytest.approx(0.5)
    assert z[2] == pytest.approx(0.0)
    assert math.isnan(z[3])


def test_heightmap_row_zero_is_plus_y(tmp_path):
    write_png(tmp_path / 'h.png', [[65535, 65535], [0, 0]])
    img = gt.load_heightmap_image(str(tmp_path / 'h.png'))
    z = gt.cast_heightmap(P(), [1.0, 1.0, 2.0], [0.0, 0.0, -1.0], img, pts((0, 0.5), (0, -0.5)))
    assert z == pytest.approx([1.0, -1.0])


WORLD = """<?xml version="1.0"?>
<sdf version="1.8"><world name="t">
  <model name="ground"><static>true</static><link name="l"><collision name="c">
    <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
  </collision></link></model>
  <model name="block"><static>true</static><pose>1 0 0 0 0 1.5707963267948966</pose>
    <link name="l"><pose>1 0 0 0 0 0</pose><collision name="c">
      <pose>0 0 0.25 0 0 0</pose>
      <geometry><box><size>0.4 0.2 0.5</size></box></geometry>
    </collision></link></model>
  <model name="hm"><static>true</static><link name="l"><collision name="c"><geometry>
    <heightmap><uri>h.png</uri><size>2 2 0.4</size><pos>-5 -5 -0.2</pos></heightmap>
  </geometry></collision></link></model>
</world></sdf>
"""


def test_world_nested_poses_and_max(tmp_path):
    write_png(tmp_path / 'h.png', [[0, 0, 0], [0, 65535, 0], [0, 0, 0]])
    (tmp_path / 'w.sdf').write_text(WORLD)
    shapes = gt.load_shapes(str(tmp_path / 'w.sdf'))
    assert sorted(s.kind for s in shapes) == ['box', 'heightmap', 'plane']
    # Model yaw 90 deg moves the link's +x offset to +y: the block sits at (1, 1), its 0.4 m
    # side now along y. Top at 0.5. Ground elsewhere at 0. Heightmap peak 0.2 at (-5, -5).
    z = gt.ground_truth(shapes, -6.0, 3.0, -6.0, 3.0, 0.05)
    def at(x, y):
        return z[int((3.0 - y) / 0.05), int((x + 6.0) / 0.05)]
    assert at(1.0, 1.0) == pytest.approx(0.5)
    assert at(1.0, 1.17) == pytest.approx(0.5)       # inside the 0.4 m side, now along y
    assert at(1.17, 1.0) == pytest.approx(0.0)       # outside the 0.2 m side, now along x
    assert at(2.5, -2.0) == pytest.approx(0.0)
    assert at(-5.0 + 0.025, -5.0 - 0.025) == pytest.approx(0.2 - 0.4 * 0.025, abs=0.02)
    assert not np.isnan(z).any()


def write_stl(path, tris, ascii_=False):
    tris = np.asarray(tris, dtype=float)
    if ascii_:
        lines = ['solid t']
        for t in tris:
            lines += ['facet normal 0 0 1', 'outer loop']
            lines += [f'vertex {v[0]} {v[1]} {v[2]}' for v in t]
            lines += ['endloop', 'endfacet']
        path.write_text('\n'.join(lines + ['endsolid t']) + '\n')
        return
    rec = np.zeros(len(tris), dtype=[('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')])
    rec['v'] = tris
    path.write_bytes(b'\0' * 80 + np.uint32(len(tris)).tobytes() + rec.tobytes())


def plane_square(n=10):
    """Unit square split into n x n cells of two triangles each, on z = 0.1 x + 0.2 y."""
    out = []
    for i in range(n):
        for j in range(n):
            x0, y0, d = i / n, j / n, 1 / n
            c = [(x0, y0), (x0 + d, y0), (x0 + d, y0 + d), (x0, y0 + d)]
            v = [(x, y, 0.1 * x + 0.2 * y) for x, y in c]
            out += [[v[0], v[1], v[2]], [v[0], v[2], v[3]]]
    return out


MESH_WORLD = """<?xml version="1.0"?>
<sdf version="1.9"><world name="w">
<model name="m"><static>true</static><pose>5 0 1 0 0 1.5707963267948966</pose>
<link name="l"><collision name="c"><geometry><mesh><uri>t.stl</uri><scale>2 2 2</scale></mesh>
</geometry></collision></link></model>
</world></sdf>
"""


@pytest.mark.parametrize('ascii_', [False, True])
def test_mesh_scaled_and_posed(tmp_path, ascii_):
    write_stl(tmp_path / 't.stl', plane_square(), ascii_)
    (tmp_path / 'w.sdf').write_text(MESH_WORLD)
    shapes = gt.load_shapes(str(tmp_path / 'w.sdf'))
    assert [s.kind for s in shapes] == ['mesh']
    # Scale 2 makes the square [0, 2]^2 on z = 0.1 x + 0.2 y. Yaw 90 deg maps mesh (x, y) to
    # world (5 - y, x), and z rises by 1. World (4.5, 1.0) is mesh (1.0, 0.5): 1 + 0.1 + 0.1.
    # World (3.2, 0.4) is mesh (0.4, 1.8): 1 + 0.04 + 0.36. World (5.5, 1) is mesh y -0.5, a miss.
    z = shapes[0].cast(pts((4.5, 1.0), (3.2, 0.4), (5.5, 1.0), (4.0, 1.234)))
    assert z[0] == pytest.approx(1.2, abs=1e-6)
    assert z[1] == pytest.approx(1.4, abs=1e-6)
    assert math.isnan(z[2])
    # Mesh (1.234, 1.0), on a triangle edge in neither bucket corner: 1 + 0.1234 + 0.2.
    assert z[3] == pytest.approx(1.3234, abs=1e-6)
    lo, hi = shapes[0].xy_bounds()
    assert lo == pytest.approx([3.0, 0.0]) and hi == pytest.approx([5.0, 2.0])


def test_mesh_large_and_overlapping_triangles_take_top_hit():
    # One big triangle at z 0 over (0,0), (10,0), (0,10) and a small one at z 0.5 inside it.
    # The big one spans many buckets, the small one sets the bucket size. A vertical triangle
    # has no xy area and is dropped.
    big = [(0, 0, 0), (10, 0, 0), (0, 10, 0)]
    small = [(1, 1, 0.5), (1.5, 1, 0.5), (1, 1.5, 0.5)]
    vertical = [(2, 2, 0), (3, 2, 0), (3, 2, 1)]
    c = gt.MeshCaster(np.array([big, small, small, vertical], dtype=float))
    z = c.cast(pts((1.1, 1.1), (6.0, 1.0), (4.9, 4.9), (5.1, 5.1), (2.5, 2.0)))
    assert z[0] == pytest.approx(0.5)
    assert z[1] == pytest.approx(0.0)
    assert z[2] == pytest.approx(0.0)
    assert math.isnan(z[3])     # beyond the hypotenuse x + y = 10
    assert z[4] == pytest.approx(0.0)
