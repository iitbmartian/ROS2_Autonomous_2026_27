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
