import math

import numpy as np
import pytest
from grid_map_msgs.msg import GridMap
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

from rover_nav2.benchmark import terrain_eval as te


def msg_with(stored, outer=0, inner=0, res=0.5, cx=2.0, cy=-1.0):
    rows, cols = stored.shape
    m = GridMap()
    m.info.resolution = res
    m.info.length_x, m.info.length_y = rows * res, cols * res
    m.info.pose.position.x, m.info.pose.position.y = cx, cy
    arr = Float32MultiArray()
    arr.layout.dim = [MultiArrayDimension(label='column_index', size=cols, stride=rows * cols),
                      MultiArrayDimension(label='row_index', size=rows, stride=rows)]
    arr.data = stored.T.ravel().astype(np.float32).tolist()    # column-major
    m.layers, m.data = ['elevation'], [arr]
    m.outer_start_index, m.inner_start_index = outer, inner
    return m


def test_layer_unwraps_the_circular_buffer():
    unwrapped = np.array([[0, 1, 2, 3], [10, 11, 12, 13], [20, 21, 22, 23]], dtype=float)
    stored = np.empty_like(unwrapped)
    for i in range(3):
        for j in range(4):
            stored[(i + 1) % 3, (j + 2) % 4] = unwrapped[i, j]
    got = te.gridmap_layer(msg_with(stored, outer=1, inner=2), 'elevation')
    assert np.array_equal(got, unwrapped)
    assert te.gridmap_layer(msg_with(stored), 'missing') is None


def test_cell_centres():
    # 3 x 4 cells of 0.5 m centred at (2, -1): row 0 at x 2.5, column 0 at y -0.25.
    m = msg_with(np.zeros((3, 4)))
    X, Y = te.gridmap_cell_xy(m, (3, 4))
    assert X[0, 0] == pytest.approx(2.5) and X[2, 0] == pytest.approx(1.5)
    assert Y[0, 0] == pytest.approx(-0.25) and Y[0, 3] == pytest.approx(-1.75)


def test_ground_truth_sampling(tmp_path):
    # 2 x 2 cells of 1 m from x 0 (left) and y 2 (top): centres at (0.5, 1.5), (1.5, 1.5)...
    np.savez(tmp_path / 'g.npz', z=np.array([[0.0, 1.0], [2.0, 3.0]]), x_min=0.0, y_max=2.0,
             res=1.0)
    g = te.GroundTruthGrid(str(tmp_path / 'g.npz'))
    z = g.sample(np.array([0.5, 1.5, 1.0, 1.0, 5.0]), np.array([1.5, 1.5, 1.0, 1.5, 1.0]))
    assert z[:4] == pytest.approx([0.0, 1.0, 1.5, 0.5])
    assert math.isnan(z[4])


def test_footprint_reaches():
    # 1.1 x 0.9 m footprint at the origin heading +x: front edge at x 0.55.
    assert te.footprint_reaches(0, 0, 0.0, 1.1, 0.9, 0.8, 0.0, 0.3)       # 0.25 m past the edge
    assert not te.footprint_reaches(0, 0, 0.0, 1.1, 0.9, 1.0, 0.0, 0.3)   # 0.45 m out
    # Heading +y: the long side now faces y, so the same disc at (0, 0.8) is reached.
    assert te.footprint_reaches(0, 0, math.pi / 2, 1.1, 0.9, 0.0, 0.8, 0.3)
    # Corner: (0.55, 0.45) to (0.75, 0.65) is 0.283 m.
    assert te.footprint_reaches(0, 0, 0.0, 1.1, 0.9, 0.75, 0.65, 0.29)
    assert not te.footprint_reaches(0, 0, 0.0, 1.1, 0.9, 0.75, 0.65, 0.27)
