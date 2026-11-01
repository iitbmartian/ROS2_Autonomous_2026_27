import math

import numpy as np
import pytest

from rover_nav2.benchmark.coverage_check import edge_distance, nearest_per_bearing


def test_edge_distance():
    # 1.1 x 0.9 footprint: half 0.55 x 0.45.
    d = edge_distance(np.array([0.0, 1.55, 0.0, 0.85, 0.3]), np.array([0.0, 0.0, 1.45, 0.75, 0.1]),
                      0.55, 0.45)
    assert d == pytest.approx([0.0, 1.0, 1.0, math.hypot(0.3, 0.3), 0.0])


def test_nearest_per_bearing():
    # Observed cells straight ahead 0.6 m past the edge and straight behind 1.5 m past it.
    lx = np.array([1.15, -2.05, 3.0])
    ly = np.array([0.0, 0.0, 0.0])
    obs = np.array([True, True, False])
    per = nearest_per_bearing(lx, ly, obs, 0.55, 0.45, 4)
    # Bins of 90 deg starting at 0: ahead lands in bin 0, behind (180 deg) in bin 2.
    assert per[0] == pytest.approx(0.6)
    assert per[2] == pytest.approx(1.5)
    assert math.isinf(per[1]) and math.isinf(per[3])
