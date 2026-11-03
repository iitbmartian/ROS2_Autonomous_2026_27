import math

import pytest

from rover_nav2.benchmark import metrics as m


def line(n, dx=0.1, t0=0.0, dt=0.1, roll=0.0, pitch=0.0):
    return [(t0 + i * dt, i * dx, 0.0, 0.0, roll, pitch, 0.0) for i in range(n)]


def test_straight_line_length():
    # 101 samples 0.1 m apart: 10 m.
    assert m.path_length(line(101)) == pytest.approx(10.0)


def test_triangle_3_4_5():
    pts = [(0, 0, 0, 0, 0, 0, 0), (1, 3, 0, 0, 0, 0, 0), (2, 3, 4, 0, 0, 0, 0),
           (3, 0, 0, 0, 0, 0, 0)]
    assert m.path_length(pts) == pytest.approx(12.0)


def test_vibration_is_not_travel():
    # Jitter of 5 mm around the origin for 100 samples adds nothing.
    pts = [(i * 0.01, 0.005 * (-1) ** i, 0.0, 0.0, 0, 0, 0) for i in range(100)]
    assert m.path_length(pts) == 0.0


def roll_trace(values, dt=0.05):
    return [(i * dt, 0, 0, 0, r, 0.0, 0) for i, r in enumerate(values)]


def test_two_crossings_two_entries():
    over, under = 0.4, 0.0
    trace = [under] * 5 + [over] * 10 + [under] * 5 + [over] * 10 + [under] * 5
    assert m.unsafe_entries(roll_trace(trace), 0.3, 0.5) == 2


def test_short_spike_is_debounced():
    # 3 samples over the limit at 50 ms spacing span 0.1 s, under the 0.2 s debounce.
    trace = [0.0] * 5 + [0.4] * 3 + [0.0] * 5
    assert m.unsafe_entries(roll_trace(trace), 0.3, 0.5) == 0


def test_roll_and_pitch_together_count_once():
    pts = [(i * 0.05, 0, 0, 0, 0.4, 0.6, 0) for i in range(20)]
    assert m.unsafe_entries(pts, 0.3, 0.5) == 1


def test_tip_limits_hand_computed():
    phys = {'track': 0.436, 'wheelbase': 0.744729, 'cg_height': 0.45,
            'cg_offset_x': 0.0, 'cg_offset_y': 0.0, 'stability_margin_min': 0.17}
    roll, pitch = m.tip_limits(phys)
    # atan(0.218 / 0.45) - 0.17 = 0.45138 - 0.17 = 0.28138 rad = 16.11 deg.
    assert math.degrees(roll) == pytest.approx(16.11, abs=0.01)
    # atan(0.3723645 / 0.45) - 0.17 = 0.69127 - 0.17 = 0.52127 rad = 29.87 deg.
    assert math.degrees(pitch) == pytest.approx(29.87, abs=0.01)


def test_cg_offset_reduces_the_limit():
    phys = {'track': 0.4, 'wheelbase': 0.8, 'cg_height': 0.4,
            'cg_offset_x': 0.0, 'cg_offset_y': 0.1, 'stability_margin_min': 0.0}
    roll, _ = m.tip_limits(phys)
    assert roll == pytest.approx(math.atan2(0.1, 0.4))


def test_quat_to_rpy():
    # 30 degrees roll about x.
    h = math.radians(30) / 2
    r, p, y = m.quat_to_rpy(math.sin(h), 0.0, 0.0, math.cos(h))
    assert (math.degrees(r), p, y) == pytest.approx((30.0, 0.0, 0.0), abs=1e-9)


def test_success_needs_status_and_distance():
    assert m.is_success('succeeded', 0.2, 0.5)
    assert not m.is_success('succeeded', 0.8, 0.5)
    assert not m.is_success('aborted', 0.1, 0.5)


def test_final_error_and_max_attitude():
    pts = [(0, 0, 0, 0, 0.1, -0.2, 0), (1, 3, 4, 0, -0.3, 0.1, 0)]
    assert m.final_error(pts, (0.0, 0.0)) == pytest.approx(5.0)
    assert m.max_abs_attitude(pts) == pytest.approx((0.3, 0.2))


def test_estimate_error_spawn_at_origin():
    # Estimate (3, 4) heading 0, truth at (0, 0) heading 0.1: 5 m and 0.1 rad off.
    e = m.estimate_error((3.0, 4.0, 0.0), (0, 0.0, 0.0, 0, 0, 0, 0.1),
                         {'x': 0.0, 'y': 0.0, 'yaw': 0.0})
    assert e[0] == pytest.approx(5.0)
    assert e[1] == pytest.approx(0.1)


def test_estimate_error_rotated_spawn():
    # Spawn at (1, 2) facing +y: frame (2, 0) is world (1, 4). Truth there, so no error.
    e = m.estimate_error((2.0, 0.0, 0.0), (0, 1.0, 4.0, 0, 0, 0, math.pi / 2),
                         {'x': 1.0, 'y': 2.0, 'yaw': math.pi / 2})
    assert e[0] == pytest.approx(0.0, abs=1e-9)
    assert e[1] == pytest.approx(0.0, abs=1e-9)


def test_estimate_error_wraps_yaw():
    # 179 deg against -179 deg is 2 deg, not 358.
    e = m.estimate_error((0.0, 0.0, math.radians(179)), (0, 0, 0, 0, 0, 0, math.radians(-179)),
                         {'x': 0.0, 'y': 0.0, 'yaw': 0.0})
    assert math.degrees(e[1]) == pytest.approx(2.0)


def test_estimate_error_without_estimate():
    assert m.estimate_error(None, (0, 0, 0, 0, 0, 0, 0), {'x': 0, 'y': 0, 'yaw': 0}) is None


def test_refused_needs_no_success_and_staying_low():
    low = [(0, 0, 0, 0.1, 0, 0, 0), (1, 1, 0, 0.2, 0, 0, 0)]
    high = low + [(2, 2, 0, 0.4, 0, 0, 0)]
    assert m.is_refused('aborted', low, 0.25)
    assert not m.is_refused('succeeded', low, 0.25)
    assert not m.is_refused('aborted', high, 0.25)
