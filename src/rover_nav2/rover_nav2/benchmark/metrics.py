"""Per-run metrics from ground truth. Pure functions on recorded series, testable without ROS.

A pose series is a list of (t, x, y, z, roll, pitch, yaw) tuples in seconds, metres and radians,
from Gazebo's ground-truth odometry.
"""

import math

# Movements smaller than this between kept samples are vibration, not travel.
PATH_STEP_MIN = 0.02     # m
# A limit violation must last this long to count, so one noisy sample is not an entry.
UNSAFE_DEBOUNCE = 0.2    # s


def quat_to_rpy(qx, qy, qz, qw):
    """Roll, pitch, yaw (rad) of a unit quaternion, ZYX convention."""
    roll = math.atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))
    s = max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx)))
    pitch = math.asin(s)
    yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    return roll, pitch, yaw


def tip_limits(phys):
    """Roll and pitch limits (rad) from rover_physical values: the static tip-over angle about
    the support polygon edge, minus stability_margin_min. Uses the wheel contacts at
    (+-wheelbase/2, +-track/2) and the CG at (cg_offset_x, cg_offset_y, cg_height)."""
    h = phys['cg_height']
    roll_tip = math.atan2(phys['track'] / 2.0 - abs(phys['cg_offset_y']), h)
    pitch_tip = math.atan2(phys['wheelbase'] / 2.0 - abs(phys['cg_offset_x']), h)
    m = phys['stability_margin_min']
    return roll_tip - m, pitch_tip - m


def path_length(poses, step_min=PATH_STEP_MIN):
    """3D distance travelled, counting only moves of at least step_min from the last kept point."""
    if not poses:
        return 0.0
    total = 0.0
    _, lx, ly, lz = poses[0][:4]
    for p in poses[1:]:
        d = math.dist((lx, ly, lz), p[1:4])
        if d >= step_min:
            total += d
            lx, ly, lz = p[1:4]
    return total


def unsafe_entries(poses, roll_limit, pitch_limit, debounce=UNSAFE_DEBOUNCE):
    """Number of separate episodes where |roll| or |pitch| exceeds its limit for at least
    debounce seconds. Roll and pitch over the limit at the same time are one episode."""
    entries = 0
    start = None
    counted = False
    for t, _, _, _, roll, pitch, _ in poses:
        over = abs(roll) > roll_limit or abs(pitch) > pitch_limit
        if over:
            if start is None:
                start, counted = t, False
            if not counted and t - start >= debounce:
                entries += 1
                counted = True
        else:
            start = None
    return entries


def max_abs_attitude(poses):
    """Largest |roll| and |pitch| (rad) in the series."""
    if not poses:
        return 0.0, 0.0
    return max(abs(p[4]) for p in poses), max(abs(p[5]) for p in poses)


def final_error(poses, goal_xy):
    """Ground-truth planar distance from the last pose to the goal."""
    if not poses:
        return float('nan')
    return math.hypot(poses[-1][1] - goal_xy[0], poses[-1][2] - goal_xy[1])


def is_success(status, final_err, radius):
    """Success needs both: Nav2 reported SUCCEEDED and ground truth is within radius of the goal."""
    return status == 'succeeded' and final_err <= radius


def estimate_error(est, gt_pose, spawn):
    """Position and yaw error of a frame's estimate (x, y, yaw), which starts at the spawn pose,
    against the last ground-truth pose (t, x, y, z, roll, pitch, yaw). None without an estimate."""
    if est is None:
        return None
    c, s = math.cos(spawn['yaw']), math.sin(spawn['yaw'])
    wx = spawn['x'] + c * est[0] - s * est[1]
    wy = spawn['y'] + s * est[0] + c * est[1]
    dyaw = math.atan2(math.sin(est[2] + spawn['yaw'] - gt_pose[6]),
                      math.cos(est[2] + spawn['yaw'] - gt_pose[6]))
    return math.hypot(wx - gt_pose[1], wy - gt_pose[2]), abs(dyaw)


def is_refused(status, poses, z_max):
    """A goal that should be refused: Nav2 did not report success and the rover never rose above
    z_max (it stayed off the hazard)."""
    return status != 'succeeded' and all(p[3] <= z_max for p in poses)
