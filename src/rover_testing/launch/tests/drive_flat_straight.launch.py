"""Drive, flat baseline: empty_world.sdf, 5.5 m straight ahead at 0.5 m/s, no planner.

    ros2 launch rover_testing drive_flat_straight.launch.py

Spawn (-3, 0) facing +x; the rover should end near (2.5, 0), short of the east wall
at 4.4 m. The reference for the slope drives: the slip, drift and localization error
the vehicle and estimator show with no terrain at all.
"""

from rover_testing.testcase import drive_test


def generate_launch_description():
    return drive_test("drive_flat_straight", "empty_world.sdf", spawn=(-3.0, 0.0, 0.0),
                      distance=5.5, speed=0.5)
