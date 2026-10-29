"""Drive up a steeper slope: slopes.sdf, 4.8 m at 0.5 m/s up hill 2's 15 degree ramp.

    ros2 launch rover_testing drive_slope_15deg.launch.py

Spawn (2.6, 0) facing +x, on the flat strip between the hills. The ramp runs x 4.0
to 6.24, the 0.6 m plateau 6.24 to 8.24; the rover should stop on the plateau near
x 7.4, 0.6 m up. As drive_slope_8deg, at nearly twice the grade.
"""

from rover_testing.testcase import drive_test


def generate_launch_description():
    return drive_test("drive_slope_15deg", "slopes.sdf", spawn=(2.6, 0.0, 0.0),
                      distance=4.8, speed=0.5, expected_climb=0.6)
