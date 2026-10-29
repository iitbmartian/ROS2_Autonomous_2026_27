"""Drive up a slope: slopes.sdf, 6.5 m at 0.5 m/s up hill 1's 8 degree ramp, no planner.

    ros2 launch rover_testing drive_slope_8deg.launch.py

Spawn (-9, 0) facing +x. The ramp runs x -6.0 to -3.15, the 0.4 m plateau -3.15 to
-1.15; the rover should stop on the plateau near x -2.5, 0.4 m up.

Benchmarks: progress (slip on the slope), height gained, peak pitch, and the
localization error all the way up, where the EKF runs in 3D off the IMU and GPS.
"""

from rover_testing.testcase import drive_test


def generate_launch_description():
    return drive_test("drive_slope_8deg", "slopes.sdf", spawn=(-9.0, 0.0, 0.0),
                      distance=6.5, speed=0.5, expected_climb=0.4)
