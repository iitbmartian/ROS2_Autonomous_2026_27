"""Navigation onto a slope: slopes.sdf, (-9, 0) up hill 1's 8 degree ramp to its plateau.

    ros2 launch rover_testing nav_slope_plateau.launch.py

The goal, world (-2.2, 0), map (6.8, 0), sits on the 0.4 m plateau, which is only
reachable up the ramp (the hill spans the corridor wall to wall).

Expect this to expose the stack's height-only obstacle classification: RTAB-Map's
Grid/MaxGroundHeight and Nav2's min_obstacle_height are both 0.15 m above the
ground the rover stands on, and the ramp passes 0.15 m about 1.1 m up it. If the
ramp is mapped as a wall, Nav2 cannot plan onto it and the test fails with Nav2
ABORTED. drive_slope_8deg drives the same ramp without the planner, to tell a
vehicle limit from a perception one.
"""

from rover_testing.testcase import nav_test


def generate_launch_description():
    return nav_test("nav_slope_plateau", "slopes.sdf", spawn=(-9.0, 0.0, 0.0),
                    waypoints=[(6.8, 0.0)], timeout=120.0)
