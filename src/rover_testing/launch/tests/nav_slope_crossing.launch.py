"""Navigation over a hill: slopes.sdf, (-9, 0) over hill 1 and down to (2.8, 0).

    ros2 launch rover_testing nav_slope_crossing.launch.py

Up the 8 degree ramp, across the 0.4 m plateau, down the far ramp, to the flat strip
between the two hills. World (2.8, 0), map (11.8, 0). The hill spans the corridor,
so there is no way around. See nav_slope_plateau.launch.py for what is expected to
stop the planner here.

Via the plateau, map (6.8, 0): Nav2 refuses goals outside RTAB-Map's /map, which
at the spawn reaches only about 8 m, and the far side of the hill is not seen
until the rover is on top of it.
"""

from rover_testing.testcase import nav_test


def generate_launch_description():
    return nav_test("nav_slope_crossing", "slopes.sdf", spawn=(-9.0, 0.0, 0.0),
                    waypoints=[(6.8, 0.0), (11.8, 0.0)], timeout=150.0)
