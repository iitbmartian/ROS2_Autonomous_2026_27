"""Mapping a slope: the start of slopes.sdf, looking at hill 1, scored against the ideal map.

    ros2 launch rover_testing map_slopes.launch.py

Spawn (-9, 0). A short loop on the flat ground before the hill, world (-7, 1.2),
(-10, -1.2), (-9, 0), facing the 8 degree ramp from 1 to 3 m away. Scored only
over x up to 1.7 (walls included): the flat start and hill 1, what the route sees.

The ideal map (maps/slopes.yaml) has the ramp and plateau free: the rover can
drive them (drive_slope_8deg). A height-only grid marks every cell more than
0.15 m above the rover as an obstacle, so expect phantom obstacles (red in the
overlay) over most of the hill, and low precision. That gap between "what is
drivable" and "what the map says" is what this test measures.
"""

from rover_testing.testcase import nav_test


def generate_launch_description():
    return nav_test("map_slopes", "slopes.sdf", spawn=(-9.0, 0.0, 0.0),
                    waypoints=[(2.0, 1.2), (-1.0, -1.2), (0.0, 0.0)],
                    timeout=200.0,
                    mapping={"reference": "slopes", "region": (-12.3, -3.3, 1.7, 3.3)})
