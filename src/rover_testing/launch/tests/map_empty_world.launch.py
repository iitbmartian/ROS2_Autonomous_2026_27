"""Mapping: a 4 m square around empty_world.sdf and back, scored against the ideal map.

    ros2 launch rover_testing map_empty_world.launch.py

Route (map = world here): (2, 2), (2, -2), (-2, -2), (-2, 2), back to (0, 0). Then
RTAB-Map's /map is compared with maps/empty_world.yaml: the four walls, nothing
else. Also a loop: the final localization error is the drift after returning home.

Benchmarks: map precision (phantom obstacles), recall (missed walls), coverage,
mean obstacle position error, plus the navigation numbers. The report links an
overlay image: green found, red phantom, blue missed.
"""

from rover_testing.testcase import nav_test


def generate_launch_description():
    return nav_test("map_empty_world", "empty_world.sdf", spawn=(0.0, 0.0, 0.0),
                    waypoints=[(2.0, 2.0), (2.0, -2.0), (-2.0, -2.0), (-2.0, 2.0), (0.0, 0.0)],
                    timeout=300.0, mapping={"reference": "empty_world"})
