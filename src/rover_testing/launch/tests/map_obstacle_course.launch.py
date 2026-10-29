"""Mapping: a tour of obstacle_course.sdf, scored against the ideal map.

    ros2 launch rover_testing map_obstacle_course.launch.py

Spawn (-6.5, 0). Route in the world frame: (-1.5, -0.6), (2.5, -1.5), (6, -2.5),
(6, 2.5), (2.5, 3); the map frame adds 6.5 to x. It passes every obstacle and sees
both halves of the arena. maps/obstacle_course.yaml has the four 1 m obstacles
occupied and the 8 cm plate at (-5.5, -3) free: it is under the 0.15 m step the
rover climbs, so a correct height-classified map leaves it free too.
"""

from rover_testing.testcase import nav_test


def generate_launch_description():
    return nav_test("map_obstacle_course", "obstacle_course.sdf", spawn=(-6.5, 0.0, 0.0),
                    waypoints=[(5.0, -0.6), (9.0, -1.5), (12.5, -2.5), (12.5, 2.5),
                               (9.0, 3.0)],
                    timeout=300.0, mapping={"reference": "obstacle_course"})
