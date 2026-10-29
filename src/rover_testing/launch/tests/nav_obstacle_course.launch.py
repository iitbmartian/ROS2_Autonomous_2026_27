"""Navigation: across obstacle_course.sdf, (-6.5, 0) to (6.5, 0), around the slalom.

    ros2 launch rover_testing nav_obstacle_course.launch.py

The straight line is blocked twice (block_a at x -3, block_d at x 4) and squeezes
between pillar_c and block_b at x 0.5 to 1, so the planner has to plan, and the
local controller has to follow, a real path. Goal (13, 0) in the map frame.

Via world (-1, -0.8), map (5.5, -0.8), because Nav2 refuses goals outside the
global costmap, which is RTAB-Map's /map: at the spawn that covers only about the
lidar's 8 m grid range, and sent straight, (13, 0) was rejected as "outside
bounds" before the rover moved. By the waypoint the far wall is mapped.

Benchmarks: time to goal, path length against the 13 m straight line, recoveries,
localization error.
"""

from rover_testing.testcase import nav_test


def generate_launch_description():
    return nav_test("nav_obstacle_course", "obstacle_course.sdf", spawn=(-6.5, 0.0, 0.0),
                    waypoints=[(5.5, -0.8), (13.0, 0.0)], timeout=180.0)
