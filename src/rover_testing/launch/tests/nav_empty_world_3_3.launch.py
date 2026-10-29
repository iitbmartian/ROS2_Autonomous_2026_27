"""Navigation: centre of empty_world.sdf to (3, 3) with Nav2.

    ros2 launch rover_testing nav_empty_world_3_3.launch.py
    ros2 launch rover_testing nav_empty_world_3_3.launch.py tolerance:=0.2 gui:=true

The spawn is the world origin facing +x, so map and world coincide; (3, 3) is well
inside the walls at +-4.5 m (the (5, 5) first asked for is outside them). The goal
faces along the straight line from the spawn: a goal yaw across it would also make
the rover, 0.8 m minimum turn radius, manoeuvre to within Nav2's 0.25 rad yaw
tolerance, which measured as a 120 s timeout 0.33 m short of the goal.

Benchmarks: time to goal, path length against the 4.24 m straight line, initial and
final localization error. Checks: see goal_benchmark.py.
"""

from rover_testing.testcase import nav_test


def generate_launch_description():
    return nav_test("nav_empty_world_3_3", "empty_world.sdf", spawn=(0.0, 0.0, 0.0),
                    waypoints=[(3.0, 3.0)], timeout=120.0)
