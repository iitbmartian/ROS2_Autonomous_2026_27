"""Build a test case launch description: the pipeline plus one benchmark node.

Every file in launch/tests/ is a few constants and one call to nav_test() or
drive_test(). They all take the same launch arguments, which run_tests' UI relies
on:

    tolerance    m, localization error allowed against ground truth (default 0.3)
    timeout      sim s for the test's own motion (default: the test's own)
    gui          show the Gazebo GUI (default false)
    result_file  where the JSON report goes

and they all shut the whole launch down when the benchmark node exits, so a test
file runs one test and returns.

World names: a name in rover_testing/worlds (obstacle_course, slopes) resolves to
that file; anything else is passed through to rover_gazebo/worlds.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            RegisterEventHandler, Shutdown)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PKG = "rover_testing"


def share(*parts):
    return os.path.join(get_package_share_directory(PKG), *parts)


def world_path(world):
    local = share("worlds", world)
    return local if os.path.exists(local) else world


def reference_map(name):
    return share("maps", f"{name}.yaml")


def _test(test_name, world, spawn, timeout, executable, parameters, nav2=True):
    x, y, yaw = spawn
    pipeline = GroupAction(scoped=True, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(share("launch", "pipeline.launch.py")),
        launch_arguments={
            "world": world_path(world),
            "x": str(x),
            "y": str(y),
            "yaw": str(yaw),
            "gui": LaunchConfiguration("gui"),
            "nav2": "true" if nav2 else "false",
        }.items())])

    benchmark = Node(
        package=PKG, executable=executable, name=executable, output="screen",
        parameters=[{
            "use_sim_time": True,
            "test_name": test_name,
            "spawn_x": float(x),
            "spawn_y": float(y),
            "spawn_yaw": float(yaw),
            "tolerance": ParameterValue(LaunchConfiguration("tolerance"), value_type=float),
            "timeout": ParameterValue(LaunchConfiguration("timeout"), value_type=float),
            "result_file": LaunchConfiguration("result_file"),
            **parameters,
        }])

    return LaunchDescription([
        DeclareLaunchArgument("tolerance", default_value="0.3",
                              description="m, localization error allowed against ground truth"),
        DeclareLaunchArgument("timeout", default_value=str(float(timeout)),
                              description="sim seconds allowed for the test's own motion"),
        DeclareLaunchArgument("gui", default_value="false", description="show the Gazebo GUI"),
        DeclareLaunchArgument("result_file",
                              default_value=f"~/.ros/rover_testing/{test_name}.json"),
        pipeline,
        benchmark,
        RegisterEventHandler(OnProcessExit(
            target_action=benchmark,
            on_exit=[Shutdown(reason=f"{test_name} finished")])),
    ])


def nav_test(test_name, world, spawn, waypoints, timeout=120.0, mapping=None):
    """Nav2 route through waypoints [(x, y), ...] in the map frame (spawn = origin).

    mapping: None, or a dict with "reference" (a maps/ name) and optionally
    "min_precision", "min_recall", "min_coverage", "match_distance" overrides and
    "region": (x_min, y_min, x_max, y_max), world frame, to score only that area.
    """
    parameters = {"waypoints": [float(v) for p in waypoints for v in p]}
    if mapping:
        # Score the map, not the parking: pass every waypoint within pass_radius,
        # move on from a stuck leg after 90 s.
        parameters["navigation_checks"] = False
        parameters["leg_timeout"] = float(mapping.get("leg_timeout", 90.0))
        parameters["reference_map"] = reference_map(mapping["reference"])
        for key in ("min_precision", "min_recall", "min_coverage", "match_distance"):
            if key in mapping:
                parameters[f"map_{key}"] = float(mapping[key])
        if "region" in mapping:
            parameters["map_region"] = [float(v) for v in mapping["region"]]
    return _test(test_name, world, spawn, timeout, "goal_benchmark", parameters)


def drive_test(test_name, world, spawn, distance, speed=0.5, expected_climb=0.0,
               timeout=60.0, min_progress=0.85):
    """Fixed forward command for distance / speed seconds, with Nav2 left out."""
    parameters = {"distance": float(distance), "speed": float(speed),
                  "expected_climb": float(expected_climb),
                  "min_progress": float(min_progress)}
    return _test(test_name, world, spawn, timeout, "drive_benchmark", parameters, nav2=False)
