"""rover_bringup's rover_sim.launch.py with test-friendly defaults.

    ros2 launch rover_testing rover_sim.launch.py
    ros2 launch rover_testing rover_sim.launch.py gui:=true world:=bars.sdf

A thin wrapper: it includes the rover_bringup file, so it always runs the current
bringup code, plus its own GZ_PARTITION and a shutdown hook that kills the gz
server launch orphans (rover_testing/gz_reaper.py). Only the defaults below differ; every other rover_sim argument
passes straight through from the command line.

    world       empty_world.sdf
    gui         false   (gz server only, headless rendering)
    teleop_gui  false
    x, y, yaw   0, 0, 0 (the centre of empty_world.sdf's walled box)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from rover_testing.gz_reaper import isolate_and_reap

OVERRIDES = {
    "world": "empty_world.sdf",
    "gui": "false",
    "teleop_gui": "false",
    "x": "0.0",
    "y": "0.0",
    "yaw": "0.0",
}


def generate_launch_description():
    bringup = os.path.join(get_package_share_directory("rover_bringup"), "launch",
                           "rover_sim.launch.py")
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=value) for name, value in OVERRIDES.items()]
        + isolate_and_reap()
        + [GroupAction(scoped=True, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup),
            launch_arguments={name: LaunchConfiguration(name) for name in OVERRIDES}.items())])])
