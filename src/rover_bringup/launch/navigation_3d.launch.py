"""Nav2 with the 3D terrain parameters, without the simulator.

    ros2 launch rover_bringup navigation_3d.launch.py sim:=true
    ros2 launch rover_bringup navigation_3d.launch.py sim:=false     # real rover

nav2_params_sim_3d.yaml merged over nav2_params_sim.yaml (rover_nav2.params_overlay), written
under /tmp and passed to navigation.launch.py, which is not changed. Pair it with
terrain_mapping.launch.py. pipeline_3d.launch.py starts everything together in simulation.
sim:=false merges nav2_params_3d_real.yaml last (real odometry topic), on the same tuned base,
and runs on wall time. rover_3d.launch.py starts the whole 3D stack on the rover.
"""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from rover_nav2.params_overlay import merge_files


def _launch(context):
    sim = LaunchConfiguration('sim').perform(context).lower() in ('true', '1')
    cfg = os.path.join(get_package_share_directory('rover_nav2'), 'config')
    overlays = [os.path.join(cfg, 'nav2_params_sim_3d.yaml')]
    if not sim:
        overlays.append(os.path.join(cfg, 'nav2_params_3d_real.yaml'))
    extra = LaunchConfiguration('nav_extra_overlay').perform(context)
    if extra:
        overlays.append(extra)
    merged = os.path.join(tempfile.gettempdir(),
                          f"rover_nav2_params_{'sim' if sim else 'real'}_3d_{os.getpid()}.yaml")
    merge_files(os.path.join(cfg, 'nav2_params_sim.yaml'), overlays, merged)
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('rover_bringup'), 'launch', 'navigation.launch.py')),
        launch_arguments={'sim': 'true' if sim else 'false', 'params_file': merged}.items())]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='true'),
        DeclareLaunchArgument('nav_extra_overlay', default_value='',
                              description='optional params file merged last'),
        OpaqueFunction(function=_launch)])
