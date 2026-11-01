"""Nav2 with the 3D terrain parameters, without the simulator.

    ros2 launch rover_bringup navigation_3d.launch.py sim:=true

nav2_params_sim_3d.yaml merged over nav2_params_sim.yaml (rover_nav2.params_overlay), written
under /tmp and passed to navigation.launch.py, which is not changed. Pair it with
terrain_mapping.launch.py. pipeline_3d.launch.py starts everything together in simulation.
Only the simulation overlay exists so far, so sim:=false refuses to start.
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
    if LaunchConfiguration('sim').perform(context).lower() not in ('true', '1'):
        raise RuntimeError('navigation_3d: only the simulation overlay exists so far (sim:=true)')
    cfg = os.path.join(get_package_share_directory('rover_nav2'), 'config')
    merged = os.path.join(tempfile.gettempdir(), f'rover_nav2_params_sim_3d_{os.getpid()}.yaml')
    merge_files(os.path.join(cfg, 'nav2_params_sim.yaml'),
                os.path.join(cfg, 'nav2_params_sim_3d.yaml'), merged)
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('rover_bringup'), 'launch', 'navigation.launch.py')),
        launch_arguments={'sim': 'true', 'params_file': merged}.items())]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='true'),
        OpaqueFunction(function=_launch)])
