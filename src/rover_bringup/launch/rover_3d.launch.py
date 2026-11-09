"""The 3D terrain navigation stack on the real rover (NUC), without the simulator.

    ros2 launch rover_bringup rover_3d.launch.py
    ros2 launch rover_bringup rover_3d.launch.py governor_v_nominal:=0.15 safety:=true

Starts terrain_mapping.launch.py (sim:=false: terrain_mapping_real.yaml, the L2 on
/unilidar/cloud, gravity from the IMU, depth_range_cut left to the Orin), navigation_3d.launch.py
(sim:=false: the tuned 3D Nav2 stack with nav2_params_3d_real.yaml), the speed governor and the
safety monitor, all on wall time.

Expects from the rest of the rover, not started here: the L2 and SBG drivers, the EKF on
/odometry/filtered with odom -> base_footprint, RTAB-Map's map -> odom, the static TF from
base_footprint to unilidar_lidar (measured mount), and rover_kinematics_node taking /cmd_vel.
Every hardware fact this needs is marked TO VERIFY in terrain_mapping_real.yaml and
nav2_params_3d_real.yaml. rover_traversability/config/rover_physical.yaml still holds
placeholders (cg_height, mass), so the tip limits are estimates until it is measured.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    launch_dir = os.path.join(get_package_share_directory('rover_bringup'), 'launch')
    base = arg('base_frame')
    odom = arg('odom_topic')
    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'terrain_mapping.launch.py')),
            launch_arguments={'sim': 'false', 'depth_range_cut': arg('depth_range_cut'),
                              'base_frame': base, 'sensor_frame': arg('sensor_frame'),
                              'lowest_beam': arg('lowest_beam')}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation_3d.launch.py')),
            launch_arguments={'sim': 'false',
                              'nav_extra_overlay': arg('nav_extra_overlay')}.items()),
    ]
    if arg('governor').lower() in ('true', '1'):
        actions.append(Node(
            package='rover_traversability', executable='terrain_speed_governor', output='screen',
            parameters=[{'use_sim_time': False, 'base_frame': base, 'odom_topic': odom,
                         'v_nominal': float(arg('governor_v_nominal')),
                         'u_free': float(arg('governor_u_free'))}]))
    if arg('safety').lower() in ('true', '1'):
        actions.append(Node(
            package='rover_traversability', executable='terrain_safety_monitor', output='screen',
            parameters=[{'use_sim_time': False, 'base_frame': base, 'odom_topic': odom,
                         'imu_topic': arg('imu_topic')}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('base_frame', default_value='base_footprint',
                              description='Ground-level body frame of the real TF tree'),
        DeclareLaunchArgument('odom_topic', default_value='/odometry/filtered'),
        DeclareLaunchArgument('imu_topic', default_value='/imu/data',
                              description='Attitude for the tip monitor (SBG)'),
        DeclareLaunchArgument('sensor_frame', default_value='unilidar_lidar',
                              description='L2 frame, for negative-obstacle occlusion reasoning'),
        DeclareLaunchArgument('lowest_beam', default_value='-1.5708',
                              description='L2 mounted inverted reaches straight down'),
        DeclareLaunchArgument('depth_range_cut', default_value='false',
                              description='true only when no Orin runs it'),
        DeclareLaunchArgument('nav_extra_overlay', default_value=''),
        # Slower than simulation's 0.5 for the first hardware runs.
        DeclareLaunchArgument('governor_v_nominal', default_value='0.25'),
        DeclareLaunchArgument('governor_u_free', default_value='0.3'),
        DeclareLaunchArgument('governor', default_value='true'),
        DeclareLaunchArgument('safety', default_value='true'),
        OpaqueFunction(function=_launch)])
