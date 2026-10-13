"""Optional GNSS/GPS position fusion for the rover EKF.

Converts a raw sensor_msgs/NavSatFix into an odometry message in the EKF's
own frame via robot_localization's navsat_transform_node, so GPS can be
fused as another position source alongside the wheels (and IMU, if enabled):

    gps_topic (NavSatFix) -\\
    imu_topic (Imu)         |-> navsat_transform_node -> /odometry/gps -> ekf_filter_node (x, y)
    /odometry/filtered (the EKF's own output, used to anchor the local origin) -/

Included from rover_bringup's ekf_rover_gazebo.launch.py when use_gps:=true;
see ekf_rover_gazebo_gps.yaml for how the EKF fuses /odometry/gps.

Nothing in rover_gazebo publishes a NavSatFix yet -- this just wires up the
fusion path for whenever a GPS/GNSS source (sim plugin or real hardware,
e.g. an SBG receiver) is added.
"""
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('rover_ekf')
    navsat_config_path = os.path.join(pkg_share, 'config', 'navsat_transform.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('gps_topic', default_value='/gps/fix',
                              description='sensor_msgs/NavSatFix topic to fuse'),
        DeclareLaunchArgument('imu_topic', default_value='/imu_relay',
                              description='sensor_msgs/Imu topic providing heading'),

        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[navsat_config_path, {'use_sim_time': True}],
            remappings=[
                ('imu', LaunchConfiguration('imu_topic')),
                ('gps/fix', LaunchConfiguration('gps_topic')),
            ],
        ),
    ])
