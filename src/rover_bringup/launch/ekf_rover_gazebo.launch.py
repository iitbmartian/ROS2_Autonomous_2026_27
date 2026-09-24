"""EKF on the rover_gazebo four-wheel-steer rover.

Runs the per-wheel steer-aware encoder chain into robot_localization, against a
simulation that is already running. It does not start Gazebo;
rover_sim.launch.py does that.

    ros2 launch rover_bringup rover_sim.launch.py
    ros2 launch rover_bringup ekf_rover_gazebo.launch.py

    /joint_states -> steer_wheel_splitter -> /wheel_xx/steer_encoder
                  -> steer_wheel_relay (x4) -> /wheel_xx/odom_relayed -> ekf_filter_node (vx, vy)
                  -> wheel_body_velocity -> /wheel_odom/body          -> ekf_filter_node (yaw rate)

    use_imu:=true also fuses the gyro yaw rate from /imu_relay.

    publish_tf:=false stops the filter publishing odom -> base_footprint, for
    when something else owns that transform.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_name = 'rover_ekf'
    pkg_share = get_package_share_directory(pkg_name)

    ekf_config_path = os.path.join(pkg_share, 'config', 'ekf_rover_gazebo.yaml')
    ekf_imu_config_path = os.path.join(pkg_share, 'config', 'ekf_rover_gazebo_imu.yaml')
    wheels_config_path = os.path.join(pkg_share, 'config', 'steer_wheels.yaml')
    kinematics_file = os.path.join(
        get_package_share_directory('rover_gazebo'), 'config', 'rover_kinematics.yaml')
    # Overrides publish_tf in ekf_rover_gazebo.yaml.
    publish_tf = {'publish_tf': ParameterValue(LaunchConfiguration('publish_tf'), value_type=bool)}

    def wheel_relay(wheel):
        # One instance per wheel, each with its own block in steer_wheels.yaml
        # so noise can be tuned per-wheel independently for testing.
        return Node(
            package=pkg_name,
            executable='steer_wheel_relay',
            name=wheel,
            output='screen',
            parameters=[wheels_config_path, {'use_sim_time': True}],
        )

    return LaunchDescription([
        DeclareLaunchArgument('publish_tf', default_value='true',
                              description='publish odom -> base_footprint from the filter'),
        DeclareLaunchArgument('use_imu', default_value='false',
                              description='also fuse the gyro yaw rate from /imu_relay'),

        # EKF node, fusing the four wheel velocities and the wheel-fitted yaw rate
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[ekf_config_path, {'use_sim_time': True}, publish_tf],
            condition=UnlessCondition(LaunchConfiguration('use_imu')),
        ),
        # Same, plus the gyro
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[ekf_config_path, ekf_imu_config_path, {'use_sim_time': True}, publish_tf],
            condition=IfCondition(LaunchConfiguration('use_imu')),
        ),

        # rover_gazebo bridges ground truth on /odom and the IMU on /imu/data
        Node(
            package=pkg_name,
            executable='covariance_relay',
            name='covariance_relay',
            output='screen',
            parameters=[{'use_sim_time': True,
                         'odom_topic': '/odom',
                         'imu_topic': '/imu/data'}],
        ),
        Node(
            package=pkg_name,
            executable='steer_wheel_splitter',
            name='steer_wheel_splitter',
            output='screen',
            parameters=[wheels_config_path,
                        {'use_sim_time': True, 'kinematics_file': kinematics_file}],
        ),

        Node(
            package=pkg_name,
            executable='wheel_body_velocity',
            name='wheel_body_velocity',
            output='screen',
            parameters=[{'use_sim_time': True, 'kinematics_file': kinematics_file}],
        ),

        wheel_relay('wheel_fl'),
        wheel_relay('wheel_fr'),
        wheel_relay('wheel_rl'),
        wheel_relay('wheel_rr'),
    ])
