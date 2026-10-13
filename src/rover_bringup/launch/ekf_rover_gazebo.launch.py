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

    use_gps:=true also fuses GPS/GNSS position via navsat_transform_node
    (rover_ekf's gps_fusion.launch.py); requires something publishing
    sensor_msgs/NavSatFix on gps_topic (default /gps/fix).

    publish_tf:=false stops the filter publishing odom -> base_footprint, for
    when something else owns that transform.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_name = 'rover_ekf'
    pkg_share = get_package_share_directory(pkg_name)

    ekf_config_path = os.path.join(pkg_share, 'config', 'ekf_rover_gazebo.yaml')
    ekf_imu_config_path = os.path.join(pkg_share, 'config', 'ekf_rover_gazebo_imu.yaml')
    ekf_gps_config_path = os.path.join(pkg_share, 'config', 'ekf_rover_gazebo_gps.yaml')
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

    def ekf_node(context, *args, **kwargs):
        # odomN sources beyond the four wheels (imu, gps) are layered on by
        # stacking their yaml files, so this stays one node regardless of
        # which options are enabled.
        ekf_params = [ekf_config_path]
        if LaunchConfiguration('use_imu').perform(context) == 'true':
            ekf_params.append(ekf_imu_config_path)
        if LaunchConfiguration('use_gps').perform(context) == 'true':
            ekf_params.append(ekf_gps_config_path)
        ekf_params += [{'use_sim_time': True}, publish_tf]

        return [Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=ekf_params,
        )]

    return LaunchDescription([
        DeclareLaunchArgument('publish_tf', default_value='true',
                              description='publish odom -> base_footprint from the filter'),
        DeclareLaunchArgument('use_imu', default_value='false',
                              description='also fuse the gyro yaw rate from /imu_relay'),
        DeclareLaunchArgument('use_gps', default_value='false',
                              description='also fuse GPS/GNSS position via navsat_transform_node'),
        DeclareLaunchArgument('gps_topic', default_value='/gps/fix',
                              description='sensor_msgs/NavSatFix topic, only used when use_gps:=true'),

        # EKF node, fusing the four wheel velocities and the wheel-fitted yaw
        # rate, plus the IMU and/or GPS when those options are enabled.
        OpaqueFunction(function=ekf_node),

        # GPS/GNSS fusion: navsat_transform_node converting a NavSatFix into
        # odometry the EKF above can fuse. Lives in rover_ekf since it's
        # purely an EKF input; see rover_ekf/launch/gps_fusion.launch.py.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'gps_fusion.launch.py')),
            launch_arguments={'gps_topic': LaunchConfiguration('gps_topic')}.items(),
            condition=IfCondition(LaunchConfiguration('use_gps')),
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
