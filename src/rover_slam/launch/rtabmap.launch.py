"""RTAB-Map SLAM for the rover, against a running rover_gazebo simulation.

This brings up the SLAM stack only. Start the simulator separately, or use
slam_sim.launch.py, which starts both.

    ros2 launch rover_gazebo rover_sim.launch.py
    ros2 launch rover_slam rtabmap.launch.py rviz:=true

Inputs, all published by rover_gazebo's ros_gz_bridge:

    /camera/image_raw    sensor_msgs/Image        RGB, 15 Hz
    /camera/depth_image  sensor_msgs/Image        depth, aligned, 15 Hz
    /camera/camera_info  sensor_msgs/CameraInfo   intrinsics, 15 Hz
    /lidar/points        sensor_msgs/PointCloud2  3D lidar, 5 Hz
    /imu/data            sensor_msgs/Imu          100 Hz
    /odom                nav_msgs/Odometry        ground truth, only for odom_source:=ground_truth

Outputs:

    /map                 nav_msgs/OccupancyGrid   the 2D grid Nav2 plans on
    /rtabmap/cloud_map   sensor_msgs/PointCloud2  the assembled 3D map
    /rtabmap/odom        nav_msgs/Odometry        visual odometry
    TF                   map -> odom -> base_footprint

Structure follows rtabmap_demos/launch/husky/husky_slam3d.launch.py, which is the
same problem: a simulated robot carrying a 3D lidar and an RGB-D camera.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = "rover_slam"


def launch_setup(context: LaunchContext, *args, **kwargs):
    # Resolved here rather than through IfCondition so the node list is built in
    # plain Python. Conditions would leave rtabmap holding an empty argv entry
    # for the database flag. This is the pattern upstream uses in
    # rtabmap_examples/launch/vlp16_zed.launch.py.
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    # odom_source is a string rather than a bool because a third source (wheel
    # odometry from rover_controls) is expected to land here later.
    odom_source = arg("odom_source")
    localization = arg("localization") == "true"
    delete_db = arg("delete_db_on_start") == "true"
    database_path = os.path.expanduser(arg("database_path"))

    # ------------------------------------------------------------------
    # Shared by every RTAB-Map node.
    #
    # RTAB-Map's own parameters, the ones with a slash in the name, are passed
    # as strings. The node forwards them into RTAB-Map's string-keyed parameter
    # map, so a real bool or float here is silently mishandled. Plain ROS node
    # parameters above them are normally typed.
    # ------------------------------------------------------------------
    shared_parameters = {
        "frame_id": "base_footprint",
        "map_frame_id": "map",
        "use_sim_time": True,
        # ros_gz_bridge publishes reliable. RTAB-Map defaults to best effort,
        # which matches nothing the bridge sends and drops every message with
        # no error. 1 = reliable.
        "qos": 1,
        "qos_imu": 1,
        # Camera at 15 Hz, lidar at 5 Hz. Nothing lines up exactly, so exact
        # sync would never fire.
        "approx_sync": True,
        "approx_sync_max_interval": 0.1,
        # The default 0.1 s is tight when the clock comes across the bridge from
        # Gazebo and TF lands late. Same value upstream uses for a simulated robot.
        "wait_for_transform": 0.2,

        # Reg/Strategy is deliberately NOT here. rgbd_odometry supports visual
        # registration only and logs "Ignoring value 2" if it sees it, so it is
        # set on the mapping node alone.

        # Not 3DoF. The husky demo this is modelled on forces 3DoF for a flat
        # warehouse floor, but this rover exists to be driven over ledge.sdf and
        # bars.sdf, where roll and pitch are signal rather than noise.
        "Reg/Force3DoF": "false",

        "Icp/PointToPlane": "true",
        "Icp/PointToPlaneK": "10",
        "Icp/VoxelSize": "0.1",
        "Icp/MaxCorrespondenceDistance": "1.0",   # ~10x voxel size
        "Icp/MaxTranslation": "1.0",
        # The lidar sits on the roof with a 0.5 m minimum range, but be explicit
        # so self-hits can never enter the match.
        "Icp/RangeMin": "0.5",
        # 16 rings over a +/- 15 degree fan is a ring-like scan; flip ground
        # normals up so point-to-plane behaves on flat terrain.
        "Icp/PointToPlaneGroundNormalsUp": "0.9",
    }

    # ------------------------------------------------------------------
    # Visual odometry.
    # ------------------------------------------------------------------
    odom_parameters = {
        "odom_frame_id": "odom",
        # Hold initialisation until the IMU has arrived, so the very first pose
        # is already gravity-aligned instead of being corrected later.
        "wait_imu_to_init": True,
        "publish_tf": True,
        "Odom/Strategy": "0",          # frame-to-map, the default and the steadiest
        "Odom/GuessMotion": "true",
        # The rover can crab and spot-turn, which look nothing like the motion
        # model. Rather than fail, reset after 10 dead frames and carry on.
        "Odom/ResetCountdown": "10",
        "Vis/MinInliers": "15",        # 640x360 is a small image; 20 is too strict
        "Vis/EstimationType": "1",     # 3D->2D PnP
    }

    # ------------------------------------------------------------------
    # Mapping.
    # ------------------------------------------------------------------
    rtabmap_parameters = {
        "subscribe_rgbd": True,
        "subscribe_scan_cloud": True,
        "subscribe_depth": False,
        "subscribe_rgb": False,
        "database_path": database_path,
        "odom_sensor_sync": True,
        # subscribe_odom_info is deliberately absent. rtabmap synchronises through
        # a fixed set of message_filters combinations, and while rgbdOdomScan3d
        # and rgbdOdomInfo both exist, there is no rgbdOdomScan3dInfo. Subscribing
        # to RGB-D and lidar together therefore rules OdomInfo out, and asking for
        # it only produces "subscribe_odom_info ignored..." at startup. The
        # combination is listed in rtabmap_sync/CommonDataSubscriber.h.

        # Off by default. Without it RTAB-Map builds no occupancy grid at all
        # and /map never appears, with nothing in the log to say why.
        "RGBD/CreateOccupancyGrid": "true",
        # 0 = laser scan, 1 = depth image, 2 = both. Both: the 360 degree lidar
        # for range and coverage, the camera depth for close ground detail the
        # roof lidar cannot see over the chassis.
        "Grid/Sensor": "2",
        # This is the 3D-to-2D squish. False projects the cloud onto xy and
        # emits the plain OccupancyGrid Nav2 consumes.
        "Grid/3D": "false",
        "Grid/CellSize": "0.05",
        "Grid/RangeMin": "0.5",
        "Grid/RangeMax": "15.0",
        "Grid/RayTracing": "true",     # fill known-free space between rover and hits
        # Height segmentation rather than normals. ledge.sdf is a 10 cm step the
        # rover is supposed to climb; a normals-based segmenter reads its face
        # as a wall. 0.15 m clears the ledge, 1.0 m ignores overhangs.
        "Grid/NormalsSegmentation": "false",
        "Grid/MaxGroundHeight": "0.15",
        "Grid/MaxObstacleHeight": "1.0",
        # Do not map the rover's own wheels as obstacles.
        "Grid/FootprintLength": "1.0",
        "Grid/FootprintWidth": "0.8",
        "Grid/FootprintHeight": "0.5",

        # Visual registration refined by ICP on the lidar cloud. This is the
        # reason both sensors are wired in: features give the match, the lidar
        # geometry sharpens it.
        "Reg/Strategy": "2",
        "RGBD/ProximityBySpace": "true",
        # rtabmap raises this from 0 to 1 by itself whenever scan_cloud and ICP
        # are both on. Set explicitly so the choice is visible and the startup
        # warning goes away.
        "RGBD/ProximityPathMaxNeighbors": "1",
        "RGBD/AngularUpdate": "0.05",
        "RGBD/LinearUpdate": "0.05",
        "RGBD/OptimizeMaxError": "0.3",
        "Rtabmap/DetectionRate": "1",
        # Gravity constraints against the default GTSAM optimiser, fed by the
        # IMU through the odometry node. Without this the map slowly tilts.
        "Optimizer/GravitySigma": "0.3",
        "Mem/NotLinkedNodesKept": "false",
    }

    remappings = [
        ("rgb/image", "/camera/image_raw"),
        ("rgb/camera_info", "/camera/camera_info"),
        ("depth/image", "/camera/depth_image"),
        ("rgbd_image", "/rgbd_image"),
        ("scan_cloud", "/lidar/points"),
        ("imu", "/imu/data"),
        # rgbd_odometry publishes on "odom" by default, and so does the
        # ros_gz_bridge for Gazebo ground truth. Two publishers interleaving on
        # one topic is silent corruption, so the estimate goes somewhere of its
        # own and /odom stays ground truth, as rover_gazebo documents it.
        ("odom", "/rtabmap/odom"),
    ]

    rviz_config = os.path.join(
        get_package_share_directory(PKG), "rviz", "rover_slam.rviz")

    nodes = [
        # Pack RGB, depth and CameraInfo into one RGBDImage so the odometry and
        # mapping nodes each subscribe once instead of synchronising three
        # topics twice over.
        Node(
            package="rtabmap_sync", executable="rgbd_sync", output="screen",
            parameters=[{"approx_sync": True,
                         "approx_sync_max_interval": 0.1,
                         "use_sim_time": True,
                         "qos": 1}],
            remappings=remappings),
    ]

    # odom -> base_footprint. Exactly one of these publishes it.
    if odom_source == "visual":
        nodes.append(Node(
            package="rtabmap_odom", executable="rgbd_odometry", output="screen",
            parameters=[odom_parameters, shared_parameters,
                        {"subscribe_rgbd": True}],
            remappings=remappings,
            arguments=["--ros-args", "--log-level", "warn"]))
    else:
        nodes.append(Node(
            package=PKG, executable="gt_odom_tf.py", name="gt_odom_tf",
            output="screen",
            parameters=[{"use_sim_time": True}],
            remappings=[("odom", "/odom")]))

    # On the visual path the mapper takes odometry from the /rtabmap/odom topic,
    # which carries OdomInfo and so tells it how much to trust each link. On the
    # ground-truth path there is no such topic, only TF, and naming odom_frame_id
    # is what switches the mapper to reading TF instead.
    odom_input = {} if odom_source == "visual" else {"odom_frame_id": "odom"}

    if localization:
        # Keep the existing map, do not extend it.
        nodes.append(Node(
            package="rtabmap_slam", executable="rtabmap", output="screen",
            parameters=[rtabmap_parameters, shared_parameters, odom_input,
                        {"Mem/IncrementalMemory": "false",
                         "Mem/InitWMWithAllNodes": "true"}],
            remappings=remappings))
    else:
        nodes.append(Node(
            package="rtabmap_slam", executable="rtabmap", output="screen",
            parameters=[rtabmap_parameters, shared_parameters, odom_input],
            remappings=remappings,
            arguments=["-d"] if delete_db else []))

    if arg("viz") == "true":
        nodes.append(Node(
            package="rtabmap_viz", executable="rtabmap_viz", output="screen",
            parameters=[rtabmap_parameters, shared_parameters, odom_input],
            remappings=remappings))

    if arg("rviz") == "true":
        nodes.append(Node(
            package="rviz2", executable="rviz2", output="screen",
            parameters=[{"use_sim_time": True}],
            arguments=["-d", rviz_config]))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "odom_source", default_value="visual",
            choices=["visual", "ground_truth"],
            description="'visual' runs rgbd_odometry; 'ground_truth' rebroadcasts "
                        "Gazebo's /odom as TF, which isolates mapping faults from "
                        "estimator faults"),
        DeclareLaunchArgument(
            "localization", default_value="false", choices=["true", "false"],
            description="localise against an existing database instead of mapping"),
        DeclareLaunchArgument(
            "database_path", default_value="~/.ros/rover_slam.db",
            description="RTAB-Map database file"),
        DeclareLaunchArgument(
            "delete_db_on_start", default_value="true", choices=["true", "false"],
            description="start each mapping run from an empty database"),
        DeclareLaunchArgument(
            "viz", default_value="false", choices=["true", "false"],
            description="launch rtabmap_viz, RTAB-Map's own inspector"),
        DeclareLaunchArgument(
            "rviz", default_value="false", choices=["true", "false"],
            description="launch RViz with the map, cloud and TF preloaded"),

        OpaqueFunction(function=launch_setup),
    ])
