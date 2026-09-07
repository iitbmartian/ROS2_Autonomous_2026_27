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
    /cloud_map           sensor_msgs/PointCloud2  the assembled 3D map
    /mapData             rtabmap_msgs/MapData     the pose graph, for rtabmap_viz
    /info                rtabmap_msgs/Info        loop closures, for rtabmap_viz
    /rtabmap/odom        nav_msgs/Odometry        the estimate, visual or lidar
    TF                   map -> odom -> base_footprint

These names have no /rtabmap prefix on purpose. RTAB-Map's topics sit under the
node's namespace, and upstream rtabmap_launch sets that namespace to 'rtabmap',
which is why its documentation says /rtabmap/map. This file sets no namespace,
so that /map reaches Nav2 where Nav2 looks for it, and the rest follow.
/rtabmap/odom is the one exception, and it is an explicit remap.

Pick the odometry source to match the world. The rover_gazebo worlds are a bare
ground plane plus at most two boxes, which is why the default is ground truth:

    flat.sdf            ground_truth only. No texture for vision, no geometry for ICP.
    bars.sdf            ground_truth or lidar.
    ledge.sdf           ground_truth or lidar.
    husarion_world.sdf  ground_truth only. A grey plane with a floor decal, so
                        flat.sdf in substance and mapped the same way.

'visual' needs a textured scene and no world provides one yet. It stays here for
real camera data and for a world with texture in it.

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
        "Icp/Iterations": "10",
        "Icp/Epsilon": "0.001",
        "Icp/Strategy": "1",
        "Icp/OutlierRatio": "0.7",
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
    # Lidar odometry, odom_source:=lidar.
    #
    # icp_odometry registers scan geometry, so unlike rgbd_odometry it does not
    # care whether the scene has visual texture. That is the difference that
    # matters in this simulation: every world in rover_gazebo is a bare ground
    # plane plus at most two boxes, so rgbd_odometry only ever extracts a
    # handful of features and never reaches the 15 it needs to initialise.
    #
    # ICP needs geometry to bite on, though, and a perfectly flat plane
    # constrains nothing in x, y or yaw. On flat.sdf this drifts freely; use
    # bars.sdf or ledge.sdf, or odom_source:=ground_truth.
    # ------------------------------------------------------------------
    icp_odom_parameters = {
        "odom_frame_id": "odom",
        "publish_tf": True,
        "wait_imu_to_init": True,
        # The lidar is declared at 5 Hz in rover_sensors.xacro. Naming the rate
        # lets icp_odometry warn about dropped scans instead of silently
        # integrating over a gap.
        "expected_update_rate": 6.0,
        # 360 horizontal samples x 16 rings. icp_odometry infers this and warns
        # when it is unset, so state it and keep the log clean.
        "scan_cloud_max_points": 5760,
        "Odom/ScanKeyFrameThr": "0.4",
        "OdomF2M/ScanSubtractRadius": "0.1",   # match Icp/VoxelSize
        "OdomF2M/ScanMaxSize": "15000",
        "OdomF2M/BundleAdjustment": "false",
        # 360x16 samples is a sparse cloud, so demand little overlap per match.
        "Icp/CorrespondenceRatio": "0.01",
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
        # The depth images are 32-bit float. RTAB-Map's default .rvl compression
        # only handles 16-bit, so it falls back to .png and warns once per run.
        # Saying .png outright keeps full float depth and drops the warning.
        "Mem/DepthCompressionFormat": ".png",
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
    elif odom_source == "lidar":
        nodes.append(Node(
            package="rtabmap_odom", executable="icp_odometry", output="screen",
            parameters=[shared_parameters, icp_odom_parameters],
            remappings=remappings,
            arguments=["--ros-args", "--log-level", "warn"]))
    else:
        nodes.append(Node(
            package=PKG, executable="gt_odom_tf.py", name="gt_odom_tf",
            output="screen",
            parameters=[{"use_sim_time": True}],
            remappings=[("odom", "/odom")]))

    # The two estimator paths publish a real odometry topic on /rtabmap/odom, so
    # the mapper subscribes to it and gets the per-link confidence with it. The
    # ground-truth path only rebroadcasts TF and has no such topic, and naming
    # odom_frame_id is what switches the mapper over to reading TF instead.
    odom_input = ({"odom_frame_id": "odom"}
                  if odom_source == "ground_truth" else {})

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
        # RTAB-Map's own inspector: the RGB and depth feeds side by side, the
        # assembled cloud in the 3D view, the pose graph, and the loop closure
        # counters. It reads /info and /mapData from the mapping node, both of
        # which it already subscribes to under those names.
        #
        # It gets its own parameter dict rather than the mapper's. The mapper's
        # carries database_path, the whole Grid/ block and Reg/Strategy, none of
        # which the viewer acts on, and a second node holding database_path
        # reads as if it were also writing the database. What the viewer does
        # need is the same set of input subscriptions, so those are repeated.
        #
        # subscribe_odom_info would add the feature and inlier overlay on the
        # camera image, and it is left off on purpose. It is unavailable in the
        # combination used here for the same reason it is on the mapper, there
        # being no rgbdScan3dInfo sync, and under the default
        # odom_source:=ground_truth nothing publishes odom_info at all, so
        # asking for it would leave the window waiting on a topic that never
        # arrives.
        viz_parameters = {
            "subscribe_rgbd": True,
            "subscribe_scan_cloud": True,
            "subscribe_depth": False,
            "subscribe_rgb": False,
            "subscribe_odom_info": False,
        }
        # One "Could not get odometry pose from TF for stamp N, aborting" is
        # normal on startup and is not worth chasing. The viewer is the last
        # node up and its first synchronised frame is stamped about 10 ms ahead
        # of the newest TF its listener has collected. Raising wait_for_transform
        # does not help, because the buffer is empty rather than late. The
        # following frame lands and it does not recur.
        nodes.append(Node(
            package="rtabmap_viz", executable="rtabmap_viz", output="screen",
            parameters=[viz_parameters, shared_parameters, odom_input],
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
            "odom_source", default_value="ground_truth",
            choices=["visual", "lidar", "ground_truth"],
            description="'ground_truth' rebroadcasts Gazebo's /odom as TF and is "
                        "the only source that works on flat.sdf; 'lidar' runs "
                        "icp_odometry, which needs geometry so use bars.sdf or "
                        "ledge.sdf; 'visual' runs rgbd_odometry and needs a "
                        "textured scene, which no world currently provides"),
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
            "viz", default_value="true", choices=["true", "false"],
            description="launch rtabmap_viz, RTAB-Map's own inspector: camera "
                        "feeds, cloud, pose graph, loop closures. viz:=false "
                        "for a headless run"),
        DeclareLaunchArgument(
            "rviz", default_value="false", choices=["true", "false"],
            description="launch RViz with the map, cloud and TF preloaded"),

        OpaqueFunction(function=launch_setup),
    ])
