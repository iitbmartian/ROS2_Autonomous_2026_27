# rover_slam

**Owner(s):** Sohan (VO)  
**Status:** RTAB-Map running against the Gazebo simulation

## Scope

- 3D SLAM
- RTAB-Map
- 3D → 2D map squishing (for 2D costmap consumption by Nav2)
- Loop closures

## Run it

Simulator and SLAM together:

```bash
ros2 launch rover_slam slam_sim.launch.py rviz:=true
```

then drive it, in another terminal:

```bash
ros2 run rover_gazebo teleop_rover.py
```

To restart SLAM without restarting Gazebo, run the two halves separately:

```bash
ros2 launch rover_gazebo rover_sim.launch.py
ros2 launch rover_slam rtabmap.launch.py rviz:=true
```

## Arguments

| Argument | Default | Meaning |
|---|---|---|
| `odom_source` | `visual` | `visual` runs RTAB-Map's visual odometry; `ground_truth` takes the simulator's pose instead |
| `localization` | `false` | localise against an existing database rather than extending it |
| `database_path` | `~/.ros/rover_slam.db` | where the map is stored |
| `delete_db_on_start` | `true` | start each mapping run from an empty database |
| `rviz` | `false` | RViz, with the map, cloud and odometry preloaded |
| `viz` | `false` | `rtabmap_viz`, RTAB-Map's own inspector |

`slam_sim.launch.py` also takes `world`, `sim`, `gui` and `teleop`, and passes them to
`rover_gazebo`.

## What it puts on the graph

| Topic | Type | Meaning |
|---|---|---|
| `/map` | `nav_msgs/OccupancyGrid` | the 2D grid, for Nav2 |
| `/rtabmap/cloud_map` | `sensor_msgs/PointCloud2` | the assembled 3D map |
| `/rtabmap/odom` | `nav_msgs/Odometry` | visual odometry |
| `/rtabmap/info` | `rtabmap_msgs/Info` | loop closure IDs and timing |
| `/rgbd_image` | `rtabmap_msgs/RGBDImage` | RGB, depth and intrinsics, synchronised |

TF: `map` → `odom` from the mapper, `odom` → `base_footprint` from whichever odometry
source is selected. `rover_gazebo` publishes everything below `base_footprint`.

Visual odometry publishes on `/rtabmap/odom`, **not** `/odom`. `/odom` stays what
`rover_gazebo` documents it as, ground truth straight from the simulator.

## Inputs

Everything comes from `rover_gazebo`'s `ros_gz_bridge`: `/camera/image_raw`,
`/camera/depth_image`, `/camera/camera_info`, `/lidar/points` and `/imu/data`.

## Read this before trusting it

The launch has been verified to start clean and to load every parameter it sets, and the
camera frame fix it depends on is confirmed present in the processed model. The mapping
itself, loop closure and the occupancy grid have **not** been driven end to end in Gazebo
here. `doc/RTABMAP_PIPELINE.md` lists exactly what was checked and what was not, and how to
run the rest.

## How it works

`doc/RTABMAP_PIPELINE.md` covers the design, the frame bug this had to fix in
`rover_gazebo`, every parameter choice and why, and what to check when the map looks wrong.

See [CONTRIBUTING.md](../../CONTRIBUTING.md) for the branch and PR workflow.
