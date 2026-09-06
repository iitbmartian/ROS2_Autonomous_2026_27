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

`W`/`S` drive, `A`/`D` turn, `1`/`2`/`3` pick Ackermann, crab or spot turn, space stops.
Keep that terminal focused or the keys go nowhere. The map only grows while the rover
moves: watch `WM=` climb in the SLAM log.

Pick the odometry source to match the world, because all three worlds are a bare ground
plane plus at most two boxes:

| World | Works with |
|---|---|
| `flat.sdf` | `ground_truth` only. No texture for vision, no geometry for ICP |
| `bars.sdf` | `ground_truth` or `lidar` |
| `ledge.sdf` | `ground_truth` or `lidar` |

`lidar` is the honest test, since it uses a real sensor rather than the simulator's answer:

```bash
ros2 launch rover_slam slam_sim.launch.py world:=bars.sdf odom_source:=lidar rviz:=true
```

Shut down with Ctrl-C and let it finish. A leftover `rtabmap` process fights the next run
and makes the clock jump backwards. If a run behaves oddly, clear it with
`pkill -f rtabmap; pkill -f "gz sim"`.

To restart SLAM without restarting Gazebo, run the two halves separately:

```bash
ros2 launch rover_gazebo rover_sim.launch.py
ros2 launch rover_slam rtabmap.launch.py rviz:=true
```

## Arguments

| Argument | Default | Meaning |
|---|---|---|
| `odom_source` | `ground_truth` | `ground_truth` takes the simulator's pose, `lidar` runs ICP odometry on the 3D lidar, `visual` runs RTAB-Map's visual odometry |
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

Mapping has been driven end to end in Gazebo on `bars.sdf` with `odom_source:=lidar`.
Odometry publishes at 4.86 Hz against the 5 Hz lidar, the full `map` to `odom` to
`base_footprint` chain resolves, and driving the rover grew the map to 31 nodes and a
786x1207 occupancy grid at 5 cm on `/map`, with no errors.

Still unverified: **loop closure**, which needs a circuit driven back to its start, and
**localization mode** against a saved database. `visual` odometry has never initialised in
any world here, because none of them has enough texture; it is kept for real camera data
and for a textured world. `doc/RTABMAP_PIPELINE.md` section 7 has the detail.

## How it works

`doc/RTABMAP_PIPELINE.md` covers the design, the frame bug this had to fix in
`rover_gazebo`, every parameter choice and why, and what to check when the map looks wrong.

See [CONTRIBUTING.md](../../CONTRIBUTING.md) for the branch and PR workflow.
