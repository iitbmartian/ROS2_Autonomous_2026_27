# rover_terrain_mapping

2.5D terrain mapping for the rover's 3D navigation stack (`rover_nav2/doc/3d_nav/SPEC.md`).

```
/lidar/points ──────────────────────┐
/camera/depth_points ─► depth_range_cut ─► /terrain/depth_points ─┤
                                                                  ▼
                                                    terrain_mapping_node (FastDEM)
                                                      ├─► /terrain/local   (odom, rolling)
                                                      └─► /terrain/global  (map, coarse)
/terrain/local  ─► terrain_features_node ─► /terrain/local_features
/terrain/global ─► terrain_features_node ─► /terrain/global_features
```

```bash
ros2 launch rover_bringup terrain_mapping.launch.py            # all of the above, sim
ros2 launch rover_bringup pipeline_3d.launch.py world:=slopes.sdf   # with Gazebo, SLAM, Nav2
```

## Nodes

| Node | What it does |
|---|---|
| `terrain_mapping_node` | Fuses named point-cloud inputs into FastDEM elevation maps. Each input has its own sensor model (lidar: near-constant range noise, RGB-D: noise growing with range squared) and range limits. Removes the rover's own points with a body box from `rover_physical.yaml`. Roll and pitch from TF or the IMU (`gravity_source`, use `imu` with a planar EKF). Publishes `elevation`, `variance`, `observations`, `last_seen` (an age in seconds). |
| `terrain_features_node` | Feature layers of the SPEC.md contract: slope and normals from a chassis-scale plane fit, roughness, step over a wheel-scale window, curvature, negative obstacles by occlusion reasoning, and the per-heading layers `safe_yaw_mask`, `best_yaw`, `best_utilisation` from `traversability_core`'s pose evaluation. |
| `depth_range_cut_node` | Crops the depth cloud beyond `max_range` and voxel-downsamples it. Runs on the Orin on the rover, stands in for it in sim. |

## Vendored FastDEM

`third_party/` holds FastDEM (core library only) and nanoGrid at pinned commits
(`third_party/VERSION`), built statically inside this package. Local changes are listed in
`third_party/PATCHES.md`. FastDEM's own tests run under `colcon test`.

grid_map_core is never linked here: its CMake adds Eigen plugin definitions to every target in
the directory, which break FastDEM's Eigen code. GridMap messages are converted by
`rover_traversability/grid_map_msg.hpp` instead.

## Tests and tools

`colcon test --packages-select rover_terrain_mapping`: FastDEM's suite, the contract layers,
feature math (hand-computed planes, steps, roughness, negative-obstacle cases), gravity
alignment, the depth range cut.

`build/rover_terrain_mapping/bench_features [repeats] [yaw_bins] [threads]` reports the process
CPU time of one fully observed 10 x 10 m feature map, stage by stage.
