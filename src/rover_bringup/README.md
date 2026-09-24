# rover_bringup

**Owner(s):** (unassigned)  
**Status:** launch files for the simulated stack

## Scope

- Launch files
- Config files
- bashfile / environment setup

## Notes

Top-level entry point that composes the other packages. Changes here affect everyone — announce
them after merge.

The launch files live here; configs, URDF, worlds, RViz layouts and nodes stay in the packages
that own them (`rover_gazebo`, `rover_ekf`, `rover_slam`, `rover_nav2`).

## Launch files

| File | Moved from | What it starts |
|---|---|---|
| `pipeline.launch.py` | — | everything below, together |
| `rover_sim.launch.py` | `rover_gazebo` | Gazebo, ros2_control, the ros_gz bridge. The only file that starts Gazebo |
| `ekf_rover_gazebo.launch.py` | `rover_ekf` | wheel encoder chain into robot_localization, against a running sim |
| `rtabmap.launch.py` | `rover_slam` | RTAB-Map SLAM, or localisation against a saved database, against a running sim |
| `navigation.launch.py` | `rover_nav2` | Nav2 planning, control, recovery |

The default world is `empty_world.sdf` (in `rover_gazebo/worlds/`), a 9 m walled box.

## Full pipeline

```bash
ros2 launch rover_bringup pipeline.launch.py
ros2 launch rover_bringup pipeline.launch.py world:=marsyard.sdf rviz:=true
ros2 launch rover_bringup pipeline.launch.py localization:=true   # against the saved RTAB-Map database
```

The simulator starts first, the EKF and RTAB-Map 8 s later, and Nav2 at 15 s.

Each transform has exactly one publisher:

| Transform | Publisher |
|---|---|
| `map -> odom` | RTAB-Map, mapping or with `localization:=true` |
| `odom -> base_footprint` | the EKF under `odom_source:=ekf` (default); otherwise the RTAB-Map odometry node for `visual` / `lidar` / `ground_truth`, and the EKF's TF is switched off |

## Running the pieces separately

Start the simulator first; nothing else starts Gazebo.

```bash
ros2 launch rover_bringup rover_sim.launch.py
ros2 launch rover_bringup ekf_rover_gazebo.launch.py
ros2 launch rover_bringup rtabmap.launch.py
ros2 launch rover_bringup navigation.launch.py sim:=true
```

Run by hand like this, both the EKF and `rtabmap.launch.py`'s default `odom_source:=ground_truth`
publish `odom -> base_footprint`. Pass `publish_tf:=false` to the EKF, or `odom_source:=ekf`
to RTAB-Map.

See [CONTRIBUTING.md](../../CONTRIBUTING.md) for the branch and PR workflow.
