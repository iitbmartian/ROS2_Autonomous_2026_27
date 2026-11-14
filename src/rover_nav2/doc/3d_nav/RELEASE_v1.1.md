# 3D navigation v1.1 (Ackermann), 2026-11-14

Terrain-aware 3D navigation for the rover in simulation, Ackermann drive. Ships as the 3D stack
next to the unchanged 2D stack:

    ros2 launch rover_bringup pipeline_3d.launch.py world:=marsyard.sdf

The 2D stack (`pipeline.launch.py`, `nav2_params_sim.yaml`) behaves as before. Nothing in v1.1
changes it, and its tests pass.

## What it does

- FastDEM 2.5D elevation maps, local (0.05 m, odom) and global (0.2 m, map), from the lidar and
  the range-cut depth cloud (`rover_terrain_mapping`).
- Terrain features (slope, roughness, step, negative obstacles) and the utilisation cost model of
  SPEC.md: a pose is lethal when any limit is used up, soft cost comes from the worst limit, no
  hand-tuned feature weights (`rover_traversability`).
- Both costmaps use the TraversabilityLayer, Smac Hybrid plans, MPPI follows. A speed governor
  slows the rover on hard terrain and a safety monitor aborts on measured tip risk.

## New in v1.1

- **Fine step in the global map** (`rover_terrain_mapping/fine_step.hpp`, launch arg
  `global_fine_step`, default true). At 0.2 m a rock smaller than a cell was averaged into one
  cell and trimmed as noise, so the planner sent a wheel onto 0.11 to 0.12 m rocks and the rover
  stuck (urc_mix far_south, rock_field far_right, marsyard behind_left). Each global cell now
  keeps the worst step the 0.05 m local map saw inside it. 7 hand-computed gtests.
- **Re-approach at the goal** (3D tree `navigate_to_pose_ackermann_3d.xml`, BT condition
  `NearGoalTooLong`). A car-like rover that parks beside the goal cannot slide sideways and saws
  back and forth. After 25 s within 1 m of the goal it backs up 1.5 m and comes in again.
- **Rocker coupling damping D 250** (`rover_gazebo`, branch `ram/suspension`). D 500 shook the
  body at 2 to 3 Hz and slowed the 2D stack 2 to 9 times.
- Speed governor diagnostics topic (`/terrain_speed_governor/diagnostics`) for bags.

## Results (simulation, ground-truth odometry, D 250, 6 worlds, 19 goals x 3 runs)

| stack | goals reached | unsafe entries | recoveries |
|---|---|---|---|
| 3D v1.1 | 49/57 | 0 | 46 |
| 3D before v1.1 | 44/57 | 0 | 68 |
| 2D baseline | 36/57 | 1 | 271 |

Phase 3 gate (`evidence/phase3_gate_ckpt_d250.txt`): unsafe entries 0 on every world against 1
for 2D. Every item passes on 4 of 6 worlds. The two remaining items were accepted by Ram on
2026-11-14:

- marsyard behind_left: reached 3/3 (was 0/3), but the back-up makes it 60 s against 25 s for 2D
  and counts as 3 recoveries.
- bars past_bars_4m: a 12 cm bar is over the 0.10 m step limit, so not climbing it is correct for
  the placeholder parameters.

Data: `~/.ros/rover_benchmark/2026-11-14_3d_gt_ckpt_d250`, `2026-11-13_3d_gt_plain_d250`,
`2026-11-13_2d_base_d250`. Per-goal table: `tools/compare_final.py` with those three dirs.

## Known limits

- slopes lane25_top (25 deg ramp) and sidehill_uphill (20 deg) sit at the tip limits by design and
  pass in 1/3 to 3/3 of runs depending on the approach line. Never unsafe.
- urc_mix far_south still needs up to 7 recoveries in about half of its runs.
- Swerve (v2) works but is 2 to 3 times slower than Ackermann.
- Physical parameters in `rover_physical.yaml` are placeholders until measured on the rover.
- Simulation only, ground-truth odometry. Real odometry and the real rover come next.

## Branches for other owners

`ram/suspension` (rover_gazebo coupling damping), the rover_gazebo lidar, worlds and swerve-mode
branches, and the rover_bringup ICP branch, all merged into `ram/3d_nav` locally (PROGRESS.md
lists them).
