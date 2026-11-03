# 3D navigation: design spec

## Approach

The rover senses in 3D, maps in **2.5D** (one ground height per cell), plans in 2D on that
surface, and checks its full 3D attitude (roll, pitch, per-wheel contact) for every candidate
motion. Overhangs, tunnels and multi-level terrain are out of scope: URC and IRC ground is a single
surface. Camera-based terrain classification and learned traversability are out of scope.

Goals, in priority order: never enter unsafe terrain, zero recoveries, minimum time to goal.

## Compute split

| Machine | Runs |
|---|---|
| Jetson Orin Nano 8 GB (JetPack 7.2) | ZED SDK depth, `depth_range_cut` node (range cut plus voxel downsample) |
| Intel NUC | Unitree L2 driver, odometry, RTAB-Map, terrain mapping, features, Nav2 |

In simulation everything runs on one machine, with `depth_range_cut` standing in for the Orin.

## Data flow

```
/lidar/points ──► self filter ──► terrain_mapping_node ◄── depth_range_cut ◄── /camera/depth_points
                                   (FastDEM, per-sensor noise)
                                   ├─► /terrain/local  (grid_map, odom, rolling)
                                   └─► /terrain/global (grid_map, map, coarse)
/terrain/local ──► terrain_features_node ──► /terrain/local_features
/terrain/local_features ──► TraversabilityLayer (local costmap) ──► MPPI + TerrainCritic
/terrain/global ──► terrain_features_node (global instance) ──► TraversabilityLayer (global costmap) ──► Smac 2D
/terrain/local_features ──► speed governor ──► /speed_limit
/terrain/local_features + /imu/data ──► safety monitor ──► collision monitor source, goal abort
```

## Packages

### `rover_terrain_mapping` (ament_cmake)

- `third_party/fastdem/`: vendored FastDEM (https://github.com/Ikhyeon-Cho/FastDEM, BSD-3,
  keep its LICENSE, pin the commit in a `VERSION` file). Upstream tests on Humble only, so Phase 0
  verifies it on Jazzy.
- `terrain_mapping_node`: subscribes to the lidar and the range-cut depth cloud. Each input has its
  own sensor model (lidar: near-constant range noise, RGB-D: noise growing with range squared).
  Gravity-aligned via the SBG or sim IMU. Publishes local and global elevation maps.
- `terrain_features_node`: computes feature layers from an elevation map (see below).
- `depth_range_cut`: crops depth points beyond `max_range` and voxel-downsamples.
- The backend sits behind an interface so the layer contract below survives a backend swap. The
  fallback backend is the prototype's `mrt_elevation_mapping`, extended with variance.

### `rover_traversability` (ament_cmake)

- `traversability_core` library: pure C++ geometry shared by every consumer (single source of
  truth). Wheel contact placement, plane fit, roll and pitch, stability margin, belly clearance,
  articulation, utilisation, disc dilation.
- `TraversabilityLayer`: Nav2 costmap plugin, ported from `mrt_traversability_layer` and extended.
- `TerrainCritic`: MPPI critic plugin (heading-aware, see cost model).
- `terrain_speed_governor`: publishes `/speed_limit`.
- `terrain_safety_monitor`: publishes lethal cells as a cloud for the collision monitor, aborts the
  goal on measured tip risk.
- `config/rover_physical.yaml`: the only source of physical parameters.

### `rover_nav2` (existing, config only)

- `config/nav2_params_sim_3d.yaml`, `config/nav2_params_3d.yaml`: the 3D stack.
- `rover_nav2/benchmark/`: benchmark runner, goal sets, scoring, report generation.
- `doc/3d_nav/`: these documents.

## Layer contract

Every elevation map carries `elevation`, `variance`, `observations`, `last_seen`.
Every feature map adds `slope`, `roughness`, `step`, `normal_x`, `normal_y`, `normal_z`,
`curvature`, `negative_obstacle`, `safe_yaw_mask`, `best_yaw`, `best_utilisation`.
Names are constants in one header (`rover_traversability/layer_names.hpp`), used by all nodes
and plugins. Feature maps may carry extra layers (currently `slope_std`).

Units and definitions (`terrain_features_node`, `rover_terrain_mapping/features.hpp`):

| Layer | Definition |
|---|---|
| `elevation`, `variance` | FastDEM Kalman estimate and its variance P (not FastDEM's sample spread of raw returns), m and m^2, gravity-aligned. Each scan feeds the mean of a cell's returns (FastDEM PATCH 3). |
| `observations` | points fused into the cell (FastDEM `n_points`) |
| `last_seen` | age in seconds of the newest point in the cell, relative to the map stamp (an age keeps millisecond precision in float, an absolute stamp would not) |
| `slope`, `normal_*`, `slope_std` | variance-weighted least-squares plane over a disc of radius `wheelbase / 2` (chassis scale). The disc must be at least 30 percent observed and its cells spread in both directions, so a single scan line yields NaN instead of an arbitrary tilt. `slope_std` propagates the fit covariance. |
| `roughness` | RMS residual to that plane, m |
| `step` | second highest minus second lowest elevation over a disc of radius `wheel_radius` (at least 1.5 cells), after subtracting the chassis-scale plane's gradient, m. A smooth ramp is not a step, and an abrupt jump keeps its height, less at most one cell of plane slope. One stray cell at either end is trimmed: any real feature under a wheel covers two or more cells, while untrimmed max minus min over about 37 cells has a noise floor near 4 sigma (6 to 8 cm at the sim lidar's 1.5 to 2 cm). Plain max minus min with fewer than 4 cells, raw heights where the plane fit is invalid. The trimmed step still has a noise floor of about 5 cm on flat ground (3 sigma, order statistics over the disc), so the published step is the larger of the trimmed step on a 3x3 median of the map and the raw trimmed step less `step_noise_floor` (0.05 m). The median keeps edges and features 3 cells wide, the floor term keeps tall features narrower than that (a 0.20 m stub 2 cells wide reads 0.15 m). Parameters `step_median` (true in the node) and `step_noise_floor`. |
| `curvature` | smallest over total eigenvalue of the plane window's 3D covariance |
| `negative_obstacle` | 1 where occlusion reasoning finds a hole: along a ray from the sensor, an unknown run that flat ground at the last observed height would have exposed (within the sensor's range and above its lowest beam, and clear of the terrain horizon), longer than both 0.3 m and twice the sensor's row spacing at that distance. Shadows behind higher terrain and the blind zone under the sensor stay plain unknown. |

## Cost model

The core idea is **utilisation**: for each limit, the fraction of the allowed amount that a pose
uses (0 is flat and benign, 1 is at the limit). A pose is lethal when any utilisation exceeds 1.
Soft cost comes from the worst utilisation, so no hand-tuned feature weights exist.

### Pose evaluation (shared by layer and critic)

For a pose (x, y, yaw):

1. Place the four wheel contact points from `wheelbase` and `track`. Read their heights
   (bilinear: the nearest cell alone understates a 10 degree tilt by about 0.8 degree). Use the
   risk-adjusted height `z + kappa * sigma` where it makes the check stricter (CVaR-style tail
   handling of map uncertainty). In practice: plane slopes are taken at the worst corner of
   their `kappa * sigma` box, the articulation difference and the steps are widened by `kappa`
   times their standard deviation, belly heights raised by `kappa * sigma`.
2. Fit the support plane. Get roll and pitch in the body frame.
3. **Stability margin**: the minimum, over the edges of the wheel-contact support polygon, of the
   tip-over angle of the CG about that edge. Exact in 3D, with the wheels body-fixed on the
   support plane: per edge, the angle between the edge-to-CG vector and the vertical, both
   projected on the plane perpendicular to the edge.
   `u_tip = (margin_flat - margin) / (margin_flat - stability_margin_min)`, where `margin_flat` is
   the margin on level ground. This is 0 on flat ground and 1 at the minimum allowed margin.
4. **Belly clearance**: highest terrain point inside the chassis polygon (wheel patches excluded),
   measured from the support plane. `u_clear = height / ground_clearance`. The belly is taken as
   the circumscribed disc of the rectangle `(wheelbase - 2 wheel_radius) x (track -
   wheel_width)`: larger than the true belly, so strict, and independent of yaw, which keeps the
   per-heading evaluation affordable.
5. **Articulation**: left and right rocker angles from the front and rear contact heights per side.
   `u_art = |rocker_left - rocker_right| / 2 / rocker_max`.
6. **Wheel step**: step layer at each contact. `u_step = max_i(step_i) / step_max`.
7. **Roughness**: `u_rough = roughness / roughness_max`, the largest roughness in the
   circumscribed disc of the wheel rectangle.
8. `u = max(u_tip, u_clear, u_art, u_step, u_rough)`. Lethal if `u > 1` or any contact cell is a
   negative obstacle.

### Cell cost (`TraversabilityLayer`)

- Evaluate the pose at `yaw_bins` headings per cell. Store which headings are safe in
  `safe_yaw_mask` and the best one in `best_yaw` (these are computed in `terrain_features_node`, the
  layer reads them).
- Cost from the best heading: `cost = max_soft_cost * best_utilisation^2`. Lethal when no heading
  is safe.
- Unknown cells: lethal inside the braking distance `v^2 / (2 a_brake) + margin`, otherwise
  `unknown_cost`.
- Worst value wins when several map cells land in one costmap cell.
- Verdicts are configuration space: each already places the whole rover at the cell. So the 3D
  stack gives Nav2 a point footprint, lethal cells are not dilated (`dilate_terrain_lethal:
  false`), and inflation only reaches one local cell. The braking band for unknown cells is still
  measured from the physical footprint in `rover_physical.yaml`. The prototype's inscribed-radius
  dilation stays available for the legacy cost model. (Changed 2026-11-02: footprint, inflation
  and dilation together counted the rover's width two or three times and closed a 2.5 m causeway
  to the planner.)
- Unknown cells under the rover's footprint take the plane the rover stands on (from TF), in the
  feature computation only. The wheels prove that ground carries the rover, and without it every
  pose check near the rover reaches into the blind zone under the body.

### Controller cost (`TerrainCritic`)

For every pose of every MPPI rollout, evaluate the pose at its actual yaw. Cost per point:
`w_terrain * u^2`, plus `collision_cost` if `u > 1`. Add `w_redirect * |change in travel
direction|` between consecutive points, since wheels need time to re-aim (`max_wheel_turn_rate`).
Add a direction-aware time term from the predicted pitch (uphill is slower). `PathAngleCritic` is
retuned or disabled so it does not force heading along the path.

### Speed and safety

- Speed governor: `v_max = v_nominal * (1 - alpha * u_ahead)`, floored at `v_min`, where
  `u_ahead` is the worst utilisation under the predicted footprint over `lookahead_time`.
- Safety monitor: lethal cells in the braking zone feed the collision monitor. Measured roll or
  pitch within `tip_margin` of the limit aborts the goal.
- Slip layer (Phase 5, only if wheel speed feedback exists): cells where wheel speed and EKF
  velocity disagree beyond `slip_threshold` get an added cost that decays over time.

## Drive

The rover has four independently steered and driven wheels. Nav2 drives it through a new
`swerve` mode in `rover_kinematics_node` that takes `vx`, `vy`, `wz` together. Per wheel at
(xi, yi): `vix = vx - wz * yi`, `viy = vy + wz * xi`, angle `atan2(viy, vix)` with the ±90 degree
flip (reverse the wheel speed instead of steering past the limit), speed `hypot(vix, viy)`, rate
limited by `max_wheel_turn_rate`. Existing modes stay. Until swerve lands, the 3D stack runs with
the current Ackermann configuration.

## Physical parameters

`rover_physical.yaml`. Values from the sim URDF are real for the sim. The rest are placeholders.

| Parameter | Value | Source |
|---|---|---|
| `wheelbase` | 0.744729 m | `rover_kinematics.yaml` |
| `track` | 0.436 m | `rover_kinematics.yaml` |
| `wheel_radius` | 0.14985 m | `rover_kinematics.yaml` |
| `footprint` | 1.1 x 0.9 m box | `nav2_params_sim.yaml` |
| `cg_height` | 0.45 m | PLACEHOLDER |
| `cg_offset_x`, `cg_offset_y` | 0.0 m | PLACEHOLDER |
| `mass` | 50 kg | PLACEHOLDER |
| `ground_clearance` | 0.25 m | PLACEHOLDER |
| `rocker_max` | 0.35 rad | PLACEHOLDER |
| `step_max` | 0.10 m | PLACEHOLDER (sim climbs 10 to 12 cm bars) |
| `stability_margin_min` | 0.17 rad | PLACEHOLDER |
| `roughness_max` | 0.05 m | PLACEHOLDER (prototype layer's limit). Added: the cost model needs it and the table lacked it. |
| `wheel_width` | 0.14 m | `rover_kinematics.yaml`. Added: belly disc. |

## Benchmark

Worlds: existing `marsyard`, `ledge`, `bars`, plus new `slopes` (15, 25, 35 degree ramps and a
sidehill), `crater_field`, `rock_field`, `urc_mix`, and the Gale Crater patch ported from the
prototype (`rover_gazebosim/models/mars_gale_crater_patch1`). Each world has a goal set and a
ground-truth heightmap export.

Per run: time to goal, success, recovery count, unsafe entries (ground-truth roll, pitch, or
footprint over terrain beyond the limits), path length, planner time, CPU load. Every
configuration runs each goal at least 5 times. The baseline is the current 2D stack.

Definitions used by `rover_nav2/benchmark/`: success needs Nav2 SUCCEEDED and the ground-truth
position within 0.5 m of the goal (Nav2 checks 0.15 m in its own drifting map frame). Roll and
pitch limits are the flat tip-over angles minus `stability_margin_min` (16.1 and 29.9 degrees
with the placeholder values). An unsafe entry is a violation lasting at least 0.2 s, roll and
pitch together counting once. Planner time is one `ComputePathToPose` call per run.
