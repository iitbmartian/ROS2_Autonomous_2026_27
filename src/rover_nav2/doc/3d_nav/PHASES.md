# 3D navigation: phases

Work the phases in order, without stopping between them. A phase is done when every gate item is
shown with evidence in `PROGRESS.md`. Items marked **(other owner)** go on a separate local branch
cut from `main`, committed but not pushed, as described in `AGENT.md`.

## Phase 0: foundation

Tasks:
- Create `src/rover_traversability/` (ament_cmake). Port `mrt_traversability_layer` from
  https://github.com/ramsinghal08/3d-nav (`2d-slam_mrt-jazzy/mrt_traversability_layer/`): sources,
  header, plugin XML, tests. Port source files only, the prototype repo contains committed build,
  install and log folders. Rename to the new package. Behaviour unchanged for now.
- Create `traversability_core` with the disc dilation moved into it, keeping its tests.
- Create `src/rover_terrain_mapping/` (ament_cmake). Vendor FastDEM at a pinned commit. Build it on
  Jazzy and run its own tests or example.
- Create `config/rover_physical.yaml` with the values in `SPEC.md` and the startup warning for
  placeholders.
- Create the benchmark runner in `src/rover_nav2/rover_nav2/benchmark/`: sends goal sets through
  the Nav2 action interface, records the metrics in `SPEC.md` from ground truth, writes a CSV and a
  Markdown summary per run.
- Run the baseline: current 2D stack on `marsyard`, `ledge`, `bars`.
- Create `PROGRESS.md`.

Gate:
- `colcon build` of the new packages succeeds in the `docker/` container, `colcon test` passes.
- The ported layer loads as a costmap plugin in Nav2 sim without errors.
- A FastDEM elevation map from `/lidar/points` is visible in RViz in sim.
- Baseline CSV and summary exist for all three worlds, 5 runs per goal.

## Phase 1: simulation fidelity

Tasks:
- **(other owner: `rover_gazebo`)** Add an L2-like lidar option: near-hemispherical field of view
  (vertical about -6 to 90 degrees about the sensor), a xacro argument for mount tilt, range noise.
  Default model stays unchanged.
- **(other owner: `rover_gazebo`)** Worlds `slopes`, `crater_field`, `rock_field`, `urc_mix`, and
  the Gale Crater patch.
- Ground-truth heightmap export per world (script in `rover_nav2/benchmark/`).
- Goal sets for every world.

Gate:
- With the L2-like lidar, the elevation map has ground cells within 1.0 m of the footprint edge.
- The 3D additions cost at most 10 percent of the base pipeline's real-time factor (full 3D
  stack against the 2D pipeline with the same sensors). Changed by Ram on 2026-11-01 from "at
  least 0.7 with the full sensor set": other owners' Python nodes cap the base pipeline at about
  0.55, and their load is raised with those owners.
- Baseline rerun on all worlds.

## Phase 2: terrain mapping

Tasks:
- `terrain_mapping_node` with lidar and depth inputs, per-sensor noise models, gravity alignment,
  self filter, local and global maps, the layer contract.
- `depth_range_cut` node.
- `terrain_features_node`: slope, roughness, step, normals, curvature, negative obstacle (occlusion
  reasoning behind rims), uncertainty propagation.

Gate:
- Elevation RMSE against ground truth, reported per world, within 5 cm inside 6 m of the rover.
- Every crater wider than the wheel track is marked `negative_obstacle` before the footprint
  reaches it, on `crater_field`.
- Map update latency and CPU load reported on a NUC-class machine.

## Phase 3: traversability with the current drive

Tasks:
- `traversability_core`: pose evaluation from `SPEC.md` (stability margin, clearance,
  articulation, step, roughness, risk-adjusted heights) with gtest cases computed by hand.
- `terrain_features_node` fills `safe_yaw_mask`, `best_yaw`, `best_utilisation`.
- `TraversabilityLayer` v2 (cell cost, distance-aware unknowns, dilation).
- `nav2_params_sim_3d.yaml` and `navigation_3d.launch.py` with the existing Ackermann setup.

Gate:
- Fewer unsafe entries than baseline on every world, and zero on `slopes` and `crater_field`.
- Recoveries no worse than baseline. Time to goal within 10 percent of baseline where baseline
  succeeds.

## Phase 4: swerve

Tasks:
- **(other owner: `rover_gazebo`)** `swerve` mode in `rover_kinematics_node` per `SPEC.md`.
- `TerrainCritic` with heading-aware cost, redirect cost and direction-aware time term.
- MPPI `Omni` motion model, Smac 2D, `PathAngleCritic` retuned, velocity smoother and collision
  monitor with `vy`.

Gate:
- Time to goal lower than Phase 3 on `slopes`, `rock_field` and `urc_mix`.
- Unsafe entries and recoveries no worse than Phase 3.
- A sidehill crossing on `slopes` that Phase 3 refuses is completed facing uphill.

## Phase 5: speed and safety

Tasks:
- `terrain_speed_governor`, `terrain_safety_monitor`, collision monitor wiring.
- Slip layer, only if wheel speed feedback is available. Ask before starting it.

Gate:
- Zero unsafe entries across the full benchmark.
- Time to goal no worse than Phase 4.

## Phase 6: global terrain memory

Tasks:
- Global map in `map`, handling RTAB-Map loop closures (re-stitch submaps or coarse resolution,
  propose which with evidence).
- Long multi-waypoint goal sets (100 to 500 m) in `urc_mix`.

Gate:
- Multi-waypoint runs of at least 100 m complete with zero unsafe entries.
- Global map alignment error after loop closure reported.

## Phase 7: hardware

Tasks:
- Orin and NUC bring-up, clock sync, depth cloud bandwidth check.
- Real L2 mount calibration, `rover_physical.yaml` filled with measured values.
- Bag recording, offline tuning, field slope tests.

Gate: agreed with Ram when Phase 6 closes.
