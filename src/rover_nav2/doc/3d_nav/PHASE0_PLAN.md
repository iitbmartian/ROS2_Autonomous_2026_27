# Phase 0 plan: foundation

Status: **proposed, awaiting approval.** No code is written until Ram approves this document.

This plan covers every Phase 0 task in `PHASES.md`, the files each task adds or changes, the tests,
and the evidence for each gate item. Decisions already made by Ram are recorded in section 1 so
the plan can be read on its own.

## 1. Decisions already made

| Topic | Decision |
|---|---|
| Container | `ghcr.io/iitbmartian/rover-dev:jazzy` is the container. Its digest goes into `PROGRESS.md`. Nothing is installed into the image. Every dependency is declared in `package.xml`. Anything the image lacks is listed in `PROGRESS.md`. `docker/` changes are a separate, announced step. |
| grid_map_filters | Not used. Feature math stays in our own C++ per `SPEC.md`. |
| RViz | Runs on the host. `PROGRESS.md` records the `ROS_DOMAIN_ID` and RMW settings that match the container. |
| Layer with no input | Stays not current. That is the fail-safe. No option may report current without data. |
| Staleness | If the newest GridMap is older than a timeout parameter, the layer reports not current again. |
| Where the layer loads | Only in the new `nav2_params_sim_3d.yaml`. `nav2_params_sim.yaml` and `navigation.launch.py` stay untouched. |
| Gate wording | "Loads without errors" means "loads and goes current with FastDEM feeding it". |
| FastDEM features as a bridge | Approved as temporary. Replaced in Phase 2 by `terrain_features_node`. `SPEC.md` keeps our own feature definitions. |
| Slope units | Converted from degrees to radians inside our `terrain_mapping_node`, where FastDEM's layers are published. Everything downstream sees the `SPEC.md` contract. No separate adapter node. |
| Step layer name | `SPEC.md` and FastDEM both say `step`. The ported layer's default `step_layer` changes from `step_height` to `step`. No renaming adapter. |
| Definition differences | Recorded in `PROGRESS.md`, each marked when it makes the layer less strict than `SPEC.md` intends. No threshold is tuned against FastDEM's definitions. |

## 2. Facts found while planning

These go into the first `PROGRESS.md` entry.

1. **Container.** `ghcr.io/iitbmartian/rover-dev@sha256:4ba62ec8083613f5f2d601611c2c24190cf69524a5b53e091ffa1d3cf9de31c8`.
   It already has `grid_map_core`, `grid_map_ros`, `grid_map_msgs`, `nav2_costmap_2d`,
   `pluginlib`, `tf2_eigen`, `std_srvs`, `visualization_msgs`, `ament_cmake_gtest`, Eigen 3.4,
   yaml-cpp 0.8, spdlog 1.12, psutil 5.9.8, and the sim and SLAM runtime (`rtabmap_slam`,
   `rtabmap_odom`, `rtabmap_util`, `robot_localization`, `ros_gz_*`, `gz_ros2_control`).
   It lacks `grid_map_rviz_plugin`, which only the host RViz needs.
   There is no Dockerfile in `docker/`, only a README.
2. **DDS settings.** The host has `ROS_DOMAIN_ID=0`, `RMW_IMPLEMENTATION` unset (Fast DDS) and
   `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`. The container leaves both unset, which gives the same
   defaults. The container must run with `--network host --ipc host`. Without `--ipc host`, Fast
   DDS shared memory transport fails between host and container: discovery works, data does not
   arrive.
3. **FastDEM fetches code at build time.** Its core CMake pulls `nanoGrid` at `GIT_TAG main` and
   its tests pull googletest v1.14 through FetchContent. Both get replaced (section 4.3).
   Pinned commits: FastDEM `5cd3e25` (2026-09-04), nanoGrid `058a6a6`.
4. **The prototype's dilation test mirrors the algorithm in its own file** instead of calling the
   plugin code. Moving the dilation into `traversability_core` makes the test cover the real code.
5. **The prototype layer already has a staleness timeout** (`map_timeout`), but also an opt-out,
   `mark_stale_as_not_current`, which lets a stale map count as current. That opt-out is removed
   (section 4.2).
6. **Ground truth.** `/odom` comes from Gazebo's `OdometryPublisher` with `dimensions: 3` at
   100 Hz, so full position and attitude are available. Footprint-over-terrain checks need the
   ground-truth heightmaps from Phase 1.
7. **`pipeline.launch.py` hardcodes Nav2's params.** It includes `navigation.launch.py` with
   `sim:=true` only, so a 3D params file cannot be passed through it today (section 4.6).

## 3. FastDEM feature definitions against SPEC.md

Read from FastDEM `fastdem/src/feature_extraction.cpp` at `5cd3e25`. All three features come from
one PCA over the finite elevation cells in a disc of radius `analysis_radius` around each cell.

Phase 0 uses `analysis_radius: 0.10` (a 0.2 m disc, the same as the prototype's step window) on a
0.05 m map, so about 13 cells per disc. This is a window choice, not a threshold. No layer
threshold is tuned against these definitions.

| Feature | FastDEM definition | Prototype definition | Units after our node | Strictness against SPEC.md intent |
|---|---|---|---|---|
| slope | `acos(abs(n_z))` from the PCA normal over the disc, published in degrees | `acos(n_z)` from the normal over a 0.35 m radius | radians (converted in `terrain_mapping_node`) | **Not less strict.** On uniform slopes it reads the same. A 0.2 m disc picks up the local tilt of rock faces and noise, so it reads high rather than low. SPEC.md's pitch and roll come from the wheel-contact plane at wheelbase scale (0.74 m), which Phase 3 adds. |
| step | 95th minus 5th percentile height in the disc | max minus min over a 0.20 m square | metres | **LESS STRICT.** (a) The 0.2 m disc is smaller than the 0.30 m wheel diameter, so a rise spread over more than 0.2 m reads lower than the height the wheel climbs. (b) The percentile trim drops spikes covering under about 5 percent of the disc (one cell of 13), so a narrow sharp rock reads low. |
| roughness | `sqrt(smallest eigenvalue)`, the RMS residual perpendicular to the fitted plane | standard deviation of height about the local mean over 0.18 m | metres | **LESS STRICT.** The residual is measured over a 0.2 m disc, while SPEC.md applies `u_rough` under the whole footprint. Undulations with wavelength longer than the disc fit the local plane and read near zero. It is also lower than the prototype's value on any slope, since a tilted plane is not counted as roughness. |

`PROGRESS.md` copies this table and marks the bridge as replaced in Phase 2 by
`terrain_features_node`.

## 4. Tasks

### 4.1 `traversability_core` with the disc dilation

Adds `src/rover_traversability/` (ament_cmake):

- `package.xml`, `CMakeLists.txt`. Builds two targets: `traversability_core` (plain C++ library, no
  ROS headers) and the layer plugin library.
- `include/rover_traversability/core/disc_dilation.hpp`, `src/core/disc_dilation.cpp`.
  Class `DiscDilation`: `setRadius(radius_m, resolution)` builds the chord kernel,
  `apply(buffer, size_x, size_y, min_i, min_j, max_i, max_j)` dilates in place. The algorithm is
  the prototype's `buildDilationKernel` and `dilateHazards`, unchanged, with `costmap_` replaced by
  a raw buffer. The scratch vectors stay members so `apply` does no allocation in steady state.
- `include/rover_traversability/layer_names.hpp`. Constants for every layer in the `SPEC.md`
  contract (`elevation`, `variance`, `observations`, `last_seen`, `slope`, `roughness`, `step`,
  `normal_x`, `normal_y`, `normal_z`, `curvature`, `negative_obstacle`, `safe_yaw_mask`,
  `best_yaw`, `best_utilisation`). Header only, no dependencies, so `rover_terrain_mapping` uses it
  too.

Test `test/test_disc_dilation.cpp` (gtest, against the real `DiscDilation`):

- The prototype's 400 randomised trials against a brute-force disc reference, with update windows
  flush against every edge. Same seed (20260830).
- The prototype's disc-versus-square check: r = 10, cell (7, 7) inside, (8, 8) outside, (10, 0)
  inside.
- New hand-computed cases:
  - r = 1 around one lethal cell gives exactly the 5-cell plus shape.
  - r = 2 gives exactly 13 cells (`di^2 + dj^2 <= 4`).
  - `NO_INFORMATION` cells with no cost in range stay 255.
  - A higher cost wins over a lower one where two discs overlap.
  - r = 0 leaves the buffer unchanged.
  - Radius 0.12 m at 0.05 m resolution rounds to 2 cells.

### 4.2 Port of `mrt_traversability_layer`

Adds to `src/rover_traversability/`:

- `include/rover_traversability/traversability_layer.hpp`, `src/traversability_layer.cpp`. Source
  files only, from `2d-slam_mrt-jazzy/mrt_traversability_layer/`. Namespace `mrt_nav` becomes
  `rover_traversability`. The dilation calls `DiscDilation`. Cost math, bounds handling, frame
  handling and the worst-value rule stay as they are.
- `costmap_plugins.xml` registering `rover_traversability/TraversabilityLayer`.
- `README.md` adapted from the prototype README (cost value contract, dilation notes, known
  limitations).

Three intended changes from the prototype, each listed in `PROGRESS.md`:

1. `mark_stale_as_not_current` is removed. A map older than `map_timeout` always makes the layer
   not current. A missing map already did. Nothing else can make it current.
2. The default `step_layer` changes from `step_height` to `step`.
3. The default `map_topic` changes from `/elevation_map_filtered` to `/terrain/local_features`.

Not ported: `config/robot_body_filter.yaml` (Phase 2 self filter), `config/elevation_filter_chain.yaml`
(needs `grid_map_filters`, ruled out), `config/nav2_costmap_snippet.yaml` (replaced by
`nav2_params_sim_3d.yaml`).

The layer's thresholds (`max_slope`, `max_step`, `max_roughness`) keep the prototype defaults in
Phase 0. They are the prototype's cost model, which Phase 3 replaces with the utilisation model
reading `rover_physical.yaml`. See open question 2.

Test `test/test_traversability_layer.cpp` (gtest):

- **Plugin load.** pluginlib `ClassLoader<nav2_costmap_2d::Layer>` creates
  `rover_traversability/TraversabilityLayer`. This checks the plugin XML export.
- **Costs.** A `LayeredCostmap` (rolling, 0.05 m) with the layer and a 0.4 m square footprint
  (inscribed radius 0.2 m, 4 cells) is fed a synthetic GridMap. With the prototype defaults, the
  expected values are hand-computed:

  | Cell | Input | Expected cost |
  |---|---|---|
  | flat | slope 0, step 0, roughness 0 | 0 |
  | ramp | slope 0.2 rad, rest 0 | `floor(252 * 0.5 * (0.2/0.45)^2 / 1.0)` = 24 |
  | step | step 0.3 m | 254, and every cell within 4 cells of it is 254 |
  | unknown | NaN in all three | 200 |

- **Fail-safe.**
  - With no map received, `isCurrent()` is false.
  - After one fresh map it is true.
  - After the clock passes `map_timeout` with no new map, it is false again.

  The test uses a sim clock so it does not sleep.

### 4.3 `rover_terrain_mapping` with FastDEM vendored

Adds `src/rover_terrain_mapping/` (ament_cmake):

- `third_party/fastdem/`: FastDEM's `fastdem/` core directory at `5cd3e25`, with its LICENSE
  (BSD-3). FastDEM's own `ros2/` and `ros1/` wrappers are not vendored, since
  `terrain_mapping_node` replaces them.
- `third_party/nanogrid/`: nanoGrid at `058a6a6`, with its LICENSE.
- `third_party/VERSION`: both repository URLs and commits.
- `third_party/PATCHES.md`: every local change to vendored code, with the reason. Planned:
  1. Core CMake finds the vendored nanoGrid instead of FetchContent.
  2. Tests link the system gtest instead of FetchContent googletest.
  3. Any Jazzy or GCC 13 build fix found in the process.
- `CMakeLists.txt` builds FastDEM with `add_subdirectory(third_party/fastdem)` as a static
  library inside this package.
  - colcon stops descending at `rover_terrain_mapping/package.xml`, so the vendored
    `package.xml` files stay inert. There is no separate colcon package and nothing goes outside
    my scope.
  - FastDEM's 11 test sources are registered with `ament_add_gtest`, so they run under
    `colcon test`.
  - `examples/01_hello_mapping` is built and run once as a smoke test.

Test: FastDEM's own suite (`test_config`, `test_elevation_map`, `test_rasterization`,
`test_kalman_estimation`, `test_quantile_estimation`, `test_dual_layer`, `test_sensor_models`,
`test_postprocess`, `test_fastdem_integration`, `test_online_mode`, `test_map_io`), unchanged. If any
fails on Jazzy, `PROGRESS.md` reports which and why, with no test edited to pass.

### 4.4 `terrain_mapping_node` (Phase 0 scope)

Adds to `src/rover_terrain_mapping/`:

- `src/terrain_mapping_node.cpp`, `include/rover_terrain_mapping/terrain_mapping_node.hpp`.
  Phase 0 scope only:
  - lidar input on `/lidar/points`
  - FastDEM local mode in `odom`
  - FastDEM post-processing with feature extraction on.
- **Publishes:**
  - `/terrain/local` (`grid_map_msgs/GridMap`): `elevation` and `variance`
  - `/terrain/local_features`: `elevation`, `slope`, `step`, `roughness`, `normal_x`, `normal_y`,
    `normal_z`, `curvature`.
- **Contract conversion.** Layer names come from `layer_names.hpp`. FastDEM's slope is converted
  from degrees to radians here, and its `_normal_x` style names are mapped to the contract names.
  This conversion is the only adapter. It sits in one clearly named function,
  `toContractLayers()`, whose comment says it is the Phase 0 bridge, replaced by
  `terrain_features_node` in Phase 2.
- The header stamp is the newest integrated scan's stamp, so the layer's staleness check measures
  data age, not publish time.
- Not in Phase 0: depth input, per-sensor noise beyond FastDEM's lidar model, IMU gravity
  alignment, self filter, the global map, `observations`, `last_seen`. All are Phase 2.
- `config/terrain_mapping_sim.yaml`:
  - map 10 x 10 m at 0.05 m, `base_frame: base_footprint`, `map_frame: odom`
  - FastDEM's Kalman estimator and lidar sensor model at defaults
  - raycasting and inpainting at FastDEM defaults
  - `feature_extraction.analysis_radius: 0.10`.

Test `test/test_contract_layers.cpp` (gtest on `toContractLayers()`):

- 30 degrees in gives 0.523599 rad out, within 1e-6.
- 0 gives 0 and NaN stays NaN.
- Every contract name listed above is present and no FastDEM-only name is left.
- `step` and `roughness` pass through unchanged.

### 4.5 Physical parameters

Adds:

- `src/rover_traversability/config/rover_physical.yaml`:
  - **From the sim URDF:** `wheelbase` 0.744729, `track` 0.436, `wheel_radius` 0.14985 (all from
    `rover_kinematics.yaml`), and the 1.1 x 0.9 m `footprint` box (from `nav2_params_sim.yaml`).
  - **Marked `# PLACEHOLDER`:** `cg_height` 0.45, `cg_offset_x` 0.0, `cg_offset_y` 0.0, `mass` 50,
    `ground_clearance` 0.25, `rocker_max` 0.35, `step_max` 0.10, `stability_margin_min` 0.17.
  - Each value has a source comment.
- `include/rover_traversability/core/physical_params.hpp`, `src/core/physical_params.cpp`.
  - `loadPhysicalParams(path)` returns a struct plus the list of keys marked `# PLACEHOLDER`.
    yaml-cpp drops comments, so the marker is found by scanning the raw line of each key.
  - A missing or non-numeric key throws with the key name.
  - `warnPlaceholders(logger, result)` logs one warning naming every placeholder key.
- `rover_traversability/physical_params.py` (installed Python module, same rules) for the
  benchmark runner.

Which nodes load it in Phase 0: only the benchmark runner (unsafe-entry limits). The layer does
not, since its behaviour is unchanged. Phase 3 nodes call `warnPlaceholders` at startup.

Tests:

- `test/test_physical_params.cpp` (gtest):
  - A fixture file with known values and two placeholders gives those values and exactly those
    two keys.
  - A missing key throws.
  - A placeholder marker on a different line from its key is not attributed to the key.
  - The shipped `rover_physical.yaml` parses and has exactly 8 placeholder keys:
    `cg_height`, `cg_offset_x`, `cg_offset_y`, `mass`, `ground_clearance`, `rocker_max`,
    `step_max`, `stability_margin_min`.
- `test/test_physical_params.py` (pytest): the same cases for the Python loader.

### 4.6 Nav2 3D sim config and launch

Adds:

- `src/rover_nav2/config/nav2_params_sim_3d.yaml`. A copy of `nav2_params_sim.yaml` with one
  change: the local costmap's plugins become `["voxel_layer", "traversability_layer",
  "inflation_layer"]`.
  - The layer is configured with `plugin: rover_traversability/TraversabilityLayer`,
    `map_topic: /terrain/local_features` and `map_timeout: 1.0`.
  - Its thresholds are the prototype defaults, not tuned.
  - The header comment says how this file differs from the 2D one.
  - The global costmap is unchanged in Phase 0.
- `src/rover_bringup/launch/terrain_mapping.launch.py` (new): starts `terrain_mapping_node` with
  `terrain_mapping_sim.yaml` and `use_sim_time`.
- Selecting the 3D params in the full pipeline needs one of the two options in open question 1.
  Until that is decided, the gate check runs `rover_sim`, EKF and RTAB-Map through
  `pipeline.launch.py`. It replaces the Nav2 include by starting `navigation.launch.py
  sim:=true params_file:=<3d file>` by hand. `navigation.launch.py` already accepts `params_file`.

`setup.py` in `rover_nav2` already installs `config/*.yaml`, so no change is needed there for the
config.

### 4.7 Benchmark runner

Adds to `src/rover_nav2/rover_nav2/benchmark/`:

- `runner.py`: for each world, goal and repetition:
  1. Launches the full stack headless (`gui:=false`) as a subprocess with its own process group.
  2. Waits for the Nav2 lifecycle to be active and for `/map` and the TF chain.
  3. Calls `ComputePathToPose` once for the planner time.
  4. Sends `NavigateToPose`.
  5. Records until the result arrives or a per-goal timeout passes.
  6. Tears the stack down and checks no Gazebo process is left.

  A fresh stack per run means every run starts with an empty RTAB-Map database
  (`delete_db_on_start:=true`) and no memory of earlier runs.
- `metrics.py`: pure functions on recorded series, so they can be tested without ROS.

  | Metric | Source |
  |---|---|
  | success | action status SUCCEEDED and ground-truth distance to goal within `xy_goal_tolerance` |
  | time to goal | sim time from goal acceptance to result |
  | recoveries | `number_of_recoveries` from the last `NavigateToPose` feedback |
  | unsafe entries | rising edges of ground-truth roll or pitch beyond the limit, after a 0.2 s debounce |
  | path length | 3D distance integrated over ground-truth `/odom` |
  | planner time | `planning_time` from the step 3 call. Mid-run replans are not timed, and the report says so. |
  | CPU load | psutil over the launched process tree, mean and peak percent, sampled at 2 Hz |

  Unsafe limits come from `rover_physical.yaml` as the tip-over angle minus
  `stability_margin_min`:
  - roll: `atan((track/2) / cg_height) - 0.17` = 25.85 - 9.74 = **16.1 degrees**
  - pitch: `atan((wheelbase/2) / cg_height) - 0.17` = 39.61 - 9.74 = **29.9 degrees**.

  Both come from placeholder values, and the runner logs the placeholder warning.
  Footprint-over-terrain entries are added in Phase 1 once ground-truth heightmaps exist.
- `report.py`: writes `runs.csv` (one row per run with all metrics, world, goal id, repetition,
  config name, git commit, image digest) and `summary.md` (per world and goal: success rate, mean
  and standard deviation of each metric, plus the notes on what is not yet measured).
- `goals/marsyard.yaml`, `goals/ledge.yaml`, `goals/bars.yaml`.
  - Each holds two to three goals in the `map` frame, picked to cross that world's feature: the
    ledge along +x, the bars across the path, and mixed terrain on marsyard.
  - Each goal has a timeout.
  - Spawn pose matches the world's default.
- `__main__.py` and a console entry point: `ros2 run rover_nav2 benchmark --worlds marsyard ledge
  bars --runs 5 --config 2d [--params-file ...] [--out DIR]`.
- Results are written under `~/.ros/rover_benchmark/<date>_<config>/`, outside the repository.
  `PROGRESS.md` quotes the summary and names the directory.

Changes: `src/rover_nav2/setup.py` (packages, console entry point, `goals/*.yaml` data files) and
`src/rover_nav2/package.xml` (`nav2_msgs`, `nav_msgs`, `python3-psutil`, `python3-yaml`,
`rover_traversability` for the parameter file).

Test `src/rover_nav2/test/test_benchmark_metrics.py` (pytest) on synthetic series:

- A straight line of 10 m gives a path length of 10.000 m.
- A 3-4-5 triangle path gives 12 m.
- A roll trace crossing the limit twice gives 2 entries.
- A trace that only touches the limit within the debounce time gives 0.
- Roll and pitch both crossing in the same window count as one entry.
- Success needs both the action status and the distance check.

### 4.8 Baseline runs

No new files beyond results and `PROGRESS.md`.

- Command: `ros2 run rover_nav2 benchmark --worlds marsyard ledge bars --runs 5 --config 2d`.
  The 2D config is the current, unchanged `nav2_params_sim.yaml`.
- Size: about 7 goals x 5 runs, roughly 2 to 3 hours headless.
- Failures, timeouts and crashed runs are kept in the CSV with a reason, never rerun silently.

### 4.9 PROGRESS.md

Adds `src/rover_nav2/doc/3d_nav/PROGRESS.md` with one entry per session. The first entry carries:

- the facts in section 2
- the definitions table in section 3
- the bridge note (replaced by `terrain_features_node` in Phase 2)
- the layer changes in 4.2
- the container digest and missing packages
- the DDS settings
- the gate evidence below.

## 5. Gate evidence

| Gate item | How it is proven |
|---|---|
| `colcon build` of the new packages succeeds in the container, `colcon test` passes | `docker run --rm --network host --ipc host -v $PWD:/ws -w /ws <digest> bash -lc "colcon build --packages-up-to rover_traversability rover_terrain_mapping rover_nav2 && colcon test --packages-select rover_traversability rover_terrain_mapping rover_nav2 && colcon test-result --verbose"`. The full output is saved under `~/.ros/rover_benchmark/phase0_logs/`, and `PROGRESS.md` quotes the test-result summary with the command. |
| The ported layer loads and goes current in Nav2 sim | Sim on `marsyard` with `terrain_mapping.launch.py` and the 3D params. Evidence: (a) `local_costmap` log lines for the plugin load and "subscribed to /terrain/local_features"; (b) Nav2 lifecycle reaching active; (c) the costmap going current, shown by the controller accepting a goal and the goal reaching SUCCEEDED; (d) `grep -c ERROR` over the saved launch log equal to 0, any WARN lines listed; (e) host RViz screenshot of the local costmap with graded cells; (f) the fail-safe in sim: stopping `terrain_mapping_node` makes the controller stop with a costmap timeout within `map_timeout` plus `costmap_update_timeout`, with the log line quoted. |
| A FastDEM elevation map from `/lidar/points` is visible in RViz in sim | Host RViz (`grid_map_rviz_plugin`, same `ROS_DOMAIN_ID` and RMW) showing `/terrain/local` coloured by `elevation` on `marsyard`, plus a screenshot of `slope` from `/terrain/local_features`. `ros2 topic hz /terrain/local` output quoted. Bag `phase0_terrain_marsyard` with `/lidar/points`, `/tf`, `/terrain/local`, `/terrain/local_features`, 60 s. |
| Baseline CSV and summary exist for all three worlds, 5 runs per goal | The 4.8 command. `runs.csv` and `summary.md` in the results directory. `PROGRESS.md` quotes the summary table, the run count per goal (5 each), and the unmeasured items (footprint-over-terrain, replan time). |

## 6. Commit order

Each commit builds and tests in the container before the next one starts.

1. `docs(3d_nav)`: add `AGENT.md`, `SPEC.md`, `PHASES.md`, `PHASE0_PLAN.md` (currently untracked).
2. `feat(rover_traversability)`: package skeleton, `DiscDilation`, `layer_names.hpp`, dilation
   tests.
3. `feat(rover_traversability)`: port the traversability layer, with the staleness and default
   changes and the layer tests.
4. `feat(rover_traversability)`: `rover_physical.yaml`, C++ and Python loaders, tests.
5. `feat(rover_terrain_mapping)`: vendor FastDEM and nanoGrid with `VERSION` and `PATCHES.md`,
   FastDEM tests running on Jazzy.
6. `feat(rover_terrain_mapping)`: `terrain_mapping_node`, sim config, contract layer test.
7. `feat(rover_bringup)`: `terrain_mapping.launch.py`.
8. `feat(rover_nav2)`: `nav2_params_sim_3d.yaml`.
9. `feat(rover_nav2)`: benchmark runner, goal sets, metric tests.
10. `docs(3d_nav)`: `PROGRESS.md` with gate evidence and baseline summary.

The modified Unitree binaries already in the working tree are not mine and stay out of every
commit.

## 7. Open questions

1. **Passing the 3D params into the full pipeline.**
   - **Recommended:** a separate branch `ram/rover_bringup-nav-params-arg` from `main` that adds a
     `nav_params_file` argument to `pipeline.launch.py`, with the default unchanged.
   - **Alternative:** a new `pipeline_3d.launch.py` that duplicates about 100 lines of the
     existing pipeline.

   Either way, Phase 0 can proceed with the manual workaround in 4.6.
2. **Layer thresholds against the ground rules.** `AGENT.md` says code reads physical values only
   from `rover_physical.yaml`. The ported layer's `max_slope`, `max_step` and `max_roughness` are
   ROS parameters. I read them as the prototype's cost-model thresholds rather than physical
   parameters, kept only until the Phase 3 utilisation model replaces them.
   - Note that `max_step` (0.15) differs from `step_max` in `rover_physical.yaml` (0.10).
   - Is keeping them as layer parameters through Phase 0 to 2 acceptable?
3. **Baseline scope.** Is it acceptable that the Phase 0 baseline counts only roll and pitch unsafe
   entries until the Phase 1 heightmaps exist?
