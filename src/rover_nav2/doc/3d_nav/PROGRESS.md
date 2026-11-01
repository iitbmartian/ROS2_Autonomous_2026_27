# 3D navigation: progress

## Status

| | |
|---|---|
| Current phase | Phase 1 (sim validation). **Phase 0 gate met, evidence below.** |
| Done | Phase 0: packages, layer goes current in sim, FastDEM map recorded, benchmark runner. Code: L2 lidar, worlds, ground truth, goal sets (P1). Feature node, mapping node extensions, depth_range_cut, terrain_eval (P2). Pose and heading evaluation, layer v2, 3D Nav2 overlay (P3). Swerve mode, TerrainCritic, swerve overlay (P4). Speed governor, safety monitor (P5). |
| In progress | Phase 1 sim checks: L2 near-field coverage, worlds in Gazebo, baseline rerun |
| Waiting on Ram | (1) NVIDIA module for the running kernel, see next row. Phase 1's gate item "real-time factor of at least 0.7" cannot pass on CPU rendering. (2) License of the prototype's Gale Crater mesh before it can be copied into the team repo (see Phase 1 notes). |
| Blocker for speed (not for correctness) | The NVIDIA kernel module is missing for the running kernel `7.0.0-38-generic` (installed only for `7.0.0-31`), so Gazebo renders sensors on the CPU and runs at a real-time factor of about 0.15 to 0.25. Fix needs sudo: `sudo apt install linux-modules-nvidia-595-open-7.0.0-38-generic` then reboot, or boot `7.0.0-31`. The baseline takes several times longer until then. |
| Stopped for | nothing. Commit rule since 2026-10-31: commit freely, Ram as sole author, no Claude attribution, ask before pushing. |

## Environment

- **Container:** `ghcr.io/iitbmartian/rover-dev@sha256:4ba62ec8083613f5f2d601611c2c24190cf69524a5b53e091ffa1d3cf9de31c8`.
  `docker/` has no Dockerfile yet, only a README, so this image is the reference.
- **Container build command** (repo `src/` mounted into a separate workspace so container and host
  builds never mix):
  `docker run --rm --network host --ipc host -u $(id -u):$(id -g) -e HOME=/tmp/h -v <cws>:/cws -v $PWD/src:/cws/src -w /cws <image@digest> bash -c "source /opt/ros/jazzy/setup.bash && colcon build ... && colcon test ... && colcon test-result --verbose"`
- **Evidence and tools** live outside the repo in `~/.ros/rover_3d_nav/` (`evidence/phase0/`,
  `tools/`), because the session scratchpad under `/tmp` is wiped on reboot.
- **Sim isolation:** sim runs use `ROS_DOMAIN_ID=42`, `GZ_PARTITION=rover3d`,
  `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` (`tools/simenv.sh`), so they never collide with
  Ram's own sessions.
- **Missing from the image:** `grid_map_rviz_plugin` (host RViz only). Nothing has been installed
  into the image.
- **DDS between host RViz and the container:** the host has `ROS_DOMAIN_ID=0`,
  `RMW_IMPLEMENTATION` unset (Fast DDS), `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`. The container
  leaves them unset, which gives the same defaults. Run the container with
  `--network host --ipc host`. Without `--ipc host`, Fast DDS shared memory transport fails:
  discovery works but data never arrives.

## Branches for other owners

| Branch | Package | Change | State |
|---|---|---|---|
| `ram/rover_gazebo-l2-lidar` | `rover_gazebo` (Pradyun) | `rover_sensors.xacro`: optional L2-like lidar (`lidar_model:=l2`, 360 x 96 deg, -6 to 90 deg vertical, 10 Hz, 48 x 360 samples, 2 cm noise), with `lidar_inverted` and `lidar_tilt`. Arguments default from `ROVER_LIDAR_MODEL`, `ROVER_LIDAR_INVERTED`, `ROVER_LIDAR_TILT`, so no launch file changes. Unset, the generated URDF is byte-identical to `main` (checked with xacro, paths normalised). | Committed locally (b82ed2e), merged into `ram/3d_nav`, not pushed. |
| `ram/rover_gazebo-terrain-worlds` | `rover_gazebo` (Pradyun) | `tools/make_terrain_worlds.py` and its output: `slopes`, `crater_field`, `rock_field`, `urc_mix` worlds, heightmap models `TerrainSlopes`, `TerrainCraterField`, `TerrainUrcMix`, `TerrainFlatNormal`. Existing worlds untouched. | Committed locally (2e8cf65), merged into `ram/3d_nav`, not pushed. |
| `ram/rover_gazebo-swerve-mode` | `rover_gazebo` (Pradyun) | `rover_kinematics_node.py`: `swerve` mode, vx, vy and wz together, `v_wheel = (vx, vy) + w x r`, through the existing fold and turn-rate limit. Default stays ackermann. Hand check FL at (0.2, 0.1, 0.3): 57.55 deg, 0.2509 m/s. | Committed locally (2ab6b54), merged into `ram/3d_nav`, not pushed. |
| `ram/rover_bringup-nav-params-arg` | `rover_bringup` (unassigned owner) | `pipeline.launch.py` gains a `nav_params_file` argument. Its default is the same `nav2_params_sim.yaml` that `navigation.launch.py` picks with `sim:=true`, so the 2D pipeline behaves exactly as before. | Committed locally (d46e027), merged into `ram/3d_nav` (3269f82), not pushed. |

## Notes on history

- The first 8 commits (`327cf73` to `3269f82`) had a Claude co-author line that was removed on
  Ram's instruction (messages rewritten, trees unchanged, nothing pushed). Pre-rewrite copies:
  branches `backup/3d_nav-before-trailer-strip` and `backup/bringup-before-trailer-strip`, to
  delete once Ram is happy.
- Not mine and left out of every commit: the edits in `src/rover_nav2/config/nav2_params_sim.yaml`,
  `src/rover_gazebo/worlds/obstacle_field.sdf`, `src/rover_drivers/docs/`, the deleted
  `unitree_lidar_ros2/COLCON_IGNORE` and the Unitree binaries.

## Decisions log

One line each: the choice and the reason.

- 2026-10-14: Container builds use a workspace outside the repo with `src/` mounted, because the
  host already has its own `build/` and `install/` in the repo root.
- 2026-10-14: `DiscDilation::apply` clamps the window to the grid, so a caller can pass bounds
  past the edge without reading out of range. The prototype assumed valid bounds.
- 2026-10-14: `PLACEHOLDER` markers are read from the raw line of each key, because yaml-cpp and
  PyYAML drop comments. A marker on a line of its own belongs to no key.
- 2026-10-14: `rover_physical.yaml` also carries `wheel_width` (0.14 m, from
  `rover_kinematics.yaml`), and the footprint as `footprint_length` and `footprint_width`. SPEC.md
  lists the footprint as a box only.
- 2026-10-14: The layer's thresholds (`max_slope`, `max_step`, `max_roughness`) stay layer
  parameters through Phase 2. They are the prototype's cost-model thresholds, not physical
  parameters, and Phase 3 replaces them with the utilisation model reading `rover_physical.yaml`.
  Note `max_step` 0.15 m differs from `step_max` 0.10 m.
- 2026-10-14: `nav2_params_sim_3d.yaml` is an overlay deep-merged over `nav2_params_sim.yaml` at
  launch time, not a copy, because the 2D file is being retuned and a copy would drift.
- 2026-10-14: FastDEM is built inside `rover_terrain_mapping` (`add_subdirectory`, static), so
  no separate colcon package exists and nothing lands outside my write scope.
- 2026-10-14: FastDEM's `fastdem_ros2` wrapper is not vendored. `terrain_mapping_node` replaces it
  and uses node-clock timers, so rates follow `/clock` in sim (upstream used wall timers).
- 2026-10-14: FastDEM inpainting is off. It invents ground across holes, which would hide a
  negative obstacle. Unfilled cells stay NaN and the layer prices them as unknown (200).
- 2026-10-14: FastDEM `range_min` 1.0 m drops the rover's own wheels (about 0.74 m from the
  lidar) until the Phase 2 self filter exists.
- 2026-10-14: `rover_terrain_mapping` removes grid_map_core's directory-wide Eigen plugin
  definitions after `find_package(rover_traversability)`, since they broke FastDEM's build.
- 2026-10-14: `rover_bringup/package.xml` does not yet list `rover_terrain_mapping` as a
  dependency. Adding it would edit an existing file in `rover_bringup`, so it is left for review.
- 2026-11-01: The MPPI critic evaluates only the yaw-dependent checks (tip, articulation, step)
  along rollouts. Belly clearance and roughness are yaw independent and already applied by the
  costmap at every cell, so repeating them per rollout point would only cost time.
- 2026-11-01: Unknown cells under the footprint are exempt from SPEC.md's "unknown inside the
  braking distance is lethal" rule. Every sensor has a blind zone under the rover, and marking
  it lethal puts the rover inside its own obstacle so it can never start. Braking distance is
  measured from the circumscribed circle. This is an interpretation of SPEC.md, flagged for Ram.
- 2026-11-01: The 3D collision monitor reads the terrain hazard cloud only, not raw lidar: the 2D
  lidar source treats every point 0.15 m above base_footprint as an obstacle, ramps included.
- 2026-11-01: Phases stay comparable in the benchmark through launch switches: `drive:=ackermann`
  (Phase 3) or `swerve` (Phase 4), `governor:=` and `safety:=` (Phase 5), `lidar_model:=`.
- 2026-11-01: The swerve mode is selected on `rover/steer_mode` by a low-rate latched publisher,
  since a latched sample dies with its publisher. The kinematics node acts only on change.
- 2026-11-01: `rover_traversability` defaults to a Release build. Host builds were -O0 and the
  heading evaluation cost 72.6 s of CPU per map instead of 0.52 s.
- 2026-11-01: SPEC.md updated (minor, logged here): feature definitions and units, `last_seen`
  as an age, the extra `slope_std` layer, the risk adjustment in practice, the exact 3D margin
  with body-fixed wheels, the belly disc approximation, the roughness disc, `roughness_max` and
  `wheel_width` in the physical table, the benchmark's definitions.
- 2026-11-01: `last_seen` is published as an age (s since the cell's newest point, at the map
  stamp), because float32 cannot hold an absolute stamp to better than about 100 s.
- 2026-11-01: Belly clearance uses the circumscribed disc of the belly rectangle: strict, and
  yaw independent so the per-heading evaluation stays affordable.
- 2026-11-01: The ground-truth z of Gazebo's `/odom` is absolute (terrain_eval `--spawn-z` 0).
- 2026-11-01: The baseline uses a snapshot of `nav2_params_sim.yaml` as it was on disk at start,
  Ram's uncommitted retune included (`nav2_params_sim.yaml.snapshot` and the diff are in the
  results directory), so live edits during the run cannot change it. Rows carry the params hash
  and `+dirty`.
- 2026-11-01: Benchmark success is judged by ground truth within 0.5 m of the goal, plus Nav2
  SUCCEEDED. Nav2's goal checker uses 0.15 m in its own (drifting) map frame, so 0.5 m leaves
  room for localisation error while still catching false successes.
- 2026-11-01: Benchmark runs a fresh stack per run with its own RTAB-Map database under the
  results directory, never the default `~/.ros/rover_slam.db` that Ram's own runs use.
- 2026-11-01: Terrain worlds use 16-bit heightmaps at 5 cm for smooth terrain, not stacked boxes,
  so ramps and craters have no step edges. A ground plane sits 0.5 m below the lowest terrain
  point, never propping up a crater floor.
- 2026-11-01: L2 lidar is a 48 x 360 grid (2 deg rows, about 5 cm apart on the ground 1.5 m out)
  instead of the real non-repeating pattern, which Gazebo's gpu_lidar cannot produce.
- 2026-11-01: FastDEM `min_valid_neighbors` 8 of about 13 disc cells (was 4). With the sparse
  16-ring lidar a 4-cell disc is often one ring arc, and the plane fit returned slope 90 degrees.
  A coverage rule, not a cost threshold.
- 2026-11-01: `terrain_3d.rviz` (nav2_sim.rviz plus grid_map displays) and
  `pipeline_3d.launch.py terrain_rviz:=true`.
- 2026-10-14: Phase 0 baseline counts roll and pitch unsafe entries only. Footprint-over-terrain
  needs the Phase 1 ground-truth heightmaps, and Phase 1 reruns the baseline.

## Phase 0 bridge: FastDEM features

FastDEM's `slope`, `step` and `roughness` feed the ported layer until Phase 2, when
`terrain_features_node` **replaces this bridge**. `SPEC.md` keeps our own feature definitions.
The only conversion is slope from degrees to radians, inside `terrain_mapping_node`
(`toContractLayers()`). No threshold is tuned against these definitions.

Source: FastDEM `fastdem/src/feature_extraction.cpp` at `5cd3e25`. All three come from one PCA over
the finite elevation cells in a disc of radius `analysis_radius`. Phase 0 uses 0.10 m (a 0.2 m
disc, the prototype's step window) on a 0.05 m map, about 13 cells.

| Feature | FastDEM definition | Units downstream | Strictness against SPEC.md intent |
|---|---|---|---|
| slope | `acos(abs(n_z))` of the PCA normal over the disc, published in degrees | radians | Not less strict. Same on uniform slopes. A 0.2 m disc picks up rock faces and noise, so it reads high rather than low. SPEC.md's roll and pitch come from the wheel-contact plane at 0.74 m scale, added in Phase 3. |
| step | 95th minus 5th percentile height in the disc | m | **LESS STRICT.** (a) The 0.2 m disc is smaller than the 0.30 m wheel diameter, so a rise spread over more than 0.2 m reads lower than the height the wheel climbs. (b) The percentile trim drops spikes under about 5 percent of the disc (one cell of 13), so a narrow sharp rock reads low. |
| roughness | `sqrt(smallest eigenvalue)`, RMS residual perpendicular to the fitted plane | m | **LESS STRICT.** SPEC.md applies roughness under the whole footprint, this uses a 0.2 m disc, so undulation longer than the disc fits the local plane and reads near zero. It is also lower than the prototype's standard deviation about the mean on any slope. |

## Sessions

### 2026-10-14, session 1

- Read AGENT.md, SPEC.md, PHASES.md, the prototype layer and FastDEM. Wrote `PHASE0_PLAN.md`.
  Ram's decisions are recorded in its section 1.
- Switched AGENT.md and PHASES.md to continuous work through all phases.
- `rover_traversability`: `traversability_core` (disc dilation, physical parameter loader),
  the ported layer, `rover_physical.yaml`, Python loader.
  Container test: 23 test cases, 0 failures (`colcon test --packages-select rover_traversability`).
- `rover_terrain_mapping`: FastDEM `5cd3e25` and nanoGrid `058a6a6` vendored with two CMake
  patches (`third_party/PATCHES.md`). Upstream only tests on Humble. On Jazzy all of FastDEM's
  tests pass with no source change. `terrain_mapping_node` publishes `/terrain/local` and
  `/terrain/local_features`.
  Container test: 206 tests, 0 failures (199 FastDEM, the example smoke run, 6 contract-layer).
- Surprise: FastDEM never sets the map timestamp, so its published GridMap stamp would be 0 and
  the layer's staleness check would always fire. `terrain_mapping_node` sets it from the newest
  integrated scan.
- Nav2 3D config as an overlay, `pipeline_3d.launch.py`, `terrain_mapping.launch.py`.
  Container: `colcon build --packages-up-to rover_bringup rover_terrain_mapping
  rover_traversability rover_nav2` builds 9 packages. `rover_nav2` pytest: 3 passed.
  Host build of the 4 changed packages also succeeds.
- Stopped at this checkpoint on Ram's instruction, before any sim run.

### 2026-10-31 and 2026-11-01, session 2

- Ram lifted the commit ban: commit freely, Ram sole author, no Claude attribution, ask before
  pushing. AGENT.md updated. Pending Phase 0 files committed.
- Found the repo checked out on `main` (someone switched branches). All commits were intact on
  `ram/3d_nav`. Switched back. Untracked leftovers under `third_party/fastdem/lib/nanoPCL/sandbox/`
  are upstream experiments that the vendored `.gitignore` excludes on purpose, not lost files.
- `ram/docker-setup` (pushed, not merged) holds the Dockerfile the `rover-dev` image comes from.
  It rosdeps every `package.xml`, so the dependencies our packages declare are what the image gets.

#### Gate item 2: the ported layer loads and goes current in Nav2 sim, with FastDEM feeding it

Command: `ros2 launch rover_bringup pipeline_3d.launch.py world:=marsyard.sdf gui:=false viz:=false`
(log `~/.ros/rover_3d_nav/evidence/phase0/launch_pipeline_3d_marsyard.log`).

- Plugin loaded: `TraversabilityLayer 'traversability_layer' subscribed to /terrain/local_features`.
- Nav2 lifecycle: `Managed nodes are active`.
- Goal to (3, 0) in `map`: **SUCCEEDED, 0 recoveries** (`goal_3m_marsyard.log`). The controller
  only computes while the local costmap is current, so the layer was current throughout.
- Fail-safe: after stopping `terrain_mapping_node`, the layer warned `elevation map is 1.01 s
  old` and kept reporting not current, the controller logged `Costmap timed out waiting for
  update`, and a new goal **ABORTED** instead of driving blind (`goal_failsafe_marsyard.log`).
- ERROR lines in the whole run: 2 from `ekf_filter_node` (update rate, not ours), plus the
  deliberate fail-safe abort. None from the new packages.
- Surprise: while the node was alive the layer still warned 3 times that the map was 1.00 to
  1.04 s old, right at `map_timeout` 1.0 s. At a real-time factor around 0.2 the publish chain
  lags in sim time. Revisit `map_timeout` once the sim runs at full speed.

#### Gate item 3: FastDEM elevation map from `/lidar/points` in RViz

- Bag `~/.ros/rover_3d_nav/evidence/phase0/phase0_terrain_marsyard` (86.5 s: 167 lidar scans,
  75 `/terrain/local`, 76 `/terrain/local_features`, 60 local costmaps, TF, odom, clock).
- Rendering of the last maps from the bag: `phase0_terrain_marsyard.png` (elevation, slope, step,
  costmap at the same stamp), made with `tools/render_terrain_bag.py`.
- No screenshot tool works on this desktop (Wayland, X grab fails), so the RViz view itself needs
  Ram: `ros2 bag play ~/.ros/rover_3d_nav/evidence/phase0/phase0_terrain_marsyard` with
  `rviz2 -d $(ros2 pkg prefix rover_nav2)/share/rover_nav2/rviz/terrain_3d.rviz`, or live with
  `pipeline_3d.launch.py terrain_rviz:=true` (the bag uses ROS domain 42).

#### What the bridge produces on marsyard (expected limits, recorded honestly)

- Standing still, the 16-ring lidar fills about 4 percent of the 10 x 10 m map, and with the
  coverage rule no feature cell at all, so the costmap reads unknown (200) everywhere. After the
  3 m drive: elevation 56 percent, features 49 percent.
- 536 of 19,780 feature cells (2.7 percent) exceed the prototype's 0.45 rad slope limit, step adds
  43, roughness none (`tools/lethal_cells.py`, `lethal_cells.txt`). Median slope 0.082 rad, p99
  0.587. The tail is range noise (2 cm) across a 0.2 m disc. This is the "slope reads high" entry
  in the bridge table. Thresholds stay untuned, per Ram.
- The prototype layer dilates every graded cost, including the unknown cost 200, by the inscribed
  radius (0.5 m). With NaN stripes between lidar rings almost every known cell has an unknown cell
  within 0.5 m, so the costmap is mostly 200 even where terrain is known. **Phase 3 layer v2 must
  not dilate the unknown cost**, only lethal and graded terrain cost.
- Launch shutdown can leave `gz sim` running as an orphan. The benchmark runner kills the whole
  process group and checks no Gazebo process is left before the next run.

#### Benchmark runner

- `ros2 run rover_nav2 benchmark` (`rover_nav2/benchmark/`): fresh stack per run in ROS domain 42
  and Gazebo partition `rover_benchmark`, scores from ground-truth `/odom`, writes `runs.csv`
  after each run and `summary.md`, resumes an interrupted run, stops stray processes by an
  environment marker. Metric tests: 11, hand-computed (tip limits 16.11 and 29.87 deg).
- First smoke-test crash: my `CpuSampler` shadowed `threading.Thread._stop`, so cleanup raised
  before the stack was stopped and left 29 processes running. Found them by the marker, stopped
  them, fixed the name. This is why stray-process cleanup runs in a `finally` block.
- **Smoke test finding, 2D stack on `bars`, goal (4, 0):** Nav2 reported SUCCEEDED after 28 s,
  but ground truth shows the rover stopped at x 0.92 m with its front wheels against the 8 cm bar
  and stayed there, wheels spinning, while wheel odometry (and RTAB-Map on it) believed it drove
  4 m. Path 2.16 m, final error 3.06 m. Scored as a failure. Wheel slip fools the 2D stack's
  localisation. Results: `~/.ros/rover_3d_nav/evidence/phase0/bench_smoke/`.
- `collision_monitor` segfaults (exit -11) during launch shutdown after a TF extrapolation error.
  Teardown only, not ours.

### Phase 1 work (started while the Phase 0 baseline runs)

- **L2-like lidar** on `ram/rover_gazebo-l2-lidar` (see branches). Not yet run in sim, waiting for
  the baseline to finish so CPU numbers stay clean.
- **Worlds** on `ram/rover_gazebo-terrain-worlds`. All four pass `gz sdf -k`. Ramp angles measured
  from the generated pixels: 15.0, 25.0, 35.0 deg, plateaus 0.800 m. First sidehill draft had a
  71 deg cliff on its high side and 2 m end tapers. Fixed: the high side holds a plateau and the
  tapers are sized to stay under 15 deg (measured 20.0 deg cross slope, 15.0 deg along).
  Not yet loaded in Gazebo.
- **Gale Crater patch:** the prototype's `mars_gale_crater_patch1` is an 18 MB STL, 570 x 420 m,
  93 m relief, triangles about 1 m apart, authored by Jasmeet Singh with no license file. Not
  copied: the license must be confirmed first, and at 1 m triangles it carries no rock-scale
  detail, so it suits Phase 6 long routes only. Ground-truth export skips meshes for now.
- **Ground-truth heightmaps:** `ros2 run rover_nav2 ground_truth_heightmap <world> --out x.npz
  --png`, analytic ray casts against the world SDF. 9 hand-computed tests (shapes, nested poses,
  the real `ledge.sdf` ramp). Exported for all 7 worlds to `~/.ros/rover_3d_nav/ground_truth/`.
- **Goal sets** for all 7 worlds, goal heights checked against ground truth.

### Phase 2 work (started while the Phase 0 baseline runs)

- **Feature math** (`rover_terrain_mapping/features.hpp`), 17 hand-computed tests. A real bug
  found by them: `0.15 / 0.05` is 2.9999999999999996 in floating point and the disc builder
  floored it, shrinking every window by one cell ring. Fixed with an epsilon.
- **terrain_features_node** reads and writes `grid_map_msgs` itself (circular buffer included,
  3 tests), because grid_map_core's Eigen plugin definitions break FastDEM's build in the same
  package. Same issue later hit `traversability_core` once it used Eigen: it now builds in its
  own CMake directory added before grid_map_core is found.
- **Replay of the Phase 0 marsyard bag, same map, bridge vs native features**
  (`tools/features_from_bag.sh`, `evidence/phase0/features_native_after_reach_fix.txt`):
  slope median 0.082 -> 0.017 rad, cells over the 0.45 rad limit 536 -> 0. Step p99 0.064 ->
  0.080 m, cells over 0.15 m 43 -> 79 (stricter by design). Negative-obstacle cells on hole-free
  ground: 3723 before the sensor-reach fix, 22 after.
- **Two harness mistakes, both mine, both caught:** (1) killing `ros2 run` leaves its child node
  running, so four stale `terrain_features_node` processes kept publishing old results into the
  comparison. The replay script now runs the binary directly with a trap. (2) An eval command
  backgrounded its own `source` step. Neither touched committed results.
- **Negative-obstacle limit, recorded honestly:** the 16-ring roof lidar cannot see the ground
  closer than 0.75 / tan(15 deg) = 2.8 m, so a hole nearer than that is invisible to it, and at
  3 m a gap must exceed 0.84 m along the ray to be told apart from row spacing. An inverted L2
  sees a 0.6 m hole at 2 m (tests `RoofLidarMissesANearHole`, `InvertedDomeSeesANearHole`).
- **terrain_mapping_node**: named inputs with their own sensor model and range limits, body-box
  self filter from `rover_physical.yaml`, gravity from TF or IMU (planar EKF on the real rover),
  `last_seen`, optional global map in `map` on `/terrain/global`. `depth_range_cut_node` added.
  Tests: contract layers 7, gravity alignment 2, range cut 2.
- **terrain_eval** (`ros2 run rover_nav2 terrain_eval`): elevation RMSE against ground truth in
  the world frame through ground-truth pose, and crater flag-before-reach timing. First number,
  Phase 0 bag (16-ring lidar, FastDEM local map, marsyard): **RMSE 1.7 cm within 6 m**, bias
  +0.8 cm, 69 maps. My first run showed 3.2 cm with a +2.8 cm bias: I had added the 0.02 m
  spawn height to Gazebo's ground-truth z, which is already absolute. Found by checking that the
  map's own ground sits at +0.002 m under base_footprint.
- Not yet run in sim: the new mapping node, depth input, features node live, crater check.

### Phase 3 work (started while the Phase 0 baseline runs)

- **Pose evaluation** (`traversability_core`, `core/pose_evaluation.hpp`), 13 hand-computed
  tests (flat, 10 deg sidehill, facing up the same slope, 20 deg uphill, one wheel on a block,
  rock under the belly, step over the limit, hole under a wheel, unknown contact, roughness,
  uncertainty, CG offset). The tests caught two real errors before any use: contacts read from
  the nearest cell understated a 10 degree tilt by 0.8 degree (now bilinear), and the first
  margin code placed wheels horizontally instead of body-fixed on the plane, overstating the
  margin on slopes. Both lenient, both fixed. My own first hand constant (0.451384 for the flat
  margin) was also wrong. The right value, 0.451126 rad, agrees with the benchmark's 16.11 deg
  roll limit.
- **Heading evaluation** (`core/yaw_evaluation`): `safe_yaw_mask`, `best_yaw`,
  `best_utilisation` for every cell, threaded, halved by front-back symmetry when the CG has no
  fore-aft offset. 5 tests (a 20 deg sidehill: only facing up or down is safe). Wired into
  `terrain_features_node`.
- **Performance** (`ros2 run`-free: `build/rover_terrain_mapping/bench_features`, process CPU
  time, synthetic fully observed 10 x 10 m map at 5 cm, 36064 cells with a verdict): features
  171 ms, negative obstacles 3 ms, roughness disc max 26 ms, heading evaluation 528 ms at 16
  headings on one thread. 4 threads: heading evaluation 242 ms wall on a machine loaded by the
  baseline sim. Before the Release default it was 72.6 s. NUC-class numbers still to measure.
- **TraversabilityLayer v2** (`cost_model: utilisation`), 4 tests: quadratic graded cost, lethal
  when no heading is safe, unknown lethal only in the braking band outside the footprint, only
  lethal cells dilated.
- **Phase 3 Nav2 overlay**: both costmaps on the terrain layer (the voxel layer and RTAB-Map's
  flattened grid both mark ramps as obstacles), a global `terrain_features_node` instance on
  `/terrain/global`, the L2 lidar inverted by default in `pipeline_3d.launch.py`,
  `navigation_3d.launch.py`.

### Phase 4 and 5 work (code only so far)

- **Swerve mode** on `ram/rover_gazebo-swerve-mode` (see branches).
- **TerrainCritic** (`core/trajectory_cost` plus an MPPI plugin): `w_terrain u^2`, collision cost
  on lethal poses, redirect cost per radian of travel-direction change, uphill time term. 4 tests
  (crossing a 20 deg slope facing uphill costs 10 x 20 x 0.387^2, driving across it is a
  collision) and a pluginlib load test.
- **Swerve overlay** `nav2_params_sim_3d_swerve.yaml`: MPPI Omni with TerrainCritic, Smac 2D,
  lateral velocity limits, PathAngleCritic and PreferForwardCritic off.
- **terrain_speed_governor** and **terrain_safety_monitor** (`core/speed_safety`, 4 tests), wired
  into `pipeline_3d.launch.py` with the collision monitor reading `/terrain/hazard_points`.
- **Not yet done:** any of this in sim. Every Phase 1 to 5 gate needs sim runs, which start once
  the baseline finishes.

## Phase 0 gate: met (2026-11-01)

| Gate item | Evidence |
|---|---|
| `colcon build` in the container, `colcon test` passes | `tools/dk.sh 'colcon build --packages-up-to rover_bringup rover_terrain_mapping rover_traversability rover_nav2 && colcon test --packages-select rover_traversability rover_terrain_mapping rover_nav2 && colcon test-result --verbose'`: 9 packages built, **329 tests, 0 errors, 0 failures** (`evidence/phase0/container_build_test_final.log`). Image digest in Environment. |
| Ported layer loads in Nav2 sim without errors (loads and goes current) | Session 2 entry above: plugin loaded, Nav2 active, goal SUCCEEDED with 0 recoveries, fail-safe abort when terrain stops. |
| FastDEM elevation map from `/lidar/points` visible in RViz | Bag `evidence/phase0/phase0_terrain_marsyard`, rendering `phase0_terrain_marsyard.png`, `terrain_3d.rviz`. Screen capture is impossible on this desktop, so Ram should look once: commands in the session 2 entry. |
| Baseline CSV and summary for marsyard, ledge, bars, 5 runs per goal | `ros2 run rover_nav2 benchmark --worlds marsyard ledge bars --runs 5 --config 2d --params-file <snapshot> --out ~/.ros/rover_benchmark/2026-11-01_baseline_2d`: `runs.csv`, `summary.md`, 35 runs, 5 per goal. |

### 2D baseline (`~/.ros/rover_benchmark/2026-11-01_baseline_2d/summary.md`)

| World, goal | Success | Time to goal [s] | Path [m] | Recoveries | Unsafe | Max roll / pitch [deg] | RTF |
|---|---|---|---|---|---|---|---|
| marsyard ahead_8m | 5/5 | 20.8 +- 0.2 | 7.94 | 0 | 0 | 0.1 / 1.0 | 0.38 |
| marsyard left_diag | 3/5 | 108.8 +- 119.8 | 15.7 | 0 | 0 | 7.8 / 4.3 | 0.37 |
| marsyard behind_left | 4/5 | 68.0 +- 96.2 | 11.5 | 0 | 0 | 6.3 / 4.4 | 0.36 |
| ledge on_ledge_6m | 4/5 | 48.4 +- 71.8 | 7.8 | 0.4 | 0 | 12.8 / 1.8 | 0.50 |
| ledge on_ledge_12m | 0/5 | aborted at 0.05 s | 0 | 0 | 0 | 0 / 0.1 | 0.51 |
| bars past_bars_4m | 0/5 | 50.6 +- 30.3 | 4.2 | 0.4 | 0 | 2.9 / 3.7 | 0.51 |
| bars past_bars_6m | 0/5 | 41.7 +- 15.3 | 4.4 | 0.4 | 0 | 6.9 / 6.6 | 0.42 |

16 of 35 runs succeed. Outcomes: 26 Nav2 SUCCEEDED, 6 aborted, 3 timeouts.

- **bars: 9 false successes.** Nav2 reports SUCCEEDED while ground truth puts the rover 2 to 4.4 m
  short, stuck against the 8 cm bar with its wheels spinning: wheel odometry (and RTAB-Map on it)
  believes it drove the distance. The 2D stack cannot tell a stuck rover from a moving one.
- **ledge on_ledge_12m: rejected at once.** The 2D global costmap spans only RTAB-Map's map so
  far, and a goal 12 m out lies off it.
- **marsyard timeouts:** 3 runs among the rocks hit the 240 s limit.
- No unsafe roll or pitch anywhere: the 2D stack never reaches steep ground on these worlds.
- Sim at real-time factor 0.36 to 0.51 (CPU rendering, see the NVIDIA note). Runs 6 to 9 overlapped
  my niced builds, their CPU numbers may be a little high, RTF did not change.
- Run on the 2D params snapshot taken at start (`nav2_params_sim.yaml.snapshot` and the diff of
  Ram's then-uncommitted retune in the results directory), commit `98ce60c+dirty`.
- Mid-baseline, `rover_gazebo`'s symlink-installed kinematics node gained the swerve branch. The
  ackermann path is unchanged, so the remaining runs are unaffected.

