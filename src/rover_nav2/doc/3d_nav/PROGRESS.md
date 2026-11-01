# 3D navigation: progress

## Status

| | |
|---|---|
| Current phase | Phase 0 closing (2D baseline running), Phase 1 code in parallel |
| Done | Phase 0: packages, layer loads and goes current in sim, FastDEM map recorded, benchmark runner. Phase 1: L2-like lidar option, four terrain worlds, ground-truth heightmap export, goal sets for all worlds. |
| In progress | 2D baseline, 35 runs, `~/.ros/rover_benchmark/2026-11-01_baseline_2d/` |
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

