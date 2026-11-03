# 3D navigation: progress

## Status

| | |
|---|---|
| Current phase | Phase 2 (mapping validation), with Phase 1's baseline rerun still to run. **Phase 0 gate met. Phase 1 gate items 1 and 2 met.** |
| Done | Phase 0: packages, layer goes current in sim, FastDEM map recorded, benchmark runner. Code: L2 lidar, worlds, ground truth, goal sets (P1). Feature node, mapping node extensions, depth_range_cut, terrain_eval (P2). Pose and heading evaluation, layer v2, 3D Nav2 overlay (P3). Swerve mode, TerrainCritic, swerve overlay (P4). Speed governor, safety monitor (P5). |
| Running unattended (started 2026-11-03 00:46) | `batch_overnight.sh`: Phase 2 map evaluation on all worlds, a 3D check of every goal, then the 2D baseline on all 7 worlds (5 runs per goal). Hung 6 h at slopes (fixed 07:06), now ends around 19:00, then a slopes rerun. |
| In progress | Phase 2 gate: 4 of 7 worlds pass (RMSE 1.76 to 1.94 cm), slopes and rock_field to rerun, crater_field fails on odometry yaw drift (see "Phase 2 gate run"). 3D check done: 2 of 16 succeed, 0 unsafe entries, Phase 3 work list in "3D check". |
| Waiting on Ram | Lab visit with the NUC: Phase 2's latency and CPU report on a NUC-class machine is measured there. Everything else continues on this machine. |
| Speed | Resolved: Ram installed the NVIDIA module. Gazebo alone runs at RTF 0.97, the full 3D stack at 0.50. |
| Stopped for | nothing. Commit rule since 2026-10-31: commit freely, Ram as sole author, no Claude attribution, ask before pushing. |

## Environment

- **Container:** `ghcr.io/iitbmartian/rover-dev@sha256:4ba62ec8083613f5f2d601611c2c24190cf69524a5b53e091ffa1d3cf9de31c8`.
  `docker/` has no Dockerfile yet, only a README, so this image is the reference.
- **Container build command** (repo `src/` mounted into a separate workspace so container and host
  builds never mix):
  `docker run --rm --network host --ipc host -u $(id -u):$(id -g) -e HOME=/tmp/h -v <cws>:/cws -v $PWD/src:/cws/src -w /cws <image@digest> bash -c "source /opt/ros/jazzy/setup.bash && colcon build ... && colcon test ... && colcon test-result --verbose"`
- **Evidence and tools** live outside the repo in `~/.ros/rover_3d_nav/` (`evidence/phase0/`,
  `tools/`), because the session scratchpad under `/tmp` is wiped on reboot.
- **Results archive for publication:** `~/rover_3d_nav_results/` (Ram, 2026-11-02). `RESULTS.md`
  logs every headline number with commit and command, `figures/` and `snapshots/` hold PNG and SVG
  figures with their data, and benchmarks, evidence, ground truth and tools are linked in.
- **Disk:** 96 percent full on 2026-11-02. Benchmark runs drop their RTAB-Map database unless
  `--keep-db`.
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
| `ram/rover_gazebo-terrain-worlds` | `rover_gazebo` (Pradyun) | `tools/make_terrain_worlds.py` and its output: `slopes`, `crater_field`, `rock_field`, `urc_mix` worlds, terrain models `TerrainSlopes`, `TerrainCraterField`, `TerrainUrcMix` as binary STL meshes (0.1 m, or 0.2 m for urc_mix, about 131k triangles each) for both collision and visual. Heightmaps were dropped on 2026-11-01: Gazebo's heightmap physics and rendering disagreed by about 8 cm on slopes (see Phase 2 findings). Existing worlds untouched. | Committed locally (650aaab), merged into `ram/3d_nav`, not pushed. |
| `ram/rover_gazebo-l2-lidar` (addition) | `rover_gazebo` (Pradyun) | `lidar_xyz` argument and `ROVER_LIDAR_XYZ`, so the L2 can sit on a 0.5 m mast. | Committed locally, merged into `ram/3d_nav`, not pushed. |
| `ram/rover_gazebo-swerve-mode` | `rover_gazebo` (Pradyun) | `rover_kinematics_node.py`: `swerve` mode, vx, vy and wz together, `v_wheel = (vx, vy) + w x r`, through the existing fold and turn-rate limit. Default stays ackermann. Hand check FL at (0.2, 0.1, 0.3): 57.55 deg, 0.2509 m/s. | Committed locally (2ab6b54), merged into `ram/3d_nav`, not pushed. |
| `ram/rover_bringup-nav-params-arg` | `rover_bringup` (unassigned owner) | `pipeline.launch.py` gains a `nav_params_file` argument. Its default is the same `nav2_params_sim.yaml` that `navigation.launch.py` picks with `sim:=true`, so the 2D pipeline behaves exactly as before. | Committed locally (d46e027), merged into `ram/3d_nav` (3269f82), not pushed. |

## Notes on history

- The first 8 commits (`327cf73` to `3269f82`) had a Claude co-author line that was removed on
  Ram's instruction (messages rewritten, trees unchanged, nothing pushed). Pre-rewrite copies:
  branches `backup/3d_nav-before-trailer-strip` and `backup/bringup-before-trailer-strip`, to
  delete once Ram is happy.
- Ram's own local changes, committed on 2026-11-02 at his request ("make sure any other change i
  did on my local directory goes to git"): `obstacle_field.sdf`, `rover_drivers/docs/`, the
  deleted `unitree_lidar_ros2/COLCON_IGNORE` (the package builds) and the rebuilt Unitree
  binaries. His 2D tuning of `nav2_params_sim.yaml` (MPPI horizon 50 steps, speeds doubled,
  inflation) went in earlier by mistake inside `afcb10b` ("add terrain_eval ..."), so that
  commit's message does not mention it. The 3D overlay builds on that file, so 3D runs since
  then include his tuning too.
- `ram/3d_nav` is pushed to `origin/ram/3d_nav` and merges straight into main when done (Ram,
  2026-11-02). The owner branches are not pushed separately: their commits reach the remote
  through `ram/3d_nav`'s merges.

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
- 2026-11-01, Ram: the prototype's Gale Crater mesh may be copied, no license issue.
- 2026-11-01, Ram: the blind-zone exemption (cells under the rover's footprint are not lethal
  when unknown, even inside the braking distance) is confirmed.
- 2026-11-01, Ram: Phase 2's NUC-class latency and CPU report waits for his next lab visit,
  where the stack moves to the NUC.
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
- 2026-11-02: SPEC change, configuration space. Utilisation verdicts already place the whole
  rover at a cell, so the 3D overlay gives Nav2 a 0.1 m point footprint, drops the layer's lethal
  dilation (`dilate_terrain_lethal: false`), keeps inflation to one local cell (0.08 m, only so
  Smac keeps its fast collision check), and gives the collision monitor a centre polygon. The
  braking band for unknown cells is still measured from the physical footprint
  (`braking_from_physical_footprint`). Footprint, inflation and dilation together counted the
  rover's width two or three times and closed slopes.sdf's 2.5 m causeway to the planner.
  Flagged for Ram, since SPEC.md said "dilated by the inscribed radius".
- 2026-11-02: SPEC change, `step` is second highest minus second lowest over the wheel disc,
  after subtracting the chassis plane's gradient. Raw max minus min measured the slope itself
  (0.16 m on a 15 deg ramp on the 0.2 m global grid) and, untrimmed over about 37 cells, had a
  noise floor near 4 sigma, 6 to 8 cm at the sim lidar's 1.5 to 2 cm. An abrupt step keeps its
  height less at most one cell of plane slope (under 12 percent). One stray cell at either end
  is trimmed. A cliff's hazard band narrows by one cell on each side. Flagged for Ram, since
  SPEC.md said "no trimming".
- 2026-11-02: SPEC clarification, the `variance` layer is the Kalman estimate's P, as SPEC.md
  always said. The node had been publishing FastDEM's own `variance` layer, which is the sample
  spread of raw returns and never shrinks, so kappa sigma widened every step check by the full
  2 to 3 cm lidar spread.
- 2026-11-02: Unknown cells under the footprint take the plane the rover stands on (TF), in the
  features node only, never in the published elevation. This is how Ram's "exempt the rover
  itself" reaches the pose checks: without it every pose near the rover reached into the blind
  zone under the body and came back with no verdict, which the braking rule made lethal.
- 2026-11-02: FastDEM PATCH 3, `mapping.cell_mean`: each scan feeds the estimator the mean of a
  cell's returns, with variance / n, instead of the lowest return. The lowest of n returns with
  2 cm noise sat 1 to 5 cm below flat ground. Default false keeps upstream behaviour.
- 2026-11-02: Self-filter box reaches 100 m below ground, not 0.05 m. Ground under the body is
  hidden anyway, and the 0.05 m floor let through only the returns noise pushed below it: a
  false 5 cm dip all round the rover that tripped the step check.
- 2026-11-02: Lidar `range_min` 0.3 m, not 1.0 m. The body box does the self filtering, and the
  1.0 m cut left a uniform unknown ring out to 1.0 m that the braking rule made lethal at start.
- 2026-11-03: Sim runs keep Fast DDS defaults with `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, set
  `GZ_IP=127.0.0.1`, and clean zombie shared memory (`fastdds shm clean`) before every start.
  Tried and dropped: a loopback-only Fast DDS profile (2026-11-02, after a Wi-Fi change cut
  discovery once). UDP-only, it left late-joining nodes undiscovered: the collision monitor never
  met the velocity smoother, `/cmd_vel` had no publisher, and the slopes `sidehill_uphill` run
  never moved (that run is invalid). With shared memory it still hid `/odom` from tools.
- 2026-11-03: Long runs go in one detached batch (`tools/batch_overnight.sh`, `setsid nohup`),
  so network drops or a stopped session lose nothing. Progress in
  `~/rover_3d_nav_results/batch.log`.
- 2026-11-03: The overnight batch hung 6 h after the slopes run. `ros2 run` does not forward
  SIGINT, so `phase2_eval.sh` now signals `terrain_eval` itself and forces it after 30 s, and
  `terrain_eval` treats rclpy's `ExternalShutdownException` like Ctrl-C and still writes its CSV
  and summary. The slopes evaluation was lost (its snapshot kept: RMS 3.62 cm). Batch resumed at
  marsyard 07:06, so it ends about 6 h later than planned.
- 2026-11-03: The slopes run never drove because `ros2 action send_goal` never discovered
  `bt_navigator` and sat out its 30 min timeout (no "Begin navigating" in the log). Phase 2 runs
  now send goals with `tools/send_goal.py`, an rclpy client that waits 60 s for the server and
  retries up to 4 times with a fresh context. The benchmark runner already sends in-process.
- 2026-11-03: The 3D check's 5 startup failures were the runner, not Nav2: the launch logs say
  "Managed nodes are active", but the runner's one long-lived node never discovered
  `bt_navigator/get_state`. A fresh process discovers it 10 of 10 times against a settled stack
  (0.3 to 4.5 s). The runner now gets a fresh DDS participant per stack (own rclpy context,
  stopped before the shared-memory clean, started after), and renews it again if Nav2 is
  active in its log but not discovered after 20 s. Validated live after the baseline batch.
- 2026-11-03: Step is now max(step on a 3x3 median map, raw step - 0.05 m). On flat ground
  the trimmed step read 5 cm median (u 0.65 to 0.73 at a standstill, step the limiting check in
  88 to 100 percent of cells near the rover). Offline on the saved snapshots (`p2_*_end.npz`)
  the new step reads 1.6 cm (slopes), 1.8 cm (bars) and 1.0 cm (urc_mix) median, the real bars
  still 11 cm. A plain median would erase a 2x2 stub of any height, a plain 3x3 mean shaves a
  2-cell rib to 2/3, hence the hybrid. Tests: `Median3.*`, `Features.StepMedian*`.
- 2026-11-01: Terrain worlds are STL meshes, not heightmaps (see Phase 2 findings).
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
  (Superseded, see "Phase 2: live sim findings" below.)

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

## Phase 1: sim validation (2026-11-01)

### Gate item 1: L2-like lidar puts ground cells within 1.0 m of the footprint edge: PASS

`tools/phase1_coverage.sh` and the mount sweep, rover at rest on `flat.sdf`,
`ros2 run rover_nav2 coverage_check --duration 30-40` (worst bearing over 36 bearings, from the
footprint edge to the nearest observed ground cell, median map; observed share of the 1 m band):

| Lidar and mount | Worst bearing | Band observed | Evidence |
|---|---|---|---|
| 16-ring roof lidar (default) | 2.33 m | 6 percent | `coverage_roof.txt` |
| L2 inverted at the roof-lidar point | no ground at all on some bearings | 6 percent | `coverage_l2_inverted.txt` |
| L2 inverted, front bracket (0.4, 0, 0.35) | 0.83 m | 65 percent | `coverage_front.txt` |
| **L2 inverted, 0.5 m mast (0, 0, 0.5)** | **0.53 m** | **73 percent** | `coverage_mast050.txt` |
| L2 inverted, 0.75 m mast (0, 0, 0.75) | 0.58 m | 72 percent | `coverage_mast075.txt` |

- Surprise: at the roof-lidar point the dome sits inside the chassis geometry. One scan in
  base_footprint (`tools/cloud_stats.py`): 5022 of 6270 returns on the rover itself at 5 to 10 cm
  range, nearest ground return 5 m out. Added `lidar_xyz` to the L2 branch. The sim now mounts
  it on a 0.5 m mast by default (`pipeline_3d.launch.py`). The real mount is a Phase 7 item.
- Once, Nav2's lifecycle manager waited forever on `bt_navigator/get_state` (DDS never
  discovered the service). Not our code: the same launch came up in every other run. Both
  `tools/sim_up.sh` and the benchmark runner now retry a stalled start once.

### Gate item 2: PASS under the gate Ram changed on 2026-11-01

**Ram's decision:** the gate is now "the 3D additions cost at most 10 percent of the base
pipeline's real-time factor" (PHASES.md updated). The base pipeline's load from other owners'
Python nodes is to be raised with those owners (`rover_gazebo`: rocker_coupling, kinematics,
control_node; `rover_ekf`: steer_wheel_splitter, wheel_body_velocity, covariance_relay, four
steer_wheel_relay). Measured: 0.545 to 0.496, **9 percent**. Measurements and the original
analysis below.

#### Original measurements (gate was 0.7)


Ram installed the NVIDIA driver (GTX 1650, 595.99, kernel 7.0.0-42, the HWE meta-package keeps
future kernels covered). Measured on `slopes.sdf` with `tools/rtf_probe.py` (`/clock` against wall
time, 30 s, `evidence/phase1/rtf_*`):

| Configuration | Real-time factor |
|---|---|
| Full 3D stack, CPU rendering (before the driver) | 0.465 |
| Full 3D stack, GPU (`pipeline_3d.launch.py`, L2 on the mast, camera) | 0.496 |
| 2D pipeline with the same L2 (`pipeline.launch.py`: Gazebo, EKF, RTAB-Map, 2D Nav2) | 0.545 |
| Gazebo and the rover only, L2 on the mast (`rover_sim.launch.py`) | 0.973 |

The sensors render cheaply on the GPU (10 percent use). The real-time factor is set by CPU load
on 12 cores (load average near 29): Gazebo itself about 170 percent (1 kHz physics with the
suspension constraints), and other owners' Python nodes running at joint-state rates about
550 percent together (`rocker_coupling` 65 to 73, `steer_wheel_splitter` 64 to 72,
`wheel_body_velocity` 49 to 55, `rover_kinematics` 48 to 54, `covariance_relay` 42 to 48,
`control_node` 41, four `steer_wheel_relay` about 40 each). Our 3D additions cost 0.545 to 0.496,
9 percent (terrain_features 60 percent CPU, terrain_mapping 24).

**Question for Ram.** The 0.7 gate cannot pass without changes to `rover_gazebo` and `rover_ekf`.
- Recommended: judge the 3D stack by what it adds, "at most 10 percent below the base pipeline's
  real-time factor" (met: 9 percent), and raise the Python nodes' load with their owners. Every
  benchmark number is in sim time, so a lower real-time factor costs wall time, not validity.
- Alternative: I prepare branches for `rover_gazebo` and `rover_ekf` that cut their rates or
  move the hot loops to C++, for the owners to review. Large and outside my scope.
- Alternative: keep 0.7 and treat the gate as blocked until a faster machine.

### Gate item 3: baseline rerun on all worlds: pending

Starts after the Phase 2 evaluation runs, so the two never share the CPU.

## Phase 2: live sim findings (2026-11-01 and 2026-11-02)

The 3D stack on slopes.sdf first could not leave its spawn point: MPPI found no collision-free
rollout ("Optimizer fail to compute path"), then Smac found no path up the ramp lanes. Each cause
below was found with a probe from `~/.ros/rover_3d_nav/tools/` and checked again after its fix.
The goal was `lane15_top` (8.5, 0) on slopes.sdf, sent with `ros2 action send_goal
/navigate_to_pose ...` against `sim_up.sh phase2 <name> world:=slopes.sdf`.

| # | Finding | Probe and evidence | Fix |
|---|---|---|---|
| 1 | Gazebo heightmaps disagree with themselves: on slopes the rover rested 5.3 cm high and the lidar saw ground 7.8 cm low relative to physics (0.0 cm on flat.sdf). | `mesh_probe_slopes.txt` | Terrain as STL meshes: rover at z -0.0006 m, lidar ground error 0.0 cm. |
| 2 | Lidar `range_min` 1.0 m left a uniform unknown disc to 1.0 m, the same at every bearing, lethal under the braking rule. | `ring.py` by bearing: 0.76 to 1.01 m known 16 to 21 percent, after 99 to 100 percent | `range_min` 0.3 m |
| 3 | FastDEM fed the lowest return per cell per scan. | Raw returns unbiased with 1.7 cm noise, map 1 to 5 cm low | PATCH 3, cell mean (tests `test_cell_mean`) |
| 4 | Self-filter box floor at -0.05 m let only noise-low returns through under the body edge: a ring of -5 to -6 cm cells. | `elev_near.py` grid, `lethal_near.py`: all lethal cells limited by step (`limiting_check` 3) | Box reaches 100 m down |
| 5 | Published `variance` was the raw-return spread, so kappa sigma added 2 to 3 cm to every step. | variance 6.8e-4 at lethal cells, after 1e-4 (the Kalman floor) | `variance` is the Kalman P (`test_contract_layers`) |
| 6 | Cells next to the rover had elevation but no verdict, since the pose check reached into the blind zone under the body. | `costmap_near.py`: 0.76 to 1.01 m verdict known 72 percent, after 100 percent | Footprint fill with the rover's own plane (`FillFootprint` test) |
| 7 | Step measured the ramp: 0.11 to 0.17 m on the 15 deg lane in the global map, so the lane was lethal from x 3.4 m. | `features_line.py /terrain/global_features 0 0 8.5 0` | Step relative to the chassis plane (`SmoothRampIsNotAStep`, `CliffOnARampKeepsItsHeight`) |
| 8 | Untrimmed step had a noise floor near the 10 cm limit. The sim lidar's fixed ray pattern leaves a 1.4 to 1.9 cm per-cell error that averaging cannot remove (`lidar_cells.py`: cell means over 40 scans spread as much as single returns), since range noise also moves each return sideways and sorts low and high returns into neighbouring cells. A real L2 sweeps its pattern, so this is partly a sim artefact. | `map_vs_gt.py`: front (lidar plus depth) 1.0 to 1.4 cm, lidar only 1.4 to 1.9 cm | Trimmed step (`SingleCellSpikeIsTrimmedButTwoCellsCount`) |
| 9 | The causeway was closed to Smac: verdict lethal from abs(y) 0.8 m, then inflation and the footprint check took another 0.5 m each side. | `global_line.py 6 -2 6 2` | Configuration space: point footprint, no dilation, minimal inflation |

Result: **`lane15_top` SUCCEEDED**, rover on the plateau at (8.00, 0.35, 0.797) by Gazebo's
`gz model -m rover -p` (log `evidence/phase2/lane15_success.log`). Tests after the changes:
rover_terrain_mapping 240, rover_traversability 66, all pass. The legacy and v2 layer tests
share topics and once clashed under `ctest -j4`; run alone they pass.

Open from these findings:
- The per-heading layers are the most expensive part of the features node. Not re-measured after
  the changes.
- Ramp cells still score u 0.6 to 0.9 (step at contacts plus kappa sigma), so the planner pays a
  high cost on ramps. Phase 3 tuning will look at this against the benchmark.
- The fixed ray pattern of the sim L2 is a sim limitation worth raising with `rover_gazebo`'s
  owner: a slow yaw spin of the sensor would sweep the pattern like the real one.


## Phase 2 gate run (2026-11-03)

Per-world elevation RMSE within 6 m of the rover, against ground truth, aligned to the world frame
through Gazebo's pose. Gate: at most 5 cm. Commands: `tools/phase2_all.sh` (runs `terrain_eval`
and `tools/snapshot.py` per world), evidence in `evidence/phase2/eval_<world>.txt` and
`snapshot_<world>.txt`.

| World | terrain_eval RMSE (maps) | Snapshot RMSE | Verdict |
|---|---|---|---|
| marsyard | 1.92 cm (256 maps, per-map worst 3.9) | 1.99 cm | pass |
| ledge | 1.92 cm (458 maps, worst 5.0) | 1.95 cm | pass |
| bars | 1.94 cm (1267 maps, worst 3.7) | 2.03 cm | pass |
| urc_mix | 1.76 cm (665 maps, worst 2.7) | 1.71 cm | pass |
| rock_field | lost, shutdown race (fixed in `77e0913`) | 3.15 cm | rerun |
| slopes | lost, the goal never reached Nav2 (fixed with `tools/send_goal.py`) | 3.62 cm | rerun queued |
| crater_field | 27.1 cm (3862 maps) | 32.2 cm | **fail, odometry** |

Crater_field is not a mapping error. Locally the map is within about 2 cm. All three goals
aborted with many recovery spins, and odometry yaw drifted 27 deg (`snapshot_crater_field.txt`:
odom to world offset 0.34 m, yaw -27.2 deg). The local map lives in `odom`, so cells mapped
before the drift stay at their old heading. The terrain mesh ends at x -3.8 m with the ground
plane 1.25 m lower, and that edge shows up rotated about 20 deg in `snapshots/p2_crater_field_end.png`,
which is where the error comes from. The EKF fuses the gyro (`ekf_rover_gazebo_imu.yaml`), so
27 deg of yaw drift during recovery spins is worth raising with `rover_ekf`'s owner. The crater
check is untested: the rover never came within 8 m of a crater.

Next for the gate: rerun slopes and rock_field, and crater_field with goals the rover reaches.
The per-world RMSE also needs the gate's 6 m window to exclude cells mapped before a large drift,
or the gate should be stated for drift under some bound. Proposed SPEC wording follows once the
reruns are in.

## 3D check, one run per goal (2026-11-03)

`ros2 run rover_nav2 benchmark --worlds all --runs 1 --config 3d --tag 3d --out
~/.ros/rover_benchmark/2026-11-03_3d_check`, commit `9597be6`.

- 21 runs: 5 startup failures (lane25_top, crater far_left, marsyard behind_left, both bars
  goals), 16 scored.
- 2 of 16 succeed by ground truth (marsyard ahead_8m 0.33 m, crater ahead_10m 0.39 m).
- 4 more report Nav2 SUCCEEDED but end 0.59 to 3.07 m from the goal by ground truth (lane15_top,
  past_lanes, urc mid_20m, urc far_east): map-frame drift.
- The rest abort or time out, most with 17 to 19 recoveries. rock_field far_right aborted 15.5 m
  short after 79 s.
- **0 unsafe entries on every run.**

Phase 3 work list from this run, most important first:
1. Startup failures, 5 of 21. Find which node stalls and why the runner's retry does not recover.
2. Flat ground costs u 0.4 to 0.8 out to 3 m because the sim lidar's ring striping reads as a
   wheel-disc step (`snapshots/p2_slopes_end.png`, limiting check "step" everywhere). This makes
   every route expensive and likely drives the recoveries.
3. Localisation drift: success by Nav2 but metres off by ground truth, and 27 deg yaw on
   crater_field. Raise with `rover_ekf` and RTAB-Map owners.
4. The lane-corner trap and the sidehill abort from 2026-11-02.
