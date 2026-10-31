# 3D navigation: progress

## Status

| | |
|---|---|
| Current phase | Phase 0, foundation |
| Done | `rover_traversability` (core dilation, ported layer, physical parameters). `rover_terrain_mapping` (FastDEM vendored, `terrain_mapping_node`). 3D Nav2 overlay config and launch files. All build in the container and on the host. |
| Not yet done in Phase 0 | Sim checks (layer goes current, FastDEM map in RViz), benchmark runner, 2D baseline runs |
| Stopped for | nothing. Commit rule since 2026-10-31: commit freely, Ram as sole author, no Claude attribution, ask before pushing. |

## Environment

- **Container:** `ghcr.io/iitbmartian/rover-dev@sha256:4ba62ec8083613f5f2d601611c2c24190cf69524a5b53e091ffa1d3cf9de31c8`.
  `docker/` has no Dockerfile yet, only a README, so this image is the reference.
- **Container build command** (repo `src/` mounted into a separate workspace so container and host
  builds never mix):
  `docker run --rm --network host --ipc host -u $(id -u):$(id -g) -e HOME=/tmp/h -v <cws>:/cws -v $PWD/src:/cws/src -w /cws <image@digest> bash -c "source /opt/ros/jazzy/setup.bash && colcon build ... && colcon test ... && colcon test-result --verbose"`
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
