# rover_testing

Ready-made benchmarks for the simulated rover stack, and a UI to run them.

```
ros2 launch rover_testing run_tests.launch.py
```

opens the test runner, and nothing else. Tick the test cases to run (grouped
into navigation, drive and mapping; hover one for its description), set the
localization tolerance, optionally a timeout override or the Gazebo GUI, and
press **Run**. Tests run one after another; each starts the full pipeline, runs,
shuts everything down, and adds a row to the results table (green pass, red
fail). Double-click a row for its checks, metrics, the map overlay for mapping
tests, and the raw report. Every session is saved under
`~/.ros/rover_testing/<timestamp>/`: a JSON report and launch log per test, the
mapping tests' maps and overlays, and `summary.json`.

Any test also runs on its own:

```
ros2 launch rover_testing nav_obstacle_course.launch.py tolerance:=0.2 gui:=true
```

## Test cases

All checks use `tolerance` (m) from the UI for localization. *Localized* is TF
`map -> base_footprint` (RTAB-Map + EKF); *ground truth* is Gazebo's `/odom`,
moved into the map frame with the spawn pose.

### Navigation (Nav2, `goal_benchmark`)

| Test | World | Route |
| --- | --- | --- |
| `nav_empty_world_3_3` | empty_world | (0, 0) to (3, 3) |
| `nav_obstacle_course` | obstacle_course | 13 m across a slalom of four obstacles |
| `nav_slope_plateau` | slopes | up an 8° ramp onto a 0.4 m plateau |
| `nav_slope_crossing` | slopes | up, across and down the 8° hill |

Checks: `initial_localization`, `initial_at_spawn`, `nav2_succeeded` (route
completed and the final goal SUCCEEDED, within the timeout),
`final_localization`, `reached_goal` (ground truth within `tolerance` + Nav2's
0.15 m `xy_goal_tolerance`). Metrics: time to goal (sim s), path length,
recoveries, distance to goal, localization errors.

Two things the routes are built around, both measured:

- Routes longer than about 8 m go through an intermediate waypoint. Nav2
  rejects a goal outside its global costmap, which is RTAB-Map's `/map`, and at
  the spawn that covers only the lidar's 8 m grid range ("Goal Coordinates ...
  was outside bounds", before the rover moves).
- Intermediate waypoints are *passed*, not parked on: within 0.5 m of the
  localized pose the goal is cancelled and the next one sent. Asked to stop on
  each, the rover (0.8 m minimum turn radius) circled a waypoint 0.3 m away for
  300 s to meet Nav2's 0.15 m / 0.25 rad goal checker. Only the final goal has to
  be a real Nav2 success, which is a test of that parking in its own right.

### Drive (fixed input, no planner, `drive_benchmark`)

A constant 0.5 m/s forward `TwistStamped` on `/cmd_vel` for distance / speed sim
seconds, with Nav2 left out of the pipeline. This separates what the *vehicle and
estimator* can do from what the planner will agree to.

| Test | World | Drive |
| --- | --- | --- |
| `drive_flat_straight` | empty_world | 5.5 m on the flat: the baseline |
| `drive_slope_8deg` | slopes | 6.5 m up the 8° ramp, 0.4 m climb |
| `drive_slope_15deg` | slopes | 4.8 m up the 15° ramp, 0.6 m climb |

Checks: `progress` (ground-truth distance ≥ 85% of commanded: slip or stall),
`climbed` (height gained ≥ expected − 0.1 m), `stable` (roll and pitch under 35°),
`localization_during_drive` (max error over the whole drive ≤ tolerance),
plus the initial and final localization checks. Metrics: progress ratio, climb,
peak pitch, max/mean localization error, and a 10 Hz trace in the report.

### Mapping (Nav2 route, then `/map` scored against an ideal map)

| Test | World | Route | Scored area |
| --- | --- | --- | --- |
| `map_empty_world` | empty_world | 4 m square loop, back home | whole arena |
| `map_obstacle_course` | obstacle_course | tour past every obstacle | whole arena |
| `map_slopes` | slopes | short loop facing the 8° hill | x up to 1.7, walls included |

Mapping tests score the map, not the driving: every waypoint, the last one too,
is passed within 0.5 m, a leg stuck for 90 s is skipped, and the navigation
check is `route_completed` (Nav2's own result stays in the report).

At the end of the route RTAB-Map's `/map` is compared, cell by cell and without
any registration (a drifted map is a wrong map), with `maps/<world>.yaml`
(`map_compare.py`), within a 0.15 m match distance:

| Metric | Meaning | Pass at |
| --- | --- | --- |
| precision | share of mapped obstacle cells that are real obstacles; low = phantom obstacles | ≥ 0.85 |
| recall | share of observed real obstacle outlines that were mapped (insides are never seen, so not scored); low = missed walls | ≥ 0.80 |
| coverage | share of the reachable floor the map knows | ≥ 0.60 |
| free_agreement | share of mapped free cells not inside an obstacle | reported |
| obstacle error | mean / p95 distance of mapped obstacles from the real ones | reported |

The overlay image (details window, and `<test>_map_overlay.png`) shows green
found obstacles, red phantoms, blue missed obstacles, orange free space painted
through an obstacle. The rover's map is saved next to it as a map_server
`.yaml`/`.pgm`.

## The ideal maps

`maps/*.yaml` are generated, not drawn: `reference_map.py` reads the world's
static boxes and cylinders, rasterises the top surface into a 2.5D height field,
and flood-fills what the rover can reach from its spawn. "Ideal" means
*traversable*, because Nav2 plans in this map:

- free: reachable from the spawn across steps of at most 0.15 m (the stack's own
  `Grid/MaxGroundHeight`, and above the 10 cm ledge the rover is built to climb)
  over ground no steeper than 30°;
- occupied: unreachable and raised, too steep, or the higher side of a step;
- unknown: unreachable ground outside the arena.

So a climbable ramp is free in the ideal map. RTAB-Map and Nav2 classify by
height alone (anything 0.15 m above the rover is an obstacle), so they may mark
it occupied; `map_slopes` measures that gap, and `nav_slope_*` its consequence
for planning. The drive tests show the vehicle itself climbs both ramps.

## Worlds

`worlds/*.sdf` are generated by `worldgen.py` from rover_gazebo's
`empty_world.sdf` template (same systems, GPS origin, and world name
`husarion_world`, which the bridge config needs):

- `obstacle_course.sdf`: 16 × 10 m arena, four 1 m obstacles (two boxes, one
  rotated box, a cylinder) and a low 8 cm plate that should stay free.
- `slopes.sdf`: 24 × 6 m corridor, two hills spanning its full width (no way
  around): 8° to a 0.4 m plateau at x −6.0 … 1.69, and 15° to a 0.6 m plateau at
  x 4.0 … 10.48.

To change a world or add one, edit `worldgen.py` and regenerate worlds and maps:

```
ros2 run rover_testing make_test_worlds --out src/rover_testing
colcon build --packages-select rover_testing
```

`make_reference_map <world.sdf> <out_stem> --spawn X Y` builds an ideal map for
any other world made of boxes and cylinders.

## Launch files

| File | What it is |
| --- | --- |
| `pipeline.launch.py` | Gazebo, EKF, RTAB-Map, Nav2, composed from rover_bringup's four component launch files (see below). `nav2:=false` leaves Nav2 out. |
| `rover_sim.launch.py`, `ekf_rover_gazebo.launch.py`, `rtabmap.launch.py`, `navigation.launch.py` | Thin wrappers that include the same-named rover_bringup file with test-friendly defaults. |
| `tests/*.launch.py` | One file per test case; each is a call to `testcase.py`'s `nav_test()` or `drive_test()`. |
| `run_tests.launch.py` | The test-runner UI only. |

Test-friendly defaults: headless Gazebo, no rtabmap_viz/RViz/teleop window,
`empty_world.sdf`, spawn at the origin, and a separate RTAB-Map database in
`~/.ros/rover_testing/` so a test never wipes the one rover_bringup maps into. A
`world:=` that is an absolute path is used as is, which is how the tests reach
rover_testing's own worlds.

### Why the pipeline is not a plain include of rover_bringup's

**Startup time.** rover_bringup's `pipeline.launch.py` hard-codes an 8 s wait
before the EKF and RTAB-Map and a 15 s wait before Nav2. rover_testing's pipeline
includes the same component files (so it always tests the current bringup code)
but starts them all at once; nothing needs the wait, since every node idles until
`/clock` and its inputs arrive, and Nav2's costmaps hold activation until the TF
chain exists. The delays are still there as `estimators_delay:=8 nav2_delay:=15`
if you want bringup's staging back. Measured on `nav_empty_world_3_3`:

| | bringup timers | rover_testing |
| --- | --- | --- |
| localization (map → base_footprint) ready | 10.7 s | 4.7 s |
| Nav2 active | 35.6 s | 15.0 s |
| first goal sent | 38.6 s | 15.5 s |

The remaining ~15 s is Nav2's own configure, almost all of it Smac Hybrid-A*
precomputing its Reeds-Shepp heuristic table (`lookup_table_size: 20.0` at
0.05 m and 72 angle bins in `rover_nav2/config/nav2_params_sim.yaml`). It now
overlaps Gazebo's startup instead of queueing behind it. Shrinking that table
would cut it further, but it changes the planner under test, so it is left to
rover_nav2. Drive tests leave Nav2 out and start moving about 10 s after launch,
as soon as the steer and wheel controllers report active.

**Isolation (the headless-mode bug).** ros_gz_sim starts Gazebo through
gz-tools' ruby wrapper, and on shutdown launch signals only the wrapper. The
server ignores SIGINT and SIGTERM; with the GUI it is even forked into a process
group of its own. So a server outlives its launch, and since every Gazebo shares
one gz-transport partition by default, the next launch's bridge talks to both
worlds, and the leftover's in-process `/controller_manager` collides with the new
one on the ROS graph: controllers fail to configure and the rover never moves. A
GUI run followed by a headless run hits this every time. Now (`gz_reaper.py`):

- every pipeline gets its own `GZ_PARTITION` (logged at startup; a `gz sim -g`
  attached by hand needs the same one);
- every process it starts carries a `ROVER_TESTING_RUN=<id>` environment marker,
  and on shutdown a detached reaper SIGKILLs whatever still carries it, group or
  no group;
- the UI runner sweeps the marker itself before starting the next test, and
  before a run offers to kill Gazebo servers left over from elsewhere.

## Adding a test case

1. Add `launch/tests/<name>.launch.py`, a few lines calling `nav_test()` (Nav2
   route, optionally `mapping={"reference": "<maps name>"}`) or `drive_test()`
   from `rover_testing/testcase.py`. A world name found in `worlds/` is used from
   there, anything else from rover_gazebo.
2. Add an entry to `config/test_cases.yaml`.
3. `colcon build --packages-select rover_testing`.

## Files

| File | What it is |
| --- | --- |
| `rover_testing/benchmark_common.py` | Shared benchmark node: startup waits, localized vs ground truth, report, exit code. |
| `rover_testing/goal_benchmark.py` | Nav2 route benchmark, with optional map scoring. |
| `rover_testing/drive_benchmark.py` | Fixed-input drive benchmark. |
| `rover_testing/map_compare.py` | Map loading, saving and scoring; the overlay image. |
| `rover_testing/reference_map.py` | Ideal map from a world SDF (`make_reference_map`). |
| `rover_testing/worldgen.py` | The test worlds, and their ideal maps (`make_test_worlds`). |
| `rover_testing/testcase.py` | `nav_test()` / `drive_test()` for the test launch files. |
| `rover_testing/gz_reaper.py` | Per-run GZ_PARTITION and leftover-process reaper. |
| `rover_testing/test_runner_ui.py` | The Tk UI and the sequential runner. |
