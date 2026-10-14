# Stress testing the controls

`doc/VERIFICATION.md` and the flat/ledge/bars suite answer "does each mode do what it
promises on easy ground". This answers the next question, the one other pipelines
(nav2, the EKF, perception) need answered before they build on top of this: **where do
the controls and the model stop being trustworthy?** Ackermann and explicit_pid are
the focus; every terrain is driven in both.

Everything here is "realistic-hard": slopes up to 25°, rocks no taller than the wheel
radius, 4 cm washboard, friction down to μ 0.2. The real rover is expected to handle
all of it, so a failure is a finding, not a foregone conclusion. Nothing in the controls
or the model was changed to make anything pass. This is a report, and the findings at
the bottom are open.

## Results (2026-10-13, Gazebo Harmonic 8.15, DART, patched overlay)

**Nothing diverged.** Across about 100 scenarios there were no NaN joint states, no
rollovers, no fall-through, no node deaths, and no failed controller hand-off. The
ackermann pipeline (ideal controllers) is solid on every realistic terrain tried.
What fails is narrower: one kinematics bug that any operator can trigger, missing
input validation, a suspension start-up assumption, and an explicit_pid wheel loop
that is too soft once gravity loads it.

| Suite (world) | Pass / scenarios | Notes |
|---|---|---|
| abuse (flat) | 13 / 18 | F1, F2; `abuse_mode_storm` likely F1. `abuse_unknown_mode` failed in the campaign only by inheriting F1's stuck state, and passes in a fresh sim |
| slopes | 11 / 19 | ackermann 10/10; explicit_pid 1/9 (F4) |
| side_slope | 8 / 8 | after moving the strips off the spawn point (F3 was found here first) |
| rocks | 3 / 6 | F5, F7 |
| washboard | 1 / 8 | F6 (shake only, tracking holds) |
| friction | 8 / 9 | explicit_pid rear steering 6° on the μ 0.5 ramp |
| rough | 8 / 9 | slalom suspension residual 8.4°, the known arc limitation (ROVER_CONTROLS §13). One teleport timed out (harness), and the scenario passed on re-run |

### What held

- **Ackermann**
  - Climbs 15°, 20° and 25° at 0.37–0.39 m/s against 0.40 commanded.
  - Parks on 20° and 25° with 0.000 m of creep, and restarts uphill at full commanded wheel speed.
  - Descends at exactly the commanded speed (peak 0.401 m/s).
  - Holds and traverses 12° and 18° side-slopes.
  - Crosses all three rough-terrain lanes with < 0.3° of yaw.
  - Drives μ 0.3, split μ 0.2/1.0 and a μ 0.5 15° ramp, including full-speed reversals on ice.
- **explicit_pid**
  - Crosses rough terrain as well as ackermann.
  - Runs 120 s / 59 m on flat with −0.96° of yaw and 0.24° steering p95.
  - Holds 60 s on flat without moving.
  - Passes low-friction, split-μ and side-slope.
  - Survives 40 explicit ↔ explicit_pid toggles while driving, and full-rate aim sweeps.
- **Commands**
  - The speed cap holds: a 2.5 m/s boost and 1e9 m/s both give 1.0 m/s.
  - The cmd_vel timeout stops the rover within 2 s in both modes.
  - ±1 m/s step reversals every 1.5 s are calm.

### Findings, most severe first

None of these were fixed (report only). Each has a suspected cause and a pointer.

**F1. After the explicit aim reaches its 90° clamp, ackermann/crab/spot drive sideways
until the node restarts.** To reproduce from the keyboard:
1. In explicit mode (4 or 5), hold A or D for about 6 s until the aim stops at 90°.
2. Press 1 and hold W.

The rover moves 1.97 m sideways and 0 m forward with zero yaw, and stays that way in
every non-explicit mode (`abuse_explicit_aim_clamp_then_ackermann`, and offline
against the node's own `step()`). The cause is in `rover_kinematics_node.py`:
1. `explicit_max_steer_angle` (1.5708) sits a hair above π/2 (1.570796).
2. On leaving explicit, each wheel's internal angle is set to the aim.
3. The fold step then sees a straight-ahead target just over 90° away and flips that
   target to π.
4. The internal `wheel_angle` is never limited to the joint range, so it ramps to π.
5. The published angle is clamped to ±90° only *after* the speed has been projected
   onto the unclamped π (lines 365–368). The wheels sit at 90° and roll in reverse.
6. Every later tick folds straight back to π.

The switch itself is violent too. The recording shows a ~178° steering error spike and
a −43 m/s² (4.4 g) IMU jolt at the moment of the mode change, before the sideways drive
settles in. Fixing it means keeping `wheel_angle` inside ±`max_steer_angle`, or folding
against the clamped angle and clamping before projecting. Anything that uses explicit mode
before handing over to ackermann (an operator, or a pipeline that switches modes) can
hit this.

**F2. NaN and inf in `cmd_vel` are acted on, not rejected.** Python's
`max(-top, min(top, nan))` returns `top` (line 296; line 395 is the same for the explicit
steering rate), so:
- NaN `linear.x` drives forward toward 1 m/s: 1.98 m in 4 s.
- NaN `angular.z` in explicit sweeps the aim at full rate to the 90° clamp, which then
  sets up F1.
- `inf` gives full speed at full lock: 2.1 m, −150° of yaw, 9.4 m/s² of shake.

The rover recovers normally afterwards (except via F1). A planner bug upstream would
turn into motion here, not a stop.

**F3. The suspension coupling's rest pose is captured once, 2.5 s after start, from
wherever the rover happens to be.** Spawned straddling the edge of a 12° strip, it
captured +12.9° / +9.1° / +17.5° as "rest" and held the suspension twisted by that much
for the whole session, with the coupling content at ~10 N·m. On every flat spawn it
captures 0.000°. The capture is at `rocker_coupling_node.py:142`. Anything that spawns
the rover on uneven ground (nav2 tests on terrain, a slope spawn) or disturbs it in
those 2.5 s gets a mis-constrained suspension silently. Check the "rest pose captured"
line in the launch log.

**F4. explicit_pid cannot hold or pull on a hill.**
- **Parked:** it slides 0.51 m in 6 s on 20° and 0.39 m on 25°, with nothing commanded
  (ackermann: 0.000 m). It also slid during the setup before that window, so the true
  slide is larger. The wheels roll back at 0.64 rad/s against 3.2 N·m, which is exactly
  kp × error. The integrator is cleared whenever the target is 0
  (`control_node.py:425–430`, the §10.4 fix for flat-ground wind-up), so a parked rover
  has only P to resist gravity.
- **Climbing:** wheels run 16% / 21% / 26% under target on 15° / 20° / 25°, and an
  uphill restart reaches only 50% of target speed.
- **Descending:** the 1 s peak is 0.61 / 0.67 m/s against 0.40 commanded (+53% / +67%).
- **Torque used:** never more than 11 N·m of the 60 available, so this is the loop's
  authority (`wheel_ki` = 2), not motor torque.

**F5. explicit_pid steering is pushed around by terrain.** A rock knocked the
front-left wheel 85° off its target (p95) with the steering effort at its 60 N·m
limit. Yaw drifted −31° / +29° over the rock lanes, and rear steering error reached
6–9° on slopes. The ideal position controller in the other modes showed none of this.

**F6. Washboard shakes the chassis hard.** Vertical acceleration std is 8–12 m/s² at
0.4 m/s and 21–24 m/s² at 0.9 m/s, in both modes. Wheel and steering tracking stay
within limits, and the rover survives and keeps its line. Not established: how much is
bump forcing, and how much is the topic-delayed coupling (ROVER_CONTROLS §8)
amplifying it. A coupling-off comparison would settle it. Either way, perception and
the EKF should expect this level of IMU noise on corrugated ground.

**F7. A 14.4 cm vertical-faced block (0.96 × wheel radius) under one front wheel stops
ackermann.** All four wheels keep turning at the commanded speed while the rover makes
no progress (`ackermann_rocks_dense`, rock_1_55). This is plausibly the real traction
limit of a 4-wheel rocker at μ 1.0, not a controls fault. Compare it with the real
rover's step-climb spec before treating it as a bug.

**F8. Python nodes print tracebacks on Ctrl-C** (`rover_kinematics_node.py`,
`rocker_coupling_node.py`, `control_node.py`). This is cosmetic, but it makes real
errors harder to spot in launch logs.

`abuse_mode_storm` (30 random mode and command changes) ended with the rover yawed
92° and a 49° suspension residual in its recovery window. Its wheel targets had mixed
signs, F1's signature, and random explicit aims followed by mode switches are exactly
F1's trigger. It is attributed to F1 but has not been isolated further.

Raw results are under `stress_runs/<timestamp>/`. This run was `stress_runs/full/`,
plus `full_side_fix/` for the corrected side-slope world.

## The worlds

All generated, seeded and deterministic, by `tools/generate_stress_worlds.py`. Re-run
it after editing it, and never hand-edit the `.sdf` files it writes. Each world is a set
of parallel lanes along +x. The harness teleports the rover to each lane's start
(Gazebo's `set_pose` service), so one sim launch covers a whole world.

| World | Lanes | Stresses |
|---|---|---|
| `slopes.sdf` | y = 0/4/8/12: 10°/15°/20°/25° ramps, 3 m up from x = 2, a 3 m plateau, 3 m down | traction, wheel torque under load, holding on a hill, downhill overspeed |
| `side_slope.sdf` | y = 0: 12°, y = 6: 18° cross-slope strips, x 0..12, rising toward +y | lateral drift, heading hold, roll, suspension under side load |
| `rocks.sdf` | y = 0: 45 rocks over x 1.5..13.5; y = 5: 70 rocks over x 1.5..9.5. Half-buried spheres and tilted blocks, 3–15 cm proud | single-wheel hits, steering disturbance rejection |
| `washboard.sdf` | y = 0: 4 cm bars at 0.25 m pitch; y = 4: 0.40 m pitch, x 2..12 | periodic excitation of the delay-limited suspension loop |
| `low_friction.sdf` | y = 0: μ 0.3 slab; y = 4: split μ (left wheels 0.2, right 1.0); y = 8: 15° ramp at μ 0.5 | wheel slip, PID integrators while slipping, yaw from uneven grip |
| `rough_terrain.sdf` | fractal heightfield 0..0.2 m, x 2..22, y −10..10, tapered to the ground at its edges | continuous uneven support, everything at once |

The rough terrain is an OBJ trimesh (`worlds/terrain/rough.obj`). A `<heightmap>` was
the first plan, but a mesh is the better-trodden collision path in DART. **The mesh
must carry vertex normals.** Without them gz-physics drops the submesh ("does not have
a normal count that matches its vertex count") and DART's ODE collision detector then
segfaults the whole simulator. The generator writes them. Anyone adding terrain meshes
for other tests should do the same.

## Running it

From the workspace root, sourcing `install/local_setup.bash` so the patched
gz_ros2_control overlay is used (`docs/controls/ROVER_CONTROLS.md` §9.1):

```bash
source install/local_setup.bash
ros2 run rover_gazebo run_stress_suite.sh                  # all 7 suites + 3 recordings, ~1 h
ros2 run rover_gazebo run_stress_suite.sh slopes rocks     # just these suites (+ recordings)
RECORD= ros2 run rover_gazebo run_stress_suite.sh abuse    # no recordings
RECORD="rocks:explicit_pid_rocks_dense" ros2 run rover_gazebo run_stress_suite.sh none
```

Results go to `./stress_runs/<timestamp>/` (git-ignored): `<suite>.out` is the
human-readable table, `<suite>.json` has every metric, and `<suite>_sim.log` is the
sim's own log. One scenario against a sim you already have running:

```bash
ros2 run rover_gazebo stability_test.py --suite slopes --only explicit_pid_hold_on_20
```

**Isolation matters.** The runner puts every sim in its own ROS domain (73, or
`$STRESS_DOMAIN_ID`), its own Gazebo partition, localhost only. While this was being
written, a second sim started elsewhere on the default domain spawned its rover into
the test world mid-run. That made explicit_pid look completely broken (steering stuck at
0°), and an isolated re-run passed cleanly. Do the same for any automated test you
write against this sim.

## What is checked

Every scenario, on top of `stability_test.py`'s existing shake / tracking / motion /
suspension checks:

| Check | What it catches |
|---|---|
| **survive** | NaN/inf in joint states or odom, tilt > 60° (rolled over), steering joint > 25 rad/s, wheel > 60 rad/s, body > 3 m/s over any 0.1 s, a drop of > 2 m (fell through). Judged on the whole window, settling included |
| **health** | after the scenario: the expected controller pair is active for the final mode (ideal pair, or the PID pair in explicit_pid), and `rover_kinematics`, `rocker_coupling` and `explicit_pid_control` are all still alive |
| **recover** | abuse scenarios only: switch to ackermann and drive 0.5 m/s straight for 6 s, judged against the *strict* flat-ground limits. Is the rover still normal afterwards? |

New motion kinds: `climb` (≥ 60% of commanded progress up the hill), `descend` (the
same, and the fastest 1 s stretch under 1.3× the command, so gravity is not running it
away), `sideslope` (downhill drift angle within 15°), `clamp` (an oversized command
is held to the 1.0 m/s cap), `dropout` (stops within 2 s of cmd_vel going silent),
`reject` (NaN/inf commands are ignored: < 0.1 m moved). Terrain scenarios use the
existing obstacle limits (shake 6 m/s² / 0.6 rad/s, steering 6°, residual 6°). Hold
scenarios on slopes allow 5 cm and 2°.

Each scenario gets one verdict. **CRASH** means survive failed, health failed, or the
scenario could not run. **FAIL** means a check missed. **PASS** means everything held.
In explicit_pid the JSON also has the PID torques: `steer_effort_absmax`,
`wheel_effort_absmean` per wheel, and `effort_saturated_frac` (share of samples with
any joint at its 60 N·m limit).

## Watching a recorded run

Three showcase scenarios are re-run in a fresh sim with full recording. Three is
deliberate: enough to watch the controls working without filling the disk. The default
set is one working well and two showing open findings:

| Recording | What to watch for |
|---|---|
| `rough__explicit_pid_rough_y+0` | explicit_pid crossing 9 m of rough terrain, steady and straight |
| `slopes__explicit_pid_hold_on_20` | parked mid-way up a 20° ramp with nothing commanded, it rolls back down (F4) |
| `abuse__abuse_explicit_aim_clamp_then_ackermann` | the aim sweeps to 90°, the mode switches to ackermann, and "forward" drives it sideways (F1) |

The bags hold no camera or lidar topics. Joint states and TF are relayed at 100 Hz
(`bag_throttle.py`) rather than the 1 kHz physics rate, and the bag is zstd-compressed.
Each recording lands in `stress_runs/<timestamp>/recordings/<suite>__<scenario>/`:

| File | What it is |
|---|---|
| `gzlog/` | Gazebo's own state log: the exact simulation, every link, every 20 ms |
| `bag/` | rosbag (mcap, zstd): cmd_vel, mode, aim, odom, IMU, joint states and TF at 100 Hz (as `/rec/joint_states`, `/rec/tf`), all controller targets and PID torques |
| `plot.png` | path, roll/pitch, steering and wheel error per joint, IMU shake, suspension residual |
| `result.out` | the scenario's verdict and numbers |

**The 3D replay**, which is what you want for watching: it shows the exact run, with
play/pause/step and a time slider at the bottom of the Gazebo window.

```bash
source install/local_setup.bash
gz sim --playback stress_runs/<timestamp>/recordings/abuse__abuse_explicit_aim_clamp_then_ackermann/gzlog
```

A recording covers the whole session: about 20 s of startup and settling first, then
the scenario. Each one is a fresh sim, so the rover starts at the origin and is
teleported to its lane once the test begins. Use the time slider to skip ahead. The
log references the meshes rather than copying them (Gazebo's `--record-resources`
segfaults on this model on Harmonic 8.15), so source the workspace before playing it.

**The data, in RViz.** Three terminals, from the workspace root, all sourced:

```bash
ros2 bag play stress_runs/<timestamp>/recordings/<run>/bag --clock \
  --remap /rec/joint_states:=/joint_states /rec/tf:=/tf
ros2 run rover_gazebo odom_tf_replay.py --ros-args -p use_sim_time:=true
rviz2 -d install/rover_gazebo/share/rover_gazebo/rviz/rover.rviz --ros-args -p use_sim_time:=true
```

In RViz set Fixed Frame to `odom`, so the body moves rather than just the joints.
`odom_tf_replay.py` exists because the sim's TF tree has no `odom` frame of its own:
ground truth reaches ROS only as the `/odom` topic. `ros2 bag play` has `--rate 0.5`
and the space bar for pause.

**The numbers:** open `plot.png`, or the bag in any mcap viewer.

To record different scenarios, list them as `suite:scenario`:

```bash
RECORD="slopes:explicit_pid_climb_25 rough:ackermann_rough_slalom" \
  ros2 run rover_gazebo run_stress_suite.sh none
```

The Gazebo log is the faithful replay. Re-running the same commands into a fresh sim
does not reproduce a run exactly, because the Python control loops' timing depends on
CPU load.

## Scenario reference

All of them live in `rover_gazebo/stress_suites.py`. The names say mode, manoeuvre
and lane.

- **slopes**: `<mode>_climb_{15,20,25}` from the flat; `<mode>_hold_on_{20,25}`
  parked nose-up mid-ramp with nothing commanded; `<mode>_restart_up_{20,25}` from that
  parked state; `<mode>_descend_{20,25}` from the plateau.
- **side_slope**: `<mode>_hold_side_{12,18}`, `<mode>_traverse_side_{12,18}`.
- **rocks**: `<mode>_rocks_{sparse,dense}`, `ackermann_rocks_arc`,
  `explicit_rocks_aim_20`.
- **washboard**: `<mode>_washboard_{025,040}_v{04,09}` (pitch in cm × 10, speed in
  m/s × 10).
- **friction**: `<mode>_mu03_straight`, `<mode>_mu03_stop_go` (full-speed reversals on
  μ 0.3), `<mode>_split_mu`, `<mode>_climb_15_mu05`, `ackermann_mu03_arc`.
- **rough**: `<mode>_rough_y{-4,+0,+4}`, `ackermann_rough_slalom`, `spot_on_rough`,
  `crab_on_rough`.
- **abuse** (flat):
  - `abuse_step_reversal[_pid]`: ±1 m/s every 1.5 s, 8 times.
  - `abuse_boost_2_5` and `abuse_huge_values` (1e9): checks the speed clamp.
  - `abuse_full_speed_pid`.
  - `abuse_mode_storm`: 30 random mode + command changes, 0.5 s apart.
  - `abuse_pid_toggle`: explicit ↔ explicit_pid 40 times while driving.
  - `abuse_aim_sweep_driving_pid`: full-rate aim sweeps to the ±90° clamp while driving.
  - `abuse_cmd_dropout[_pid]`.
  - `abuse_nan_linear`, `abuse_nan_steer_rate_pid` and `abuse_inf_ackermann`: invalid
    numbers in `cmd_vel`.
  - `abuse_unknown_mode`.
  - `abuse_explicit_aim_clamp_then_ackermann`: hold the explicit sweep until the aim
    clamps at 90°, then ackermann forward (F1, the operator path).
  - `endurance_explicit_pid_120s` (60 m) and `endurance_pid_hold_60s`.
