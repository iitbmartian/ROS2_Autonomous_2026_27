# Change log

What changed, and why. Newest first. Each entry names the files touched and the
reasoning behind the change; the "how" lives in the code and the other docs, not here.

## Summary

- **2026-10-14** — Fixed the four real bugs the stress suite found (doc/STRESS_TESTING.md,
  "Fix status"). F1: a wheel's folded steering target could land outside the joint range
  (e.g. after the explicit aim reached its 90 deg clamp, which is a hair past pi/2), and
  the stored angle then stuck near 180 deg, driving the rover sideways in every
  non-explicit mode until a restart. The target now folds only to reachable angles, the
  stored angle is clamped, and the speed is projected on the clamped angle. F2: non-finite
  cmd_vel is dropped instead of clamped (NaN clamped to full speed). F3: the coupling no
  longer captures a twisted rest pose from an uneven spawn (new `rest_capture_max`, 1 deg).
  F4: explicit_pid keeps its wheel integrator at a zero target as a bounded brake (new
  `wheel_hold_max_effort`, 25 N m), and `wheel_ki` goes 2 -> 10. It now holds 25 deg
  slopes, climbs within 4%, and descends within 1.3x.
  Files: `rover_gazebo/rover_kinematics_node.py`, `rover_gazebo/rocker_coupling_node.py`,
  `rover_gazebo/control_node.py`, `config/rover_control.yaml`,
  `doc/STRESS_TESTING.md`, `docs/controls/ROVER_CONTROLS.md` (repo root).
- **2026-10-13** — Added a stress suite, report only: no control or model changes.
  Six generated worlds (`tools/generate_stress_worlds.py`: slopes, side_slope,
  rocks, washboard, low_friction, rough_terrain), seven suites in
  `rover_gazebo/stress_suites.py` (the six terrains plus command abuse on flat), run by
  `tools/run_stress_suite.sh` with each sim isolated in its own ROS domain and Gazebo
  partition. A sim elsewhere on the default domain had spawned a second rover into a test
  world. `stability_test.py` gained teleporting between lanes, scripted command
  programs, a post-abuse recovery window, a survive check (NaN/rollover/runaway/
  fall-through), a controller and node health probe, and PID/coupling torque metrics.
  Three showcase runs are recorded as Gazebo state log + rosbag + plot
  (`tools/plot_run.py`, `tools/odom_tf_replay.py` for RViz). New launch arg
  `record_path`. Findings F1-F8 are in `doc/STRESS_TESTING.md`, most importantly a
  kinematics bug that leaves ackermann driving sideways after an explicit aim reaches
  its 90 deg clamp.
  Files: `tools/generate_stress_worlds.py`, `tools/run_stress_suite.sh`,
  `tools/plot_run.py`, `tools/odom_tf_replay.py`, `rover_gazebo/stress_suites.py`,
  `rover_gazebo/stability_test.py`, `launch/rover_sim.launch.py`, `CMakeLists.txt`,
  `worlds/*.sdf` (6 new), `worlds/terrain/rough.obj`, `doc/STRESS_TESTING.md`.
- **2026-10-12** — Fixed explicit_pid jitter and added a stability suite. The jitter
  was `rocker_coupling_node`, not the PID: at `damping: 250` the coupling broke into a
  self-sustaining ~2-3 g pitch shake whenever the wheels weren't velocity-locked
  (explicit_pid, but also crab/spot/arcs/explicit 45 deg). Its loop closes over a ROS
  topic, so the D term acts on stale data; 50 is calm, 70 and 100 already diverge.
  With the suspension calm, the old PID gains were exposed: steer kp=1 can't beat the
  knuckle's 0.5 N m friction (steering never moved), wheel kp/ki=0.4 ran 19% fast.
  Retuned to steer 60/20/2 (kp/ki/kd), wheel 5/2, sync and chassis-yaw terms off; the
  wheel integrator is cleared at a zero target (front/rear were squeezing +-17 N m at a
  standstill). Crab/spot keep a larger suspension residual (~8-18 deg) at this damping,
  accepted as low priority. New `stability_test.py` + `tools/run_stability_suite.sh`
  check shake, joint tracking, motion and suspension in every mode on flat/ledge/bars.
  Leaving explicit_pid used to leave all 8 steer/drive joints limp until a sim
  restart (gz_ros2_control never removes the `JointForceCmd` effort mode creates, and
  gz-sim's Physics ignores velocity commands while one exists); fixed with a patched
  gz_ros2_control 1.2.20 overlay in `src/third_party/` (see its README).
  Files: `config/rover_control.yaml`, `rover_gazebo/control_node.py`,
  `rover_gazebo/stability_test.py`, `tools/run_stability_suite.sh`, `CMakeLists.txt`,
  `src/third_party/gz_ros2_control/` (vendored, patched `gz_system.cpp`).
- **2026-10-10** — Migrated `cmd_vel` from `geometry_msgs/Twist` to
  `geometry_msgs/TwistStamped` so `command_timeout` measures a command's staleness from
  its own `header.stamp` instead of wall-clock time of receipt. Required a dedicated
  wall-clock `Clock` inside `rover_kinematics_node` (it runs `use_sim_time:true`, but
  every publisher stamps with wall-clock time) and a restamp-on-every-publish fix in
  `measure_rover.py` so a multi-second held command doesn't go falsely stale. Live
  testing against the sim caught and fixed a real crash (`rclpy.Time` subtraction
  across mismatched clock types) before it shipped; verified driving, stale/zero-stamp
  rejection, and `measure_rover.py` all work against a running headless sim.
- **2026-09-24 (11)** — Substantially reduced (not fully fixed) explicit_pid's
  driving yaw under `suspension:=torque` by raising `steer_sync_kp` from 5.0 to 15.0.
  Ruled out more target-tracking authority as the fix first: raising `steer_kp`, and
  separately `steer_ki`, both made yaw measurably worse even with 2026-09-24 (10)'s
  `chassis_yaw_ki` in place — pointing away from reaction torque and toward the
  wheels rolling in different actual directions while their steering hasn't
  converged (inter-joint *disagreement*, not insufficient individual authority) as
  the dominant driving-yaw source. Raising only `steer_sync_kp` (pulls joints toward
  each other, adds no individual authority) fixed it on one run (-0.3 to -5.6 deg,
  down from -45 to -69 deg unbounded) and substantially improved it on a repeat
  (-16 deg peak, recovering afterward instead of continuing to grow). Steering still
  does not reach the commanded aim -- see 2026-09-24 (9), still open.
- **2026-09-24 (10)** — Fixed the yaw component of 2026-09-24 (9): added
  `chassis_yaw_ki` to `control_node.py`, an integral term alongside the existing
  `chassis_yaw_kd`. Root cause was that a D-only term can bound chassis yaw *rate*
  but cannot cancel a small constant bias torque, which just settles at a nonzero
  steady rate instead — confirmed live holding a fixed aim with the wheels
  stationary: yaw rate stayed under chassis_yaw_kd's own verified bound but never
  reached zero, so absolute yaw drifted unboundedly. The integral term fixed it: the
  same hold test now settles into a bounded oscillation instead of growing forever,
  and the full sweep-hold-drive maneuver peaked at -10.2 deg and recovered, instead
  of the unbounded -45 to -69 deg runaway seen with gain changes alone. Steering
  target-tracking itself (the other half of (9)) is still open — raising steer_kp to
  compensate was retested with this fix in place and still made things worse, so it
  needs a different fix, not just a bigger gain.
- **2026-09-24 (9)** — Found, not yet fixed: `suspension:=torque` (the default) plus
  `explicit_pid` fails to reach a commanded steering aim — the knuckle joints jitter
  in a small range well short of target, commanded torque nowhere near the ceiling.
  Root-caused live to `steer_kp=1.0` being too weak against the knuckle joints' real
  0.5 N m Coulomb friction. Raising `steer_kp` or adding `steer_ki` both got the
  joints closer to target but measurably worsened chassis yaw (-45 to -69 deg vs a
  -24.6 deg baseline); disabling `chassis_yaw_kd` entirely changed the yaw almost not
  at all, ruling out the reaction-torque mechanism it targets as the cause here —
  working theory is a kinematic yaw effect from the four wheels rolling in genuinely
  different actual directions while their steering fails to converge. Confirmed this
  is explicit_pid-specific: the identical maneuver under `explicit` (ideal control)
  tracks cleanly with under 2 deg of yaw.
- **2026-09-24 (8)** — Fixed a real bug in `control_node.py`'s `chassis_yaw_kd`:
  Gazebo's OdometryPublisher reports NaN in `/odom`'s angular velocity on ~97.5% of
  messages under `suspension:=mimic`/Bullet-Featherstone (measured, 475 of 487
  sampled), and this was read unguarded. Because of how Python's `min`/`max` resolve
  a NaN operand, that silently snapped all four steering efforts to maximum positive,
  continuously, every time — confirmed live as the exact "all four identical,
  pinned at the ceiling" signature behind `suspension:=mimic`'s `explicit_pid`
  lock-to-hard-limit reports. Fixed by falling back to 0.0 on a NaN sample. Measurably
  better (no more simultaneous four-joint saturation, and the suspension recovered
  after driving stopped in one test) but not a full fix — a separate, deeper
  suspension/steering feedback loop still reproduced the lock in a clean retest; see
  Known follow-up below.
- **2026-09-24 (7)** — Raised `suspension:=mimic`'s `rocker_hold` `max_torque` again,
  2000 to 8000: measured a fresh launch idling with FLS oscillating 13 to 24 deg,
  commands pinned at exactly -2000 repeatedly, not converging — genuine saturating
  chatter, not noise. 8000 fixes that specific failure (converges to a stable point
  instead), but does not fix a deeper, structural finding from the same
  investigation: under real driving load (`measure_rover.py --test spot`), FLS/FRS
  still reached their hard limits (peak chassis roll 19.7 deg vs `suspension:=torque`'s
  1.4 deg on the identical test), because a rigid `<mimic>` constraint — unlike
  `suspension:=torque`'s deliberately soft, yielding 400 N m coupling — has no
  compliance to absorb load differences between wheels, so no value of `rocker_hold`'s
  gains can fix it. Documented as a known, unresolved architectural limitation.
- **2026-09-24 (6)** — Fixed `suspension:=mimic` being non-repeatable and sometimes
  landing FLS_joint right next to its hard limit ("random locking"), reported live: 4
  clean launches captured wildly different rest poses (-0.01, -0.01, +20.9, +21.2 deg).
  Root cause was a constant bias torque this joint's own inertia can't be left free
  against under a rigid `<mimic>` constraint — measured a 40 s trace that never
  stopped drifting — and `rocker_hold_node`'s `max_torque=400` (copied from
  `rocker_coupling_node`'s single-rocker tuning) saturating solid against it during
  settling. Fixed by holding at 0.0 with real stiffness through the settling window
  too, and raising `max_torque` to 2000. Three post-fix launches captured +0.31,
  +0.26 and +5.36 deg — 20+ degrees of margin instead of ~3.
- **2026-09-24 (5)** — Fixed explicit_pid's steering-hold runaway for real: added
  `chassis_yaw_kd` to `control_node.py`, reading actual chassis yaw rate from `/odom`
  and applying a uniform damping correction across all four steering joints. Ruled out
  gain magnitude first (`steer_kd=8` and `steer_max_effort=5`, a quarter of shipped,
  neither stopped it) and an unrealistic simulated yaw inertia (direct measurement
  implied ~223 kg m², large and believable) before concluding the loop was closed
  entirely outside what the per-joint PIDs can see. Measured live: yaw rate dropped
  from reaching -0.68 rad/s and still climbing to staying under ~0.09 rad/s and
  genuinely settling (±2.3° oscillation, not growing) over a 38 s hold-then-release
  test.
- **2026-09-24 (4)** — Added a real `<mimic>`-joint suspension path
  (`suspension:=mimic`), opt-in and Bullet-Featherstone only, alongside the original
  torque-coupling path (`suspension:=torque`, unchanged, still the default and the
  only one DART can run — DART silently ignores `<mimic>`). BLS/BRS/FRS are real
  `<mimic>` joints under the new path, held to ~4 decimal places against their
  CAD-derived ratios with zero control code; FLS gets a new, much-simplified
  single-joint hold, `rocker_hold_node.py`, since mimic constrains the other three
  only relative to FLS and nothing constrains FLS's own position. Both paths verified
  independently, live, on a fresh sim.
- **2026-09-24 (3)** — Fixed explicit_pid being unresponsive on a fresh launch until
  switching modes away and back: `rover/steer_mode` is published only on change, using
  default (VOLATILE) QoS, so `control_node.py` — which connects slightly later than
  most nodes, since it loads a YAML file at startup — could miss the one message
  telling it which mode to be in and silently stay on its startup default forever.
  Fixed with `TRANSIENT_LOCAL` durability on every publisher and subscriber to that
  topic.
- **2026-09-24 (2)** — Fixed a sign bug in explicit_pid's wheel-synchronization term
  that made the mode measurably worse, not better, right after landing: it averaged
  raw joint velocities without correcting for `drive_sign`, so BR (mechanically
  mirrored relative to the other three) was reported as "out of sync" even when
  perfectly synchronized, injecting continuous wrong torque on that one corner —
  violent jitter and a chassis that kept rotating, sometimes a full turn, after the
  steering key was released. Fixed by normalizing through `steer_sign`/`drive_sign`
  before averaging.
- **2026-09-24 (1)** — Fixed explicit_pid making ackermann and explicit jitter more
  too: `control_node.py` ran its `/joint_states` subscription and 200 Hz timer
  unconditionally in every mode, not just explicit_pid — measured pinning it at 100%
  of a core even sitting idle in ackermann, on a physics step already running at
  real_time_factor 1.0 with no slack budgeted. Fixed by creating/destroying both only
  on entry/exit of explicit_pid. Also found, and at the time only partly fixed, a real
  reaction-torque problem specific to explicit_pid itself — see 2026-09-24 (5) for the
  actual fix.
- **2026-09-23** — Added explicit_pid steering mode (teleop key 5): identical movement
  to explicit, but the 4 steer + 4 drive joints are each driven by a new node's own
  independent PID (effort control) rather than the ideal `steer_controller`/
  `wheel_controller`, with `rover_kinematics_node` handing off which ros2_control
  controllers are active whenever the mode crosses the explicit/explicit_pid boundary.
  Sanity-checked live against Gazebo: found the wheel-velocity loop's initial gains
  genuinely unstable (oscillated through zero rather than tracking), fixed by dropping
  the derivative term and lowering the remaining gains — steering tracks tightly,
  wheel speed stably but loosely. Not measured the way the other modes are; see
  VERIFICATION.md.
- **2026-09-22 (4)** — Fixed `teleop_rover_gui.py` intermittently dropping a held key
  (had to press it again to resume). The autorepeat debounce (added in 2026-09-22 (2))
  was set to 40 ms, too close to a typical ~33-40 ms OS key-repeat interval to reliably
  tell a real release from a repeat; raised the default to 150 ms and exposed it as
  `key_release_debounce_ms`. Also gave `teleop_rover_gui` its own block in
  `rover_control.yaml` — its node name never matched the existing `teleop_rover:` block,
  so it had been silently running on script defaults only.
- **2026-09-22 (3)** — Fixed every Python node in this package failing to start under a
  shell with a conda (or other non-system) `python3` earlier on `PATH`
  (`ModuleNotFoundError: No module named 'yaml'`, from `/opt/ros/jazzy`'s `rclpy` loading
  under that interpreter instead of the system one) — previously logged as a known,
  unfixed issue. Hardcoded every script's shebang to `/usr/bin/python3`.
- **2026-09-22 (2)** — Added a Tkinter keyboard teleop window (`teleop_rover_gui.py`)
  alongside the existing terminal one, sharing driving logic through a new
  `teleop_common.py` so the two can't disagree. Fixes W+A-style simultaneous key holds
  silently dropping one key, which a raw terminal can't reliably avoid.
- **2026-09-22 (1)** — Fixed a second, independent bug in the same steering code
  2026-09-21 (5) partly fixed: the fold-to-±90-degrees-and-reverse step ran on the
  published angle only, one tick after the rate limit, so the internal angle could
  overshoot toward its raw target and the published angle would jump ~179 degrees in a
  single tick when that overshoot crossed 90 degrees — reproducing the reported jerk on
  reversing exactly (live-measured peak yaw rate during a reversal: 2.04 rad/s before,
  0.005 rad/s after). Folded before ramping instead of after. Also fixed ackermann's
  yaw-rate ramp, which had been using spot's tuning constant instead of its own.
- **2026-09-21 (5)** — Fixed a real bug behind crab mode's reported jerkiness: the
  solved wheel angle could snap 90 degrees in one tick even though body speed was still
  ramping up from zero. Rate-limited the angle and eased the driven speed off while
  mid-turn. Cut the peak yaw rate during a full crab reversal by 76% (1.632 to
  0.384 rad/s), measured live. Also smooths ackermann and spot, which share the code.
- **2026-09-21 (4)** — Added two off-by-default tuning knobs to `rocker_coupling_node`
  (torque slew-rate limit, velocity low-pass) aimed at the Harmonic jitter; tested five
  configurations live. Neither knob was a clean win: one setting cut jitter sharply but
  cost real holding accuracy, another let the suspension get stuck in a bad pose, and
  the filter made things worse at the one value tried. Reverted to shipped defaults;
  documented the full results and left the trade-off decision open.
- **2026-09-21 (3)** — Isolated the Harmonic jitter to `rocker_coupling_node`'s torque
  loop, not wheel contact tuning; documented it, the Fortress/Harmonic discrepancy, and
  the mimic finding across VERIFICATION.md, both READMEs and DESIGN.md. No behavior
  change: both isolation tests were reverted, pending a decision on how to fix it.
- **2026-09-21 (2)** — Fixed teleop's Shift/hold_time timing, broken by an earlier
  fix in this same log (it had picked up sim time, which can stall at zero or run
  slow); tested live against a running Gazebo Harmonic instance for the first time,
  which also surfaced a real contact-solver jitter and confirmed mimic joints work
  under Bullet-Featherstone but not DART.
- **2026-09-21 (1)** — Fixed a teleop key held across a mode switch leaking into the new
  mode; documented the explicit-mode/crab yaw-drift mechanism in the README,
  integration guide and verification doc. No behavior change to the drift itself.
- **2026-09-20** — Added explicit (swerve) steering mode: aim all four wheels by hand,
  then drive along the aim.

---

## 2026-10-10 — Migrated cmd_vel from Twist to TwistStamped for stamp-based command_timeout

**Why.** `rover_kinematics_node.py`'s `command_timeout` coast-to-stop measured staleness
from wall-clock time of *receipt*, not from when the command was actually generated -- a
message delayed in transit (DDS, scheduling, a slow publisher) looked exactly as fresh
as one sent the instant it arrived. Migrating `cmd_vel` from `geometry_msgs/Twist` to
`geometry_msgs/TwistStamped` adds a `header.stamp` to measure the command's real age
against instead -- the actual point of this migration, not just a type rename.

**Investigation.** Every `cmd_vel` publisher in this package (`teleop_common.py`'s
`drive()`, `teleop_rover.py`/`teleop_rover_gui.py`'s shutdown zero-command,
`measure_rover.py`'s test harness) stamps with its own node's clock, and every one of
them runs wall-clock (`use_sim_time: false` for `teleop_rover`, by existing deliberate
design; no override at all for `teleop_rover_gui`/`measure_rover.py`, so rclpy's default
applies). `rover_kinematics_node` itself runs `use_sim_time: true` so its own clock is
sim time, not wall clock -- comparing a wall-clock stamp against that directly does not
work: sim time starts near zero at launch while a wall-clock stamp carries a full Unix
epoch, so the subtraction comes out large and negative, and a naive clamp-to-zero would
read every message as permanently fresh, silently defeating the whole feature. Added a
second, dedicated `SYSTEM_TIME` `Clock` to `rover_kinematics_node`, used only for this
one comparison; its main timer stays on sim time as before.

Live testing against the headless sim caught a second, real bug before it shipped:
`rclpy.time.Time.from_msg()` defaults to `ClockType.ROS_TIME`, and `rclpy.Time.__sub__`
raises `TypeError: Can't subtract times with different clock types` the moment the two
operands' clock types don't match -- so `self.wall_clock.now() - Time.from_msg(stamp)`
crashed `rover_kinematics_node` outright on the very first real `cmd_vel` message (not
on the zero/never-received path, which never reaches the subtraction). Fixed by passing
`clock_type=ClockType.SYSTEM_TIME` to `Time.from_msg()`, which only relabels the type
rclpy checks -- the stamp was already wall-clock in practice, this does not change the
value compared.

**What.**
- `rover_kinematics_node.py`: `cmd_vel` subscription `Twist` -> `TwistStamped`;
  `on_cmd()` reads `msg.twist.linear.x/y`/`msg.twist.angular.z` and records
  `msg.header.stamp` (not a receipt-time `self.get_clock().now()`). `step()`'s
  staleness check now computes age from the dedicated wall-clock `Clock` against that
  stamp (via `Time.from_msg(stamp, clock_type=ClockType.SYSTEM_TIME)`, to match clock
  types), clamps negative age to 0, and treats a never-received or all-zero stamp as
  infinitely stale (warned once, not every tick).
- `teleop_common.py`'s `drive()`: builds a `TwistStamped` every tick, stamping
  `header.stamp`/`header.frame_id = "base_link"`, fields moved under `.twist`.
- `teleop_rover.py`/`teleop_rover_gui.py`: shutdown zero-command is now a stamped
  `TwistStamped`, not a bare zero `Twist`.
- `measure_rover.py`: publisher and every scripted manoeuvre migrated to `TwistStamped`
  via a new `publish_cmd()` helper that restamps on every publish -- including every
  tick of a `Twist` held across `spin()`'s multi-second loop, which would otherwise go
  stale under the new stamp-based timeout well before the hold itself ends.
- `config/rover_control.yaml`, `doc/INTEGRATION.md`: updated wording/type references.

No backward-compatible `Twist` path kept anywhere -- clean cut-over, as intended.

**Files.** `rover_gazebo/rover_kinematics_node.py` (`on_cmd()`, `step()`, new
`wall_clock`/`last_cmd_stamp`), `rover_gazebo/teleop_common.py` (`drive()`, `cmd`
publisher), `rover_gazebo/teleop_rover.py`, `rover_gazebo/teleop_rover_gui.py`
(shutdown zero-command), `rover_gazebo/measure_rover.py` (`cmd` publisher, new
`publish_cmd()`, `spin()`/`run()`/`run_explicit()`/`sweep_aim()`),
`config/rover_control.yaml` (`command_timeout` comment), `doc/INTEGRATION.md`
(`/cmd_vel` type and driving-it-yourself section).

**Verified.** `colcon build --packages-select rover_gazebo` succeeds with no leftover
bare `Twist` references (`measure_rover.py` still imports `Twist` deliberately, for the
plain velocity object `publish_cmd()` wraps). Live against a headless
(`gui:=false teleop_gui:=false`) launch of `rover_sim.launch.py`: `ros2 topic info
/cmd_vel -v` confirms `rover_kinematics` now subscribes as `geometry_msgs/msg/
TwistStamped`. Publishing a freshly and continuously restamped `cmd_vel`
(`twist.linear.x=0.5`) drove `wheel_controller/commands` to the expected
±0.5/wheel_radius = ±3.3367 rad/s (matching this rover's measured `[+,+,+,-]` drive
signs) with `steer_controller/commands` at 0, as ackermann straight-line driving
should. Publishing the same command but with a static (non-refreshing, eventually
stale) stamp, and separately with an all-zero `header.stamp`, both correctly drove
`wheel_controller/commands` back to zero despite messages arriving continuously at
20 Hz -- the behaviour the old receipt-time check could never produce -- and the
zero-stamp case logged its warning exactly once across ~80 messages, not every tick.
Stopping publication entirely let the last real command's age exceed
`command_timeout` and commands settled back to zero on their own, matching prior
behaviour. `ros2 run rover_gazebo measure_rover.py --test forward --duration 3
--settle 1` ran end-to-end against the live sim (1.40 m forward, 0 deg yaw, 0 m lateral
drift over the full 3 s hold), confirming `publish_cmd()`'s restamping keeps a command
held across `spin()`'s loop from going stale mid-manoeuvre. This same live testing
caught the `Time.from_msg()` clock-type bug noted above, which crashed the node on the
first real message before the fix -- the published commits reflect the fixed version,
already exercised by every test above. Not exercised live in this pass: the Tkinter/
terminal teleop frontends themselves (no display in this environment) and
`explicit`/`explicit_pid`/crab/spot modes specifically -- `teleop_common.drive()`'s
`TwistStamped` construction was read-reviewed but not driven interactively; worth a
manual pass with `teleop_gui:=true` before relying on this in those modes.

---

## 2026-09-24 (11) — Substantially reduced explicit_pid's driving yaw under suspension:=torque via steer_sync_kp, not more individual authority

**Why.** Continuing 2026-09-24 (10): that fix resolved the *pure-hold* yaw drift
(fixed aim, zero drive speed), but the original report (2026-09-24 (9)) was about
driving, not just holding, and the steering target-tracking problem was still open.
Retested the full sweep-hold-drive maneuver with `chassis_yaw_ki` in place and tried
to also fix tracking by adding authority: raising `steer_kp` to 3.0 peaked at -28.6
deg of yaw during driving (worse than `steer_kp=1`'s -10.2 deg peak with the same
fix); separately, adding a modest `steer_ki=0.5` (with `steer_kp` back at 1.0) gave
-27 to -43 deg, still growing after the drive command stopped. Both made yaw worse,
not better, despite the chassis-level integral term from (10) being active in both
cases -- ruling out chassis-level reaction torque as the dominant mechanism during
actual driving and pointing back at the working theory from 2026-09-24 (9): the four
wheels rolling in genuinely different actual directions while their steering hasn't
converged to agreement, a kinematic effect neither `chassis_yaw_kd` nor
`chassis_yaw_ki` touches (both act on reaction torque).

**Investigation.** If inter-joint *disagreement* (not insufficient individual
torque) is the driving-yaw source, the correct lever is `steer_sync_kp` -- it pulls
each joint toward the other three's actual measured position, adding no additional
individual target-tracking authority at all. Tested live, `steer_kp`/`steer_ki` left
untouched at their shipped values: raising `steer_sync_kp` from 5.0 to 15.0 alone
(no other change) held yaw to -0.3 to -5.6 deg over the identical sweep-hold-drive
maneuver that gave -45 to -69 deg unbounded before. A second, independent run
(fresh launch, shipped config) gave a smaller but still real improvement: peaked at
-15.94 deg during driving, then recovered to -10.07 deg after stopping rather than
continuing to grow -- the run-to-run variance in exact magnitude matches the
stochasticity this whole investigation has measured elsewhere (spawn timing,
`suspension:=mimic`'s rest-pose spread, etc.), but the qualitative result --
bounded and recovering, not unbounded -- held both times.

Also confirmed, combining levers: `steer_sync_kp=15` plus a small `steer_ki=0.3` on
top gave -23.6 deg during driving and kept growing to -35.8 deg after stopping --
worse than `steer_sync_kp=15` alone. `steer_sync_kp` and any individual-authority
increase (`steer_kp` or `steer_ki`) do not combine safely; only raising
`steer_sync_kp` on its own is validated.

**What.** Raised `steer_sync_kp` from 5.0 to 15.0 in `config/rover_control.yaml`.
No code changes -- this parameter already existed (2026-09-24 (2)'s fix gave it its
sign-normalized form). `steer_kp`/`steer_ki` left at their original shipped values
(1.0 / 0.0); both were tested and reverted after making yaw worse.

**Files.** `src/rover_gazebo/config/rover_control.yaml` (`explicit_pid_control.steer_sync_kp:
5.0 -> 15.0`, comment explaining why).

**Verified.** Live, `suspension:=torque` (DART, the default), fresh launches,
`explicit_pid`, sweep-hold-drive maneuver (sweep to ~30 deg, hold, drive forward
10 s): `steer_sync_kp=15` alone gave -0.3 to -5.6 deg of yaw on one run and -15.94
deg peak (recovering to -10.07 deg after stopping) on a second, independent, fresh
run -- both categorically different from 2026-09-24 (9)'s unbounded -45 to -69 deg
baseline at the same `steer_kp`/`steer_ki`.

**Known follow-up.** Steering still does not reach the commanded aim under this
combination (joints observed around -2 to -14 deg against a -31 deg target in the
`steer_sync_kp=15` runs) -- `steer_sync_kp` improves *agreement* between the four
joints, it does not add authority to reach the target faster, and every attempt to
add that authority (`steer_kp` or `steer_ki`, with or without `steer_sync_kp=15`)
made yaw worse. 2026-09-24 (9)'s target-tracking problem remains open and needs a
genuinely different mechanism, not a further gain increase on the levers tried so
far.

## 2026-09-24 (10) — Fixed explicit_pid's unbounded yaw drift under suspension:=torque with a chassis_yaw_ki integral term

**Why.** Continuing 2026-09-24 (9)'s investigation: before concluding the yaw
problem needed a bigger steering gain, tested whether it happens even with the
wheels completely stationary (`linear.x=0` the whole time, published continuously
via an explicit zero `Twist`, not just left unpublished — the first attempt at this
test was contaminated by `command_timeout` letting a stale sweep continue for 0.5 s
after publishing stopped). Holding a clean, fixed 24 deg aim with zero drive speed:
yaw rate itself stayed small (under the ~0.09 rad/s `chassis_yaw_kd` was originally
verified to bound, in 2026-09-24 (5)) but never reached zero, so absolute yaw
drifted continuously and unboundedly — -0.17 to -10.29 deg over 12 s, still growing,
not settling to any fixed heading.

**Investigation.** `chassis_yaw_kd` is a pure derivative term: torque proportional
to measured yaw *rate*. Against a small, real, constant bias torque (plausible from
minor mechanical/PID asymmetry between the four steering joints, even when they
share one nominal target), a D-only term settles at whatever nonzero rate makes
`chassis_kd * rate` balance the bias — it dampens acceleration, it does not drive
the rate itself to zero. That nonzero rate then integrates into unbounded absolute
yaw. This is the same lesson `rocker_hold_node.py`'s pre-capture damping
investigation already established for a different joint entirely (2026-09-24 (7)'s
predecessor investigation): a pure D term can only slow a constant bias, never
cancel it — only integral or absolute-position action can.

**What.** Added `chassis_yaw_ki` to `control_node.py`: accumulates
`chassis_yaw_rate` over time (`self.chassis_yaw_integral`, reset alongside the
per-joint PIDs on entry to explicit_pid, same pattern as `steer_ki`/`wheel_ki`
elsewhere in this file), clamped so its own contribution alone can never exceed
`steer_max_effort` (simple anti-windup). `chassis_damp` now combines both terms:
`-(chassis_kd * rate + chassis_ki * integral)`. Shipped at `chassis_yaw_ki: 3.0` in
`config/rover_control.yaml`. Also tried raising `steer_kp` (to compensate for
2026-09-24 (9)'s separate tracking problem) with this fix in place, to see if the
new integral term could absorb the larger reaction it causes — it could not within
a 10 s test: `steer_kp=3` peaked at -28.6 deg of yaw during active driving (worse
than `steer_kp=1`'s -10.2 deg peak with the same fix), though it did stop growing
rather than running away unbounded. Reverted `steer_kp` to its shipped 1.0;
`chassis_yaw_ki` is independent of it and does not require or benefit from raising
it.

**Files.** `src/rover_gazebo/rover_gazebo/control_node.py` (`chassis_yaw_ki`
parameter, `self.chassis_yaw_integral` state and its reset in `on_mode()`, `step()`),
`src/rover_gazebo/config/rover_control.yaml` (`chassis_yaw_ki: 3.0`).

**Verified.** Live, `suspension:=torque` (DART, the default), fresh launches,
`explicit_pid`:
- Pure hold (zero drive speed, fixed 24 deg aim, `chassis_yaw_ki=3.0`): yaw
  overshot to +2.8 deg then settled into a bounded oscillation, -4 to -7.5 deg
  measured over a 25 s window, not growing — versus -0.17 to -10.29 deg and still
  growing over 12 s with `chassis_yaw_ki=0`.
- Full sweep-hold-drive maneuver (sweep to ~30 deg, hold, drive forward 10 s), fresh
  launch, `steer_kp` at its shipped 1.0: yaw peaked around -10.2 deg during driving
  and recovered to -5.3 deg after stopping — categorically different from the
  unbounded -45 to -69 deg runaway measured in 2026-09-24 (9) with the same
  `steer_kp`.
- `suspension:=mimic physics_engine:=gz-physics-bullet-featherstone-plugin`: launched
  cleanly with the new parameter present and readable
  (`ros2 param get /explicit_pid_control chassis_yaw_ki` -> 3.0), no crash or
  regression from adding this code path to a file that mode also uses.

**Known follow-up.** This fixes the yaw component of 2026-09-24 (9)'s report only.
The steering target-tracking failure (knuckles stalling well short of a commanded
aim, likely `steer_kp` too weak against the joints' real Coulomb friction) is still
open, and this entry's own retest confirms raising `steer_kp` is not a safe fix for
it on its own, even with `chassis_yaw_ki` in place.

## 2026-09-24 (9) — Found, not yet fixed: explicit_pid fails to reach a steering aim under suspension:=torque, with a yaw side effect chassis_yaw_kd does not touch

**Why.** Reported live, separately from the suspension:=mimic investigation above:
under `suspension:=torque` (the default, every engine) plus explicit_pid, steering
"jitters constantly and is very unstable and doesn't turn correctly... the wheels
dont turn fully", also causing yaw. Reproduced directly: launched
`suspension:=torque` (no `physics_engine:=` override, i.e. DART), switched to
explicit_pid, swept the aim to ~30 deg and drove forward. All four steering knuckles
(`FLE_joint`/`FRE_joint`/`BLE_joint`/`BRE_joint`) jittered in roughly the -8 to +9 deg
range, never approaching the 30 deg target, and `/steer_pid_controller/commands`
showed small, inconsistent torques (max ~1.7 N m) — nowhere near the 20 N m
`steer_max_effort` ceiling, ruling out saturation as the cause. Chassis yaw drifted
continuously the whole time and kept accelerating even after the drive command
stopped (-0.09 to -24.57 deg over about 8 s of driving in one trace).

**Investigation.** `FLE_joint`'s URDF carries `<dynamics damping="1.0"
friction="0.5"/>` (`urdf/rover_body.xacro`) — real Coulomb friction, independent of
velocity. At `steer_kp=1.0` and a 30 deg (0.524 rad) error, the proportional term
alone commands only ~0.52 N m, barely above that 0.5 N m friction floor, and less as
the error closes — consistent with a joint that stalls partway rather than reaching
target. Tested two ways to add authority:
- Raising `steer_kp` to 6.0 got the knuckles noticeably closer to target (roughly
  -13 to -20 deg against a -30.9 deg target, versus -8 to +9 before) but chassis yaw
  got measurably *worse*, not better (-66.4 deg at a comparable point in the run,
  versus -24.6 deg at `steer_kp=1.0`).
- Adding `steer_ki=1.0` (previously 0.0, an explicit no-op per its own comment) also
  improved tracking (one joint briefly reached -30.1 deg, on target) but again made
  yaw worse (-68.97 deg at a comparable point), and yaw was still accelerating, not
  settling, three seconds after the drive command stopped in both cases.

Ruled out `chassis_yaw_kd` (2026-09-24 (5)'s fix, meant for exactly this class of
symptom) as the mechanism: setting it to 0.0 (fully disabled) and re-running the
identical test gave -47.94 deg at a comparable point — the same order of magnitude
as with it enabled (15.0), not measurably worse or better. `chassis_yaw_kd` damps
the Newton's-third-law reaction from steering *torque*; this yaw does not appear to
be going through that path. Working theory instead: since the four knuckles are not
converging to the same actual angle (the tracking failure above), the four wheels
roll in genuinely different actual directions while driving — a kinematic
yaw-inducing mismatch (comparable to a car with misaligned wheels), not a
reaction-torque effect, which is why fixing (or partially fixing) target-tracking
changes the yaw's character without cleanly fixing it, and why `chassis_yaw_kd`
has so little purchase on it.

Cross-checked against `explicit` (mode 4, ideal `steer_controller`, no PID) on a
fresh launch, identical maneuver (sweep to ~30 deg, hold, drive forward 8 s): aim
reached 28.9 deg cleanly and yaw stayed under 1.7 deg the whole time, settling back
to 1.1 deg after stopping — confirming this is explicit_pid-specific behaviour, not
a shared or previously-documented characteristic of driving with the wheels turned.

**What.** Not fixed yet. All diagnostic parameter changes (`steer_kp: 6.0`,
`steer_ki: 1.0`, `chassis_yaw_kd: 0.0`) were reverted; `config/rover_control.yaml`'s
`explicit_pid_control:` block is back at its previously-shipped values
(`steer_kp: 1.0`, `steer_ki: 0.0`, `chassis_yaw_kd: 15.0`). No code changed.

**Files.** None changed (investigation only). `src/rover_gazebo/config/rover_control.yaml`
(`explicit_pid_control:` block) is the expected locus of an eventual fix;
`src/rover_gazebo/rover_gazebo/control_node.py` if the fix needs a new mechanism
(e.g. something that acts on inter-joint agreement directly, since raising individual
gains alone trades "doesn't reach target" for "worse yaw").

**Verified.** Reproduced and characterized live as described above; not yet resolved.
This suspension:=torque bug is independent of every suspension:=mimic finding in
entries (6)-(8) below — confirmed the two are unrelated: `/odom`'s angular velocity
is never NaN under DART (0 of 472 sampled), so entry (8)'s bug does not apply here.

**Known follow-up.** Next steps to try, not yet attempted: (a) measure the actual
spread between the four knuckle joints' positions over time as a direct proxy for
"wheel direction mismatch" and check whether it correlates with yaw rate more
tightly than with `steer_kp` itself; (b) a modest `steer_ki` (well under 1.0) paired
with unchanged `steer_kp`, to add just enough steady-state authority to clear the
friction floor without the larger transient torques `steer_kp=6` or `steer_ki=1.0`
produced; (c) reconsider whether `steer_sync_kp` (pulls each joint toward the other
three's *measured* agreement) is fighting or helping once individual tracking is
strong enough to actually move — not yet isolated from the gain changes above.

## 2026-09-24 (8) — Fixed a NaN bug in explicit_pid's chassis_yaw_kd that was snapping all four steering joints to maximum effort under suspension:=mimic

**Why.** Continuing the suspension:=mimic investigation from (7): live-testing
explicit_pid specifically (the user's report: it "locks to a high angle and I can no
longer move the rover", unlike ackermann which swings but recovers and explicit which
is worse but still recovers) found `/steer_pid_controller/commands` reporting all
four steering joints commanded to *exactly* the same value, continuously, for
seconds at a time — e.g. `[20.0, 20.0, 20.0, 20.0]` held for 4+ seconds while just
holding a steady aim, not sweeping. That signature — all four identical and pinned
at the ceiling — could not be explained by real tracking error alone. Checking
`/odom` directly (`ros2 topic echo /odom --once`) under `suspension:=mimic
physics_engine:=gz-physics-bullet-featherstone-plugin` found `twist.twist.angular.x/y/z`
reporting `.nan`; sampling 487 messages over 5 s found 475 of them (~97.5%) NaN in
`angular.z` specifically — the exact field `chassis_yaw_kd` (2026-09-24 (5)) reads,
unguarded, every tick.

**Investigation.** `control_node.py`'s `step()` clamps steering effort with
`max(-s_lim, min(s_lim, raw + sync + chassis_damp))`. Python's `min`/`max` compare
their second argument against the first and are not NaN-symmetric: `min(20, nan)`
returns `20`, not `nan` and not the intended "no correction". So a NaN
`chassis_damp` does not cancel or propagate — it silently resolves the entire clamp
to exactly `+s_lim` (verified directly: `max(-20, min(20, 5 + float('nan'))) == 20`
in a Python shell). Since `chassis_damp` is applied identically to all four steering
joints by design (uniform correction, same rationale as the runaway it was built to
damp), a NaN sample pins all four to maximum positive simultaneously — a large,
sustained, *additive* reaction torque on the chassis by Newton's third law, and only
explicit_pid is exposed to it at all, since ackermann/explicit/crab/spot never
subscribe to `/odom`.

**What.** `control_node.py`'s `on_odom()`: guard the assignment with `math.isnan()`
and fall back to `0.0` (a safe "no new information" default, not a stale reading)
instead of assigning `msg.twist.twist.angular.z` straight through. Updated the
module docstring and `on_odom()`'s own comment, which previously described `/odom`
as unconditionally "ground truth... not an estimated or noisy sensor" — true when it
publishes a real number, but no longer accurate as an unqualified claim given this
plugin's measured NaN rate under this specific engine.

**Files.** `src/rover_gazebo/rover_gazebo/control_node.py` (`on_odom()`, module
docstring).

**Verified.** Live, `suspension:=mimic physics_engine:=gz-physics-bullet-featherstone-plugin`,
identical sweep-hold-drive maneuver before and after: before the fix,
`/steer_pid_controller/commands` showed all four joints pinned at the ceiling for
sustained stretches (0 non-saturated ticks was not measured directly, but the
pinned-identical signature was continuous over 4+ s windows); after the fix, 0 of
3758 sampled ticks had all four joints simultaneously at >=19.9 N m, and one full
test showed FLS recovering after the drive command stopped (25.78 down to 17.6 deg
and still falling) rather than staying pinned. This is a real, measured improvement,
not a full fix — see Known follow-up.

**Known follow-up.** A second, independent test (same maneuver, fresh launch, no
contamination from a prior run) still reproduced FLS locking at exactly its 25.78 deg
hard limit for 12+ seconds, with `BRE_joint` (a steering knuckle, not a suspension
joint) diverging to +80 degrees while the other three stayed under 22 degrees — all
four steering joints share an identical axis (`0 -1 0`, confirmed in
`rover_body.xacro`, ruling out a per-joint sign bug. `BRE_joint`'s parent link is
`BRS`, one of the rockers rigidly slaved to FLS via `<mimic>` (ratio +0.577) — as FLS
gets forced toward its limit (2026-09-24 (7)'s finding: a rigid constraint has to put
any load imbalance somewhere, and FLS is the only free coordinate), `BRS` rotates
substantially too, changing the effective gravity loading on `BRE`'s own wheel
assembly enough to make its own independent PID fight harder, adding more reaction
torque onto the chassis, worsening the suspension excursion further. This is a
genuine closed feedback loop between the suspension and the per-joint steering PIDs
under suspension:=mimic specifically, not a single fixable bug, and is not yet
resolved. Options not yet tried: substantially lower reaction-torque budget
specifically under suspension:=mimic, or documenting suspension:=mimic plus
explicit_pid as an unsupported combination for now.

## 2026-09-24 (7) — Raised suspension:=mimic's rocker_hold max_torque again (2000 to 8000) and found the deeper reason a torque ceiling alone cannot fully fix this

**Why.** The 2026-09-24 (6) fix below was verified only at spawn/idle and with one
short scripted drive. Reported live afterward: the same class of symptom recurring,
this time specifically once driving starts, not at spawn — confirmed via question
(rebuilt, correct launch args, ruling out a stale build). A read-only investigation
first (Explore agent) confirmed the (6) fix was intact in source and flagged the
likely gaps: `max_torque=2000` had only been validated against the spawn-settling
transient, never a real driving load, and the fuller `arc/crab/spot/obstacle` test
suite had never been run under `suspension:=mimic` at all.

**Investigation.** Reproduced live on `worlds/bars.sdf`: a fresh launch, sitting
completely idle (no driving, no obstacles nearby — spawn is at x=0, the bars are at
x=1.6/2.6), had FLS oscillating between +13 and +24 degrees indefinitely, with
`/rocker_coupling_controller/commands` pinned at exactly -2000.0 (the ceiling)
repeatedly rather than converging — genuine saturating chatter. Raising `max_torque`
to 8000 fixed that specific run cleanly (converged to a stable point within a few
seconds, held within a fraction of a degree for the next 15). But two more identical
launches (same command, same world) landed the stable point at +17.916 and +17.905
degrees respectively — close enough to each other that this is not chaotic
run-to-run noise, more likely real (non-deterministic) ROS-level startup/controller-
activation timing relative to the physics clock. The steady-state torque needed to
hold one of these off-center points was tiny (~50 N m) — confirming each was a
genuine, low-energy stable equilibrium, not a torque-starved compromise; raising
`max_torque` further would not have moved it.

Then tested an actual maneuver: `measure_rover.py --test spot` (`--yaw-rate 0.7854`),
run right after a launch that had already settled near +18 deg. FLS reached +25.423
deg and FRS reached -25.622 deg — both within a fraction of a degree of their
+/-25.78 deg hard limits — with peak chassis roll 19.7 deg and peak pitch 12.8 deg.
The mimic constraint itself was satisfied almost exactly throughout (worst residual
0.06 deg), ruling out a torque-ceiling explanation for *this* symptom. The identical
test under `suspension:=torque`, same machine, same command: peak roll 1.4 deg, peak
pitch 5.4 deg, rockers settling within a degree of zero. `suspension:=torque`'s own
coupling keeps rocker excursions small specifically because it is deliberately soft
(400 N m, capped so it yields rather than fights, per `rocker_coupling_node.py`'s own
docstring) — it lets its geometric constraint go 1-2.5 degrees "wrong" under load,
and that give is what absorbs load differences between wheels. A rigid `<mimic>`
constraint cannot yield by definition, so any load imbalance a maneuver creates has
nowhere to go except into FLS's own travel, and nothing `rocker_hold_node.py` does —
stiffness, damping, or `max_torque` — touches the constraint itself, only how FLS is
held once the constraint has already forced it somewhere.

**What.** Raised `rocker_hold`'s `max_torque` from 2000 to 8000 N m
(`config/rover_control.yaml`) — a real, measured improvement for the idle-chatter
failure mode, kept. Documented the structural finding (rigid constraint, no
compliance, cannot be fixed by tuning this node) directly in
`rocker_hold_node.py`'s module docstring and the yaml's own comment, rather than
implying a further ceiling increase would resolve it.

**Files.** `src/rover_gazebo/config/rover_control.yaml` (`rocker_hold.max_torque:
2000 -> 8000`), `src/rover_gazebo/rover_gazebo/rocker_hold_node.py` (module
docstring).

**Verified.** Live, headless, `suspension:=mimic
physics_engine:=gz-physics-bullet-featherstone-plugin`, processes fully killed and
confirmed clean between every launch. Idle-chatter fix: one launch went from
oscillating 13-24 deg with commands pinned at -2000, to converging and holding
within a fraction of a degree at 8000. Structural finding: `measure_rover.py --test
spot` under suspension:=mimic (FLS/FRS at their hard limits, 19.7 deg peak roll) vs
identical test under suspension:=torque (rockers near zero, 1.4 deg peak roll) on
the same machine.

**Known follow-up.** This does not fully resolve driving-load excursions under
suspension:=mimic — see 2026-09-24 (8)'s own Known follow-up for the continuation
of this investigation into explicit_pid specifically, where a second, independent
bug (a NaN handling gap) and a further closed feedback loop were both found.

## 2026-09-24 (6) — Fixed suspension:=mimic's non-repeatable, near-limit FLS rest pose

**Why.** Reported live: under `suspension:=mimic`, the rover "behaves very erratic
includes jittering random locking into certain positions." Reproduced directly:
launched headless four times with identical arguments and a completely clean process
state between each (verified with `ps aux` before every launch), and `rocker_hold_node`
captured FLS_joint's rest pose at -0.007, -0.010, +20.863 and +21.216 degrees —
bimodal, not noise, and the +21 deg outcomes leave only about 3 degrees of margin to
the joint's own +/-25.8 deg hard limit (`urdf/rover_body.xacro`'s
`<limit lower="-0.45" upper="0.45".../>`) before any driving load is even applied.
That thin margin, captured as a permanent hold target, is what turns into "random
locking": whichever launch swings hard gets that near-limit pose held for the rest of
the run, and a joint parked 3 degrees from a hard stop chatters against it under load.

First hypothesis, tried and ruled out live: an underdamped spawn transient, since
this joint reflects all four rockers' inertia through the mimic constraint from the
first physics step (FRS at very nearly a 1:1 ratio), unlike `rocker_coupling_node`'s
own four *independent* rockers, each only damping its own much smaller inertia while
free. Added pure velocity-damping (no stiffness, so it could not bias *where* the
joint settled) through the pre-capture window on that theory — it narrowed the spread
(11 deg, 16 deg on two more launches) but did not fix it. Running that same
pure-damping term for 40 seconds instead of 2.5 (temporarily, for this measurement)
showed why: FLS never stopped moving, drifting monotonically down to -20 deg and still
moving at the 40 s mark. That is a genuine constant bias torque, not ringing — pure
damping can only slow the terminal drift velocity (`tau_bias = D * v_terminal`), it
cannot zero a constant torque, so whatever moment `rest_capture_delay` fires just grabs
an arbitrary point along an unbounded ramp.

Checked what was actually being commanded during that window
(`ros2 topic echo /rocker_coupling_controller/commands`): pinned at exactly `400.0`
continuously — `rocker_hold_node`'s `max_torque`, copied from `rocker_coupling_node`'s
own single-rocker tuning (good for roughly 58 N m of quarter-vehicle load on one
independent rocker) but this joint has to react a reflected fraction of *all four*
rockers' load through the mimic constraint, and 400 N m simply wasn't enough to
overcome it.

**What.** Two changes to `rocker_hold_node.py`, both required together — the first
alone (tested first, in isolation) was not sufficient:

- Hold FLS at a provisional target of 0.0, with the same full stiffness+damping PD law
  used after capture, through the whole `rest_capture_delay` window instead of leaving
  the joint fully free (`_pre_capture_hold()`, replacing an earlier damping-only
  `_pre_capture_damp()` that this entry's investigation found insufficient — see Why).
  Anchoring at exactly 0.0 is not a blind CAD-nominal guess here: `suspension:=torque`'s
  own real, measured rest position on this same rover is -0.0007 deg, indistinguishable
  from zero, so it is the value already known to be correct.
- Raised `rocker_hold`'s `max_torque` from 400 to 2000 N m
  (`config/rover_control.yaml`) — enough headroom above the measured ~1200 N m
  transient peak during settling for the PD above to actually win instead of
  saturating against it.

`rest` is still re-captured at the end of the delay exactly as before, preserving the
"hold wherever it actually settles" behaviour the whole capture-based design exists
for — it now just consistently lands near where the anchor already held it, instead of
wherever an unbounded crawl happened to be when the timer fired.

**Files.** `rover_gazebo/rocker_hold_node.py` (`_pre_capture_hold()`,
`pre_capture_hold` parameter, updated module docstring), `config/rover_control.yaml`
(`rocker_hold.max_torque: 400 -> 2000`, `pre_capture_hold: true`, comments).

**Verified.** Live, headless, `suspension:=mimic physics_engine:=gz-physics-bullet-featherstone-plugin`,
processes fully killed and confirmed clean (`ps aux`) between every launch:

- Before the fix: 4 launches, rest captured at -0.007, -0.010, +20.863, +21.216 deg.
- Pure-damping-only variant (superseded, not shipped): 2 further launches, +11.474,
  +16.345 deg — narrower but still not fixed, and the 40 s trace above showed why.
- After both changes: 3 launches, rest captured at +0.312, +0.258, +5.363 deg — every
  one with 20+ degrees of margin to the hard limit instead of ~3.
- Drove `ackermann` (`cmd_vel` linear.x=0.6, angular.z=0.4 for 8 s) after the fix:
  FLS/BLS/BRS/FRS at +3.3/-1.9/+1.9/-3.3 deg mid-drive, comfortably bounded, mimic
  ratios still exact.
- Re-ran `suspension:=torque` (default, unaffected by this change) once as a regression
  check: rest still captures at -0.000/-0.000/-0.000 deg, matching prior measurements.

## 2026-09-24 (5) — Chassis yaw feedback fixes explicit_pid's steering-hold runaway

**Why.** 2026-09-24 (1) reduced explicit_pid's steering reaction-torque kick but
didn't fix the real problem: holding A for a few seconds and releasing it left the
chassis rotating continuously, accelerating, for as long as it was watched (tens of
seconds, sometimes a full turn), well after the steering target had gone completely
still. Investigated as a magnitude problem first, per a direct request to check
whether the sim's mass/inertia was unrealistically low before touching the control
architecture.

**What.** Ruled out magnitude: raising `steer_kd` to 8 (from the shipped 3) and
separately dropping `steer_max_effort` to 5 N m (a quarter of the shipped 20) both
still let the chassis run away, unbounded, past 100 degrees. Ruled out an unrealistic
simulated yaw inertia: applied a known torque (4×15 N m, bypassing the PID entirely)
directly to the steering joints' effort commands for exactly 1 second and measured
the resulting chassis angular velocity change (0.27 rad/s), implying an effective yaw
inertia around 223 kg m² or higher — large, consistent with a genuinely heavy rover,
not a modelling bug. Neither result changed by tuning; what both pointed at instead
was a structural loop: holding a joint's angle rigidly while the chassis rotates
makes the chassis's own rotation drag the joint's *relative* angle off target through
wheel-ground scrub; the PID corrects (correctly, from its own point of view); that
correction's reaction (same non-cancelling axis convention as 2026-09-24 (1)) pushes
the chassis further the same way. None of `steer_kp`/`steer_kd`/`steer_sync_kp` can
see this, because they only ever measure each joint's angle relative to the chassis,
never the chassis's own absolute yaw.

Added `chassis_yaw_kd`: `control_node.py` now subscribes to `/odom` (ground truth
from Gazebo, not an estimated or noisy sensor) only while explicit_pid is active
(same create/destroy-on-mode-change pattern as the existing `/joint_states`
subscription), and applies a torque term proportional to measured chassis yaw rate,
uniformly across all four steering joints, deliberately using the same
reaction-torque path that was causing the problem — as a brake this time, not an
accident.

**Files.** `rover_gazebo/control_node.py` — `/odom` subscription (mode-gated),
`on_odom()`, `chassis_yaw_kd` parameter and the damping term in `step()`;
`config/rover_control.yaml` — `chassis_yaw_kd: 15.0`.

**Verified.** Live, hold-then-release test, same procedure as 2026-09-24 (1)/(2):
before this fix, yaw rate reached -0.68 rad/s and kept climbing; after, yaw rate
stayed under ~0.09 rad/s throughout an 18 s active-then-released window, and a
further 20 s of observation showed it genuinely settled — oscillating within ±2.3
degrees of a fixed point rather than continuing to grow. Sign confirmed correct on
the first try (enabling the term reduced rotation rather than amplifying it). Re-ran
`measure_rover.py --test forward` (ackermann) on a fresh simulator to confirm this
change — which only executes inside explicit_pid's own `step()` — didn't disturb the
other modes (yaw_deg: -0.0, clean). Not a full re-measurement against
doc/VERIFICATION.md's own methodology; that remains open.

## 2026-09-24 (4) — Dual-mode suspension: real `<mimic>` joints under Bullet-Featherstone, opt-in

**Why.** Asked directly: replace the suspension's torque-based constraint
approximation with real `<mimic>` joints, since the real rover's rocker/differential
linkage genuinely is a passive rigid mechanical coupling (unlike the steer/drive
joints, which have their own motors) — matching what doc/VERIFICATION.md already
found in isolation (mimic joints work correctly under Bullet-Featherstone, confirmed
to six decimal places, but DART "explicitly rejects mimic constraints outright" and
silently leaves the follower joint unmoved). Landing this as a straight replacement
would have silently broken DART, the default engine and the only one anything in
doc/VERIFICATION.md was ever measured against — not acceptable, so both paths needed
to exist and both needed to actually work.

**What.** `rover.urdf.xacro` gained a `suspension` argument (default `torque`,
alternative `mimic`), threaded into `rover_body.xacro` (the three follower rockers'
`<mimic>` tags, wrapped in `<xacro:if>`) and `rover_ros2_control.xacro` (which
command interfaces exist: `torque` keeps all four rockers effort-controlled exactly
as before; `mimic` gives only FLS one, since BLS/BRS/FRS are now physics-held).
`suspension:=torque` is unchanged in every observable way — same
`rocker_coupling_node.py`, same `config/controllers.yaml`, same
four-joint/three-constraint torque coupling — and remains the default, so nothing
changes unless `suspension:=mimic` is explicitly requested, and it must be paired
with `physics_engine:=gz-physics-bullet-featherstone-plugin` (a new launch arg,
`gz sim --physics-engine`, independent of the SDF `<physics type=...>` attribute,
which only sets step size/RTF) or DART silently leaves the suspension uncontrolled.

`suspension:=mimic` uses a new `config/controllers_mimic.yaml`
(`rocker_coupling_controller` claims only `FLS_joint`, not all four) and a new node,
`rocker_hold_node.py`: mimic constrains BLS/BRS/FRS only *relative to* FLS, nothing
constrains FLS's own absolute position, and with only joint damping (no stiffness)
resisting it, FLS drifts to its hard stop within seconds if left alone — measured
live. `rocker_hold_node.py` is `rocker_coupling_node.py`'s own rest-pose-capture PD
approach, simplified from four joints/three constraints down to one joint, holding
FLS at wherever it actually settles.

**Files.** `urdf/rover.urdf.xacro` (new `suspension` arg), `urdf/rover_body.xacro`
(conditional `<mimic>` tags on BLS/BRS/FRS), `urdf/rover_ros2_control.xacro`
(conditional command interfaces), `config/controllers_mimic.yaml` (new),
`rover_gazebo/rocker_hold_node.py` (new), `launch/rover_sim.launch.py`
(`suspension`/`physics_engine` args, conditional controller file and node
selection), `CMakeLists.txt`, `config/rover_control.yaml` (new `rocker_hold:`
block).

**Verified.** Live, both paths, on a fresh sim each. `suspension:=torque` (default):
`measure_rover.py --test forward` unchanged from the pre-existing baseline (yaw_deg
0.0, FLS settling ~-5.4 degrees, matching prior measurements). `suspension:=mimic` +
Bullet-Featherstone: mimic ratios held to ~4 decimal places against the CAD-derived
values with zero control code (e.g. BLS measured -13.30 deg against a computed
-0.577165×FLS = -13.2988 deg); a driving test came back with settled constraint
residuals of 0.002-0.003 degrees, roughly 1000x tighter than torque-coupling's usual
0.4-3 degrees; FLS held bounded (not runaway) after the `rocker_hold_node.py` fix,
though its gains are still the ones inherited from `rocker_coupling_node.py`'s own
four-joint tuning and show real jitter (~±5 degrees around the captured rest pose)
not yet re-tuned for the single-joint case.

**Known follow-up.** The fuller test suite (arc/crab/spot/obstacle worlds) hasn't
been re-run under Bullet-Featherstone. The `<mimic>` tags are currently hand-edited
directly into the *generated* `rover_body.xacro`; `tools/generate_description.py`
needs updating to emit them so they survive a CAD regeneration.

## 2026-09-24 (3) — Fixed explicit_pid unresponsive on a fresh launch until switching modes and back

**Why.** Reported directly: starting the sim and switching straight to explicit_pid
(mode 5) produced no response to any command at all; switching to another mode and
back made it work normally afterward.

**What.** `rover/steer_mode` is published only when the mode changes, not
continuously, using plain (VOLATILE) QoS. A subscriber that finishes DDS discovery
after the last publish simply never receives that message — there is no
re-delivery. `control_node.py` connects slightly later than the other nodes in this
graph, since its `__init__` loads `config/rover_kinematics.yaml` from disk before it
gets to creating subscriptions (needed for 2026-09-24 (2)'s fix), which was enough,
on a fresh launch, to lose the race against the very first mode-switch message and
leave it silently stuck thinking it was still in `ackermann` forever — explaining the
"no response at all," since with `control_node.py` never learning of the switch,
neither the ideal controllers (correctly deactivated by `rover_kinematics_node`,
which *did* win its own race) nor the PID ones (never commanded) were driving
anything.

Fixed with `TRANSIENT_LOCAL` durability (a "latched" topic) on the publisher
(`teleop_common.py`'s `mode_pub`) and every subscriber (`rover_kinematics_node.py`,
`control_node.py`), so a late-joining subscriber receives the current mode
immediately regardless of discovery timing. Every other publisher to the same topic
(`measure_rover.py`) had to be updated to match, or its publisher and these
subscribers would never connect to each other at all — QoS durability has to be
compatible, not just correct on one end.

**Files.** `rover_gazebo/teleop_common.py` (`MODE_QOS`, `mode_pub`),
`rover_gazebo/rover_kinematics_node.py` (`MODE_QOS`, `on_mode` subscription),
`rover_gazebo/control_node.py` (`MODE_QOS`, `on_mode` subscription),
`rover_gazebo/measure_rover.py` (`MODE_QOS`, mode publisher).

**Verified.** Live: switched directly into `explicit_pid` as the very first action
after a fresh launch (no mode-cycling), commanded a steering rate, and confirmed the
aim actually moved (17.3 degrees) — previously reproduced as unresponsive under the
identical sequence before this fix.

## 2026-09-24 (2) — Fixed explicit_pid's wheel-sync sign bug, which had made it worse, not better

**Why.** 2026-09-24 (1)'s gang-sync fix looked like a clean win in a headless
regression check (`--test explicit_pid --aim 0`: yaw dropped from 10.94 to 0.23
degrees) but a direct interactive retest told a different story: violent jitter, and
holding then releasing a steering key left the chassis continuing to rotate — now in
the *opposite* direction from the input, sometimes a full turn.

**What.** The wheel-sync term averaged raw joint velocities directly
(`sum(self.vel[j] for j in WHEEL_JOINTS) / 4`) without correcting for `drive_sign`.
`config/rover_kinematics.yaml` gives `FL/FR/BL` drive_sign `+1.0` and `BR` drive_sign
`-1.0` — BR's joint is mechanically mounted mirrored relative to the other three, so
a *perfectly, correctly synchronized* rover driving straight forward has raw
velocities like `FL=FR=BL=+3 rad/s, BR=-3 rad/s`, not all equal. The naive average of
that exact case comes out to 1.5, not 0 — so every wheel got a nonzero "correction"
even when perfectly on target, worst on BR (deviation `1.5 - (-3) = 4.5` vs
`1.5 - 3 = -0.5` for the others), injecting continuous, wrong, asymmetric torque onto
that one corner of the chassis any time the rover had any wheel velocity at all.

Fixed by giving `control_node.py` its own `kinematics_file` parameter (same pattern
`rover_kinematics_node.py`/`rocker_coupling_node.py` already use) to load
`steer_sign`/`drive_sign` per station, then normalizing before averaging and
de-normalizing the correction before adding it back (`norm_j = sign_j * measured_j`,
average in that common frame, `sync_j = sync_kp * sign_j * (avg_norm - norm_j)`) —
provably zero for every joint when the group is already correctly synchronized,
instead of systematically nonzero for whichever joint has the minority sign. The
steering sync term never had this bug (all four `steer_sign` are identical, `-1.0`),
but got the same normalize/de-normalize treatment for consistency rather than
leaving two differently-shaped implementations of one mechanism.

**Files.** `rover_gazebo/control_node.py` (`kinematics_file` param,
`steer_signs`/`drive_signs`, sign-normalized sync in `step()`).

**Verified.** Live: wheels now agree to ~10 decimal places while driving straight
(e.g. `FLD=3.3366700017847575, BLD=3.3366700037412507, BRD=-3.3366700037412715` —
correct opposite sign on BR, matching to the shown precision), chassis yaw stayed
under 0.003 degrees over a 7 s trace versus the pre-fix 11 degrees (and versus the
*buggy*-sync version's reported full rotations). Reproduced the exact hold-then-
release scenario afterward and confirmed it no longer reverses direction from this
specific bug (the separate steering-hold runaway this exposed is 2026-09-24 (5)).

## 2026-09-24 (1) — Fixed explicit_pid making ackermann/explicit jitter more too; found (but only partly fixed) its own reaction-torque problem

**Why.** Reported directly: the rover jittered noticeably more than before, in
every mode, not just the new one — and in explicit itself, violently enough to
physically drift. Specific to explicit_pid: pressing A rotated the whole rover body,
not just the wheel.

**What.** Two separate problems.

Cross-mode jitter: `control_node.py` subscribed to `/joint_states` (published by
`joint_state_broadcaster` at the controller_manager's own 1000 Hz) and ran its own
200 Hz PID timer unconditionally, in every mode, whether or not `explicit_pid` was
even selected. Measured live: this alone pinned the node at 100% of a CPU core
sitting idle in `ackermann`, comparable to `rocker_coupling_node.py` — an unrelated,
unmodified node independently found running just as hot from its own unconditional
`/joint_states` subscription, which is what pointed at the subscription itself
(message deserialization) rather than anything either node's callbacks do.
`worlds/flat.sdf` runs physics at `real_time_factor` 1.0 with no slack budgeted, so
a fourth always-on high-rate Python subscriber on top of the three this package
already shipped was enough to measurably degrade every mode. Fixed by creating the
`/joint_states` subscription and the 200 Hz timer only while `explicit_pid` is
active (created on entry, destroyed on exit, in `on_mode()`), rather than leaving
them running and gating their callback bodies — gating the body alone (tried first)
only got the idle cost down to ~69-100% of a core; tearing the subscription/timer
down entirely was needed to actually get near zero.

Reaction torque: commanding real effort on the steering joints necessarily reacts on
the chassis (Newton's third law), unlike mode 4's ideal *position* command interface,
which Gazebo realizes kinematically with no such reaction. All four steering joints
share one axis convention (`<axis xyz="0 -1 0"/>`) and one `steer_sign` (`-1.0`) — no
left/right mirroring to cancel the four reactions, so they add. The first gains tried
(`steer_kp=40, steer_kd=4`, no rate limit) made the whole rover body visibly spin
while A/D was held. Lowered to `steer_kp=8, steer_kd=1` and added a torque-rate
limiter (`steer_max_effort_rate`, same technique as `rocker_coupling_node`'s own
`max_torque_rate`) — cut the reaction substantially while actively steering, but this
was later found to be far from a complete fix; see 2026-09-24 (5) for what actually
solved the worse problem this partial fix left on release.

**Files.** `rover_gazebo/control_node.py` (mode-gated `/joint_states` subscription
and step timer, `on_mode()` reset-on-entry, lowered `steer_kp`/`steer_kd`,
`steer_max_effort_rate`), `config/rover_control.yaml` (matching gain changes).

**Verified.** Live. CPU: confirmed via `ps`/`top` comparison across modes (see
What. above). Driving: re-ran `measure_rover.py --test forward` (ackermann) and
`--test explicit` on a freshly restarted simulator, matching the pre-existing
baseline (yaw_deg 0.0 both). Steering reaction: live-traced chassis yaw while
holding A — confirmed reduced versus the original gains, but not resolved; see
2026-09-24 (5).

---

## 2026-09-23 — Explicit-PID steering mode (mode 5)

**Why.** Requested directly: a fifth driving mode, identical in feel to explicit (A/D
aim all four wheels together, W/S roll them along the aim, C sweeps to straight,
Space stops without re-centring), but where the 4 steering joints and 4 drive joints
are each closed over their own independent PID loop instead of ros2_control's ideal
`steer_controller`/`wheel_controller`. The point isn't a different movement — it's a
different, tunable, hand-written control path standing in for what a real per-joint
motor controller will eventually do, the same role `rocker_coupling_node` already
plays for the suspension instead of trusting a Gazebo-side constraint.

**What.** `rover_kinematics_node` gained a fifth mode, `explicit_pid`, that runs
through the *exact same* `explicit_step()` mode 4 uses (same aim integration, timeout,
recentre sweep) — the only branch is the last one, which publishes the solved
angle/speed as *targets* (`explicit_pid_control/steer_target`,
`.../wheel_target`) instead of commands, when in this mode. A new node,
`control_node.py`, subscribes to those targets and `/joint_states` and runs 8
independent PID loops (4 position, steering; 4 velocity, drive), publishing effort
commands to two new ros2_control controllers, `steer_pid_controller`/
`wheel_pid_controller` (`effort_controllers/JointGroupEffortController`, spawned
`--inactive`). Both joint groups gained a second, `effort`, command interface in
`rover_ros2_control.xacro` alongside their existing position/velocity one. Since
ros2_control will happily keep two controllers simultaneously active on the same
joint's different command interfaces even though only one can actually be in charge
in Gazebo, `rover_kinematics_node` now also calls the controller_manager's
`switch_controller` service (fire-and-forget, via `add_done_callback` rather than a
blocking call, since it fires from inside a subscription callback on a
single-threaded executor) to deactivate the ideal pair and activate the PID pair —
and reverse it — whenever the mode crosses the explicit/explicit_pid boundary. Added
to `teleop_common.py` (key `5`, `MODE_KEYS`; `drive()`'s explicit branch now covers
both modes, since the keys behave identically) and both frontends' aim readouts/help
text. `control_node.py` is launched unconditionally alongside the other control
nodes, like `rocker_coupling_node` — harmless while its controllers are inactive,
since ros2_control drops commands sent to an inactive controller.

**Files.** `urdf/rover_ros2_control.xacro` — added the `effort` command interface;
`config/controllers.yaml` — registered `steer_pid_controller`/`wheel_pid_controller`;
`rover_gazebo/control_node.py` — new, the 8-joint independent PID node;
`rover_gazebo/rover_kinematics_node.py` — `explicit_pid` mode, target publishers, the
`switch_controller` hand-off; `rover_gazebo/teleop_common.py` — key 5, shared
explicit/explicit_pid branch; `rover_gazebo/teleop_rover.py`,
`rover_gazebo/teleop_rover_gui.py` — help text, aim readout gating, `MODE_LABELS`;
`launch/rover_sim.launch.py` — two `--inactive` spawners and the new node, always
launched; `CMakeLists.txt`, `package.xml` — install the new script,
`controller_manager_msgs` dependency; `config/rover_control.yaml` — new
`explicit_pid_control:` parameter block; `rover_gazebo/measure_rover.py` — new
`explicit_pid` `--test` case, sharing `run_explicit()`/`summarise()` with `explicit`.

**Verified.** Live against a headless Gazebo Harmonic instance (`gui:=false`), not
just read. `ros2 control list_controllers` confirmed the initial state
(`steer_controller`/`wheel_controller` active, the PID pair inactive) and both
directions of the hand-off (`ros2 topic pub .../rover/steer_mode` to `explicit_pid`
and back). Read `/joint_states` directly against the published target to confirm the
PID loops are doing real, physical work, not just updating internal state: steering
settled within roughly 0.03-0.09 rad of a 0.354 rad (20 degree) target and held
there, no oscillation. The wheel loop did not start that clean — the first gains
tried (`kp=8, kd=0.5`) oscillated straight through zero, sign and all, never
settling, traced to differentiating an already-noisy simulated velocity signal at
200 Hz. Dropped `kd` to 0 and lowered `kp`/added `ki`; final shipped gains
(`kp=0.4, ki=0.4, kd=0`) are stable and correctly signed but still carry real ripple,
on the order of the commanded speed's own magnitude — there is real tuning headroom
left, and it is not measured the way the other four modes are in the table below.
Re-ran `measure_rover.py --test forward` on a freshly restarted simulator afterward
to confirm the shared `rover_kinematics_node.py`/`teleop_common.py` edits did not
regress the existing four modes (forward, indistinguishable from before: 0.01 degree
yaw drift over 4 s).

**Known follow-up.** The wheel velocity loop's gains are stable but loose; a proper
tuning pass (or replacing plain PID with something that does not fight simulated
velocity noise, e.g. filtering the measurement rather than the gains) is real
follow-up work, not done here. `doc/VERIFICATION.md`'s explicit_pid section is
deliberately left with an empty results table, the same as explicit mode's — the
numbers above are a sanity check, not a measurement run.

---

## 2026-09-22 (4) — Fixed the GUI teleop intermittently dropping a held key

**Why.** Reported directly, after 2026-09-22 (2) shipped: holding a key in the Tkinter
window sometimes stopped registering, needing a fresh press to resume. Not cosmetic —
`_tick()` republishes `cmd_vel` from live key state every 20 ms, so a key that reads as
released for even one tick is a real gap in the command stream, not just a display
glitch (usually invisible in the rover's actual motion, since `rover_kinematics_node`
ramps speed over `speed_ramp_time`, but the dropped intent is real).

Root cause: the autorepeat debounce that 2026-09-22 (2) added specifically to solve this
class of problem was itself too tight. It schedules a `KeyRelease` to take effect 40 ms
later, cancelling that if a same-key `KeyPress` (an autorepeat) arrives first, so a
genuinely still-held key never reads as released — *if* the repeat arrives inside the
window. Typical Linux autorepeat, once a key is held, fires every ~33-40 ms, right at
that margin, so ordinary Tk scheduling jitter was enough to occasionally miss a repeat
and register a false release.

Separately, re-checking the parameter wiring while diagnosing this found a second,
latent bug: `teleop_rover_gui`'s actual ROS node name (set via the launch file's
`name=` and the class's own `super().__init__(...)`) never matched any block in
`rover_control.yaml` — only `teleop_rover:` existed, which is a different node. The
window had been running entirely on its Python-declared defaults since 2026-09-22 (2);
they happen to match `teleop_rover:`'s values, so nothing looked broken, but any future
edit to that file would silently never have reached it.

**What.** Raised the debounce default from 40 ms to 150 ms — comfortably clear of a slow
repeat-rate setting while still reading as instant on a genuine release — and exposed it
as a declared parameter, `key_release_debounce_ms`, so a particular machine's repeat
rate can be tuned without a code change. Added a `teleop_rover_gui:` block to
`rover_control.yaml`, mirroring `teleop_rover:`'s driving-feel parameters plus the new
one, so the window actually reads its configuration file from now on.

**Files.** `rover_gazebo/teleop_rover_gui.py` (the parameter, `_schedule_release()`
reading it), `config/rover_control.yaml` (the new block).

**Verified.** Directly against the live `TeleopGUI` class, with real elapsed time (not
mocked): a repeat gap of 100 ms — comfortably inside the new 150 ms window, and enough
to have failed under the old 40 ms one — now correctly keeps a key held, re-run through
the same four-part debounce/e-stop suite 2026-09-22 (2) used (plain press, autorepeat
within the window, genuine release, the Space e-stop sequence including suppression and
its release) — all pass. Confirmed the node reads `key_release_debounce_ms` as `150`
from the installed `rover_control.yaml` via a real `rclpy.init(args=[--params-file...])`
load, the same mechanism `ros2 launch` uses. Live: relaunched with `teleop_gui:=true`,
confirmed `teleop_rover_gui.py` starts cleanly with the fix in place.

## 2026-09-22 (3) — Fixed every node failing to start under a conda-shadowed `python3`

**Why.** Reported directly: the GUI teleop window wasn't appearing at all. Traced to
`teleop_rover_gui.py` dying immediately on `import rclpy` with `ModuleNotFoundError: No
module named 'yaml'` — and so were `rover_kinematics_node.py` and
`rocker_coupling_node.py`, silently, every time, independent of anything added today.
Cause: `#!/usr/bin/env python3` resolves via `PATH`, and a shell with conda (or another
non-system Python) active puts that `python3` first — one that `/opt/ros/jazzy`'s
`rclpy` was never built against and that lacks `yaml`. This had already been noted as a
known, unfixed issue in this log's 2026-09-20 entry ("`ros2 run` under an active conda
environment fails to start these nodes at all... pre-existing, unrelated to this change,
not fixed here"); it was worth fixing properly once it was actively blocking real usage
rather than leaving it as a "remember to `conda deactivate`" gotcha.

With `rocker_coupling_node.py` among the crashed nodes, nothing was holding the
suspension's free rocker joint, which plausibly compounds the separate, already-known
Harmonic jitter (see "A jitter that Fortress never showed" and the fold-boundary/jitter
sections of `doc/VERIFICATION.md`) into something worse — investigated as a candidate
cause of a `robot_state_publisher` "Moved backwards in time" warning reported alongside
this, but ruled out: that warning was confirmed, by temporarily `git stash`ing every
change in this package back to the exact original code and relaunching, to be present
on the completely unmodified package too, with or without any node crashing. It is not
addressed here; see `doc/VERIFICATION.md`'s jitter sections for what is already known
about it.

**What.** Changed every installed script's shebang from `#!/usr/bin/env python3` to
`#!/usr/bin/python3` — a fixed path to the system interpreter, bypassing `PATH` (and so
bypassing conda) entirely. Ubuntu 24.04, this package's ROS 2 Jazzy target, ships
`/usr/bin/python3` by default, matching the Python `rclpy` is actually built against.

**Files.** `rover_gazebo/rover_kinematics_node.py`, `rover_gazebo/rocker_coupling_node.py`,
`rover_gazebo/teleop_rover.py`, `rover_gazebo/teleop_rover_gui.py`,
`rover_gazebo/measure_rover.py`, `tools/derive_ratios.py`,
`tools/generate_description.py` (shebang line only, in each).

**Verified.** Live, in the exact shell environment that reproduced the failure
(confirmed `python3` resolved to a conda interpreter lacking `yaml` first on `PATH`):
relaunched `rover_sim.launch.py teleop_gui:=true` with no manual `PATH` override and no
other workaround, and confirmed `rover_kinematics_node.py`, `rocker_coupling_node.py`
and `teleop_rover_gui.py` all start with no traceback, where they previously crashed
every time.

## 2026-09-22 (2) — Added a Tkinter keyboard teleop window, sharing logic with the terminal one

**Why.** Reported directly: holding W and A together to drive a left arc in Ackermann
mode did not produce a clean arc — it behaved as if only A were doing anything, and a
fresh W given shortly after sometimes appeared to be ignored. The mechanism is a real
limitation of raw-terminal keyboard input, not something `hold_time` tuning fully
solves: most terminals only keep auto-repeating one held key at a time, and a terminal
never sends a real key-release event at all — `teleop_rover.py` already has to guess a
key is still held by timing its last repeat against `hold_time` (0.4 s) for exactly this
reason. When a second key is pressed and the terminal stops repeating the first one, that
first key silently ages out of `held` even though it is still physically down, dropping
it from whatever combination was being driven.

Checked what would actually work on this machine before picking a fix: `python3-evdev`
(true hardware N-key-rollover, reading `/dev/input/eventX` directly) is not installed and
this user account is not in the `input` group that device would need; `joy` and
`teleop_twist_joy` are installed, but no gamepad is connected; `tkinter` is already
installed, needs no new dependency or permission, and — like any GUI toolkit under
X11/Wayland — delivers real, independent press and release events per key, which is the
actual property needed here.

**What.** Two new files. `teleop_common.py` pulls everything out of `teleop_rover.py`'s
`Teleop` class that was not actually terminal-specific into a shared base,
`TeleopCore(Node)`: the declared parameters, the `cmd_vel`/`rover/steer_mode` publishers,
the `rover/steer_aim` subscription, `set_mode()`/`stop()`/`recentre_on()`, and `drive()` —
the exact mode-branching Twist-building logic that used to live directly in
`teleop_rover.py`'s `step()`, now taking a plain `(fwd, turn, boost_active)` snapshot
instead of reading a terminal-specific `held` dict. `teleop_rover.py` now subclasses it,
keeping only the raw-terminal reading, the `hold_time` expiry sweep, and the on-screen aim
printout — behavior unchanged, about 60 fewer lines.

`teleop_rover_gui.py` is the new window: same keys, same modes, same topics, but reading
real Tk `<KeyPress>`/`<KeyRelease>` events instead. One gotcha had to be handled, not
skipped: a physically-held key's OS autorepeat still delivers a release immediately
followed by a new press for every repeat, which would otherwise make a genuinely-held
key's tracked state flicker. A release is not applied immediately; it is scheduled 40 ms
out, and a same-key press arriving before that fires cancels it — only a release with no
following press within that window is treated as real. Space and the mode keys (1–4) use
the same real-state tracking to give a genuine e-stop: they force-clear W/A/S/D immediately
and suppress the *next* autorepeat press for each (there will be one, since the physical
key is still down), so the drive stays stopped until an actual release is observed, not
just until the next repeat — unlike the terminal, where the equivalent (`held.clear()`)
gets silently re-populated by the very next repeat event.

Both frontends are kept, not one replacing the other: `teleop_rover.py` for a plain SSH
session with no display, `teleop_rover_gui.py` otherwise. `launch/rover_sim.launch.py`
gained a `teleop_gui` argument (default off, matching `teleop`) that spawns the window as
an ordinary `Node` action (it needs a display, not a TTY, so it does not need
`ExecuteProcess`'s terminal-attached handling the way `teleop` does).

**Files.** `rover_gazebo/teleop_common.py` (new), `rover_gazebo/teleop_rover_gui.py`
(new), `rover_gazebo/teleop_rover.py` (refactored onto the shared base),
`CMakeLists.txt` (installs both new files — `teleop_rover_gui.py` as a program,
`teleop_common.py` as a plain file since it is imported, not run), `package.xml` (declares
the `python3-tk` dependency), `launch/rover_sim.launch.py` (the `teleop_gui` argument and
`Node` action), `README.md` (the "Run it" section now covers all three ways to start
driving).

**Verified.** Live, against a running Gazebo Harmonic instance launched with
`teleop_gui:=true`: the window comes up and its node appears in `ros2 node list`;
`ros2 topic hz /cmd_vel` shows it publishing at ~46.6 Hz, matching the configured
50 Hz `update_rate` closely enough to account for normal Tk/executor overhead. Caught and
fixed a real packaging bug in the process — the new file had been written without the
executable bit, which is not itself part of what `install(PROGRAMS ...)` sets under
`--symlink-install`, so `ros2 launch` failed with `executable 'teleop_rover_gui.py' not
found`; `chmod +x` on the source file and a rebuild fixed it, confirmed by the same launch
succeeding afterward. The autorepeat-debounce and Space/mode-switch e-stop logic
(`_mark`/`_schedule_release`/`_confirm_release`/`_clear_drive_keys`) was exercised
directly, with real elapsed time (not mocked), against the live `TeleopGUI` instance: a
plain press, an autorepeat release-then-press pair within the 40 ms window (stays held), a
genuine release with nothing following (releases), and the full Space e-stop sequence
(clears and suppresses immediately, ignores a simulated continued-autorepeat press,
releases and un-suppresses on a genuine release, registers a fresh press normally
afterward) — all as expected. Not exercised: an actual physical keyboard through a window
manager (no `xdotool`/`wmctrl` available in this session to synthesize real X11 key
events), so the Tk-level `<KeyPress>`/`<KeyRelease>` binding itself, as opposed to the
logic that consumes those events, is unverified live and worth a manual check.

## 2026-09-22 (1) — Fixed a second wheel-angle-fold bug in the same code, plus ackermann's yaw-rate ramp

**Why.** Reported directly: driving in Ackermann mode, pressing S to reverse (or coming
to a stop after reversing) made the steering wheels jump to roughly +90 degrees and then
to roughly -90 degrees in what looked like one physical jerk, changing the chassis's
heading in the process. 2026-09-21 (5) already rate-limited the *solved* wheel angle
(`self.wheel_angle[i]`) so it cannot snap to a fresh target in one tick — but it never
touched the separate "past a quarter turn, point the wheel the other way and roll it
backwards" fold that runs immediately after that rate limit, on the *local* `angle`
variable only, for publishing. That fold is exactly where the jerk was still coming from:
`self.wheel_angle[i]` is free to keep ramping smoothly out past 90 degrees toward its
raw, unfolded target — which can be as far as 180 degrees away, since `atan2(0, vx)` is
exactly 0 for `vx>0` and exactly pi for `vx<0`, a hard discontinuity in the *target* the
instant a reversal crosses zero speed — and the moment that internal, still-ramping value
crosses the ±90 degree line, the *published* angle jumps by the fold amount (~180
degrees) in that single tick, even though the internal value only moved by one ordinary
tick's worth. Confirmed by direct simulation of the real ramp/fold code before touching
anything, across four scenarios (straight reversal, reverse-while-turning,
reverse-then-stop, spot spin-up from a stop): every one showed a 178.85 degree jump in a
single 10 ms tick.

Separately, re-reading the surrounding code while diagnosing this turned up a related but
independent issue: ackermann's commanded yaw rate ramps at a rate computed from
`max_spin_rate`/`steer_ramp_time` unconditionally, even though ackermann's actual range
comes from a different limit, `max_yaw_rate` (only known to `teleop_rover.py` until now).
Ackermann's real lock-to-lock ramp time came out to ~1.02 s, not the documented 0.8 s.

Two further questions came up in discussion and were decided, not acted on as code
changes: A/D alone (no W/S) intentionally does nothing in Ackermann — curvature is
capped by current speed, which is zero at rest, so no steering angle is defined without
some forward or backward speed, mirroring a real Ackermann vehicle; explicit mode (key 4)
already exists to aim the wheels by hand with the drive at zero, so this was left as-is.

**What.** `rover_kinematics_node.py`'s per-station loop now folds the raw solved angle to
whichever of itself or its reverse (wheel pointed the other way, rolling backwards) sits
closer to wherever the wheel's ramped state *already is*, before rate-limiting toward that
folded target — not after. The fold decision is re-made fresh every tick from the wheel's
actual current position, so crossing the ±90 degree boundary now costs one ordinary
`max_wheel_turn_rate * dt` step like any other tick, never a jump. The old post-ramp
`if angle > pi/2` fold block is gone; the sign the driven speed needs falls out of the
projection on its own, since `angle` is already the optimal one by the time it is used.
Also added a `max_yaw_rate` parameter to `rover_kinematics_node.py` (mirroring
`teleop_rover`'s own) and made the yaw-rate ramp use it for ackermann/crab, keeping
`max_spin_rate` only for spot.

**Files.** `rover_gazebo/rover_kinematics_node.py` (the fold-then-ramp rewrite, the new
`max_yaw_rate` parameter, the mode-aware `ang_step`, a docstring addition describing the
fold), `config/rover_control.yaml` (the new parameter, documented).

**Verified.** Standalone, against the real `RoverKinematics` class with no Gazebo (same
technique as 2026-09-21 (5)): re-ran the same four scenarios that showed the 178.85
degree jump before the fix, this time with the fix applied. Max single-tick
published-angle change across all four: reverse-while-turning and
spot-spin-up-from-cold each show exactly one 1.146 degree tick (the ordinary
`max_wheel_turn_rate * dt` step, right at the boundary crossing itself); straight
reversal and reverse-then-stop show zero — no discontinuity anywhere, in any scenario,
where the pre-fix code reproduced the reported jerk exactly every time.

Live, against a running Gazebo Harmonic instance, launched fresh both times (parameters
and code only ever compared between fresh launches, never live-patched, matching how
2026-09-21 (5) measured its own fix): a throwaway `/odom`-subscribing probe commanded the
rover straight in Ackermann mode (`linear.x` +0.8 for 2 s, then flipped straight to -0.8
for 2 s, `angular.z` 0 throughout — reproducing "press S while driving straight") and
recorded `twist.twist.angular.z` around the flip. Pre-fix (the pre-2026-09-22 (1) file,
temporarily swapped back in, rebuilt, and measured from a fresh launch): peak |angular.z|
2.043 rad/s, at 1.82 s after the command flip — a real, large yaw kick, consistent with
the wheels overshooting out toward the raw 180 degree target before the discontinuous
fold. Post-fix (the same file restored, rebuilt, fresh launch): peak |angular.z| 0.005
rad/s over the same window — a 99.7% cut, down to the same low-single-digit-mrad/s level
as the package's already-documented contact-solver jitter baseline, i.e. no longer
distinguishable from noise. Also ran `measure_rover.py --test forward` (sane: -0.01 degree
yaw drift, no lateral drift) and `--test arc_left` (a sustained 197.6 degree turn over 5 s
with real forward and lateral displacement throughout, not a degenerate
steering-only response) post-fix, as a broader regression check alongside the targeted
measurement. Full package rebuild (`colcon build --paths src/rover_gazebo`) succeeds
throughout.



**Why.** Reported directly: crab mode's wheel turning was very quick and visibly
changed the chassis's orientation while it happened. Unlike the previous two entries,
this was not the suspension coupling: `rover_kinematics_node.py`'s per-station loop
ramps body speed (`vx`, `vy`) smoothly toward a new command, but solves the wheel angle
from it fresh every tick via `atan2(vy, vx)`, with no rate limit of its own. Ramping
`vx` and `vy` independently means their *ratio* can swing through 90 degrees in a
single tick while both are still near zero, so a pure sideways crab command from a stop
asked for a full 90 degree wheel snap on the very first tick, well before the body had
picked up any real speed. All four wheels solve the identical angle in crab mode, but
nothing guarantees they physically arrive at a snapped target in lockstep, and that
brief mismatch was the yaw kick.

**What.** The solved angle is now itself rate-limited per wheel
(`max_wheel_turn_rate`, default 2 rad/s, in `rover_control.yaml`), via a new
`angle_ramp()` helper that moves the short way round a wrap point rather than refusing
to move at 179-to-179-degree boundaries. Each wheel's driven speed is now the desired
ground velocity projected onto wherever the wheel is actually pointed
(`wx*cos(angle) + wy*sin(angle)`) rather than its full magnitude, so a wheel still
mid-turn eases off instead of scrubbing sideways at full speed. Ackermann and spot
share this same per-station loop and get the same smoothing as a natural consequence,
not a separate change; crab's sideways-from-a-stop case was simply the one severe
enough to be reported. Explicit mode's handoff (`self.wheel_angle` synced from
`self.steer_aim` on every `explicit_step()` call) keeps a later switch back to
ackermann/crab/spot sweeping from wherever explicit actually left the wheels, not a
stale value.

**Files.** `rover_gazebo/rover_kinematics_node.py` (`angle_ramp()`, the
`max_wheel_turn_rate` parameter, `self.wheel_angle` state, the rewritten per-station
angle/speed solve, the explicit-mode handoff sync), `config/rover_control.yaml` (the
new parameter), `doc/VERIFICATION.md` (the mechanism and the before/after numbers).

**Verified.** Standalone, against a live `rover_kinematics_node.py` with no Gazebo:
nine checks covering a from-rest crab command not snapping on the first tick, a full
crab reversal sweeping smoothly with no single-tick jump anywhere in the trace, driven
speed easing toward zero when a fresh full-magnitude target lands on a wheel still
pointed elsewhere (a case built by pre-ramping `linear.y` while in ackermann mode,
which ignores it, so it arrives already-full the instant crab is selected), ackermann
and spot reaching their correct settled angles unchanged, and a smooth explicit-to-crab
handoff — all passing. Separately confirmed live in Gazebo: commanding a full crab
reversal (`linear.y` +1 to -1) with this fix reverted gave a peak `angular.z` of
1.632 rad/s during the transition; with it restored, 0.384 rad/s, a 76% cut, measured
by temporarily swapping in the pre-fix file, rebuilding, and measuring both from fresh
launches, then restoring the fix and rebuilding again.

## 2026-09-21 (4) — Tried to tune the coupling loop out of ringing; no clean win

**Why.** The previous entry isolated `rocker_coupling_node`'s torque loop as the
dominant source of the Harmonic jitter. The user asked whether the node's own
parameters could reduce it, and asked to see the actual trade-off numbers before
picking a direction.

**What.** Two new parameters, both off by default so shipped behaviour does not change
until tuned: `max_torque_rate`, a ceiling in N·m/s on how fast the published torque may
change (independent of `max_torque`, which only caps its size), and
`velocity_filter_tau`, a one-pole low-pass on the velocity feeding the D term. Both are
implemented in `_filtered_velocities()` and the slew loop in `step()`; filtering is done
once per joint per cycle rather than inline per constraint, since three of the four
joints each appear in two of the three constraints and would otherwise get filtered
twice as fast as intended.

Five configurations were then measured live, each from a fresh launch: the shipped
defaults, `max_torque_rate` at 2,000 / 50,000 / 150,000 N·m/s, and
`velocity_filter_tau` at 0.02 s. None was a clean improvement:

- **2,000 N·m/s** nearly eliminated the jitter but let the suspension settle into a
  visibly wrong pose (residual in the high thirties of degrees, chassis sitting
  noticeably lower) that did **not** recover when the limit was lifted again. This is
  a real hazard, not just an aggressive trade-off, and ruled the value out outright.
- **50,000 N·m/s** cut speed jitter by about 80% but raised the settled differential
  residual from under a degree to roughly ten.
- **150,000 N·m/s** held residual closer to normal but showed jitter worse than the
  unlimited baseline in this one run -- given that two back-to-back runs of the
  identical shipped configuration measured 0.185 and 0.213 m/s speed std dev earlier in
  this same session, that may be measurement noise rather than a real reversal, and
  wasn't re-run enough times to tell.
- **The velocity filter, tried alone at 0.02 s,** made both numbers worse than doing
  nothing, matching the risk flagged before it was built: the loop's problem is that it
  already reacts to stale information, and a filter is more of the same.

Given that, every parameter change was reverted; `rover_control.yaml` ships with both
new knobs at their off defaults, unchanged from before this entry.

**Files.** `rover_gazebo/rocker_coupling_node.py` (the two mechanisms, kept),
`config/rover_control.yaml` (the two new parameters, both at off defaults, kept),
`doc/VERIFICATION.md` (the full results and the run-to-run variance caveat, kept).

**Verified.** Each of the five configurations was measured from its own fresh
`ros2 launch`, not by changing parameters live on an already-running rover: an earlier
live change (tightening `max_torque_rate` on a settled instance, not one of the five
configurations reported above) was what produced the stuck-suspension hazard, which is
itself useful information but not a fair comparison point for the table. Jitter came
from the same `/odom`-recording probe used in the previous entry; residual came from
`measure_rover.py --test forward`, an existing, already-verified metric. After
reverting, confirmed `rover_control.yaml` parses with both parameters at their
documented defaults and the package still builds.

## 2026-09-21 (3) — Isolated the Harmonic jitter; documented it and the mimic finding

**Why.** The previous entry found a continuous speed/yaw jitter on Gazebo Harmonic,
absent from the numbers `doc/VERIFICATION.md` had recorded on Fortress, and a working
`<mimic>` joint under Bullet-Featherstone that DART refuses. The user asked to isolate
the jitter's source before touching anything, and declined to pursue the physics-engine
migration for now. This entry is that isolation, and getting all of it into the docs
rather than leaving it in a chat transcript.

**What.** Two live, reverted experiments against the same plain forward drive used to
first measure the jitter: forcing `rocker_coupling`'s stiffness and damping to zero via
its existing `set_parameters` service, and separately softening the wheel contact
surface (`kp` 100000 to 20000, `kd` 10000 to 2000, `maxVel` 0.1 to 1.0) with the coupling
left at its default. Zeroing the coupling cut speed jitter by roughly 60% and eliminated
yaw-rate noise entirely (0.0000 rad/s std dev, down from 0.0030). Softening the contact
instead cut yaw noise by a similar margin but nearly doubled speed variance and widened
its range in both directions — trading one symptom for a different one, not a fix.
Conclusion: the coupling's own torque loop is the dominant driver of the jitter, not
wheel-ground contact tuning; retuning contact alone will not resolve it.

Both changes were reverted immediately after measuring — the shipped defaults are
untouched. The findings are written up in `doc/VERIFICATION.md`'s new "A jitter that
Fortress never showed" section, with the numbers in a table. `doc/VERIFICATION.md`'s
opening section, both `README.md` copies, and `doc/DESIGN.md` also had their "Gazebo has
no solver-level way to enforce this" and "nothing depends on which Gazebo version you
run" claims corrected: both are no longer quite true, given the mimic finding and the
Fortress/Harmonic discrepancy, and are now stated precisely rather than as a blanket
claim.

**Files.** `doc/VERIFICATION.md`, `README.md` (both copies), `doc/DESIGN.md`. No code
files changed; `urdf/rover_gazebo.xacro`'s wheel contact values were edited for the
second experiment and reverted before this entry was written — `git diff`/a file compare
against the previous entry's state should show no difference there.

**Verified.** All three probe runs (baseline, coupling-off, soft-contact) were against
a live Gazebo Harmonic instance, `/odom` recorded at full rate via a throwaway script in
`/tmp`, not added to this package. Numbers in the table above and in
`doc/VERIFICATION.md` come directly from those recordings. Confirmed after reverting
that `urdf/rover_gazebo.xacro` matches its pre-experiment state and the package still
builds.

## 2026-09-21 (2) — Fixed teleop's use_sim_time regression; live Gazebo findings

## 2026-09-21 (2) — Fixed teleop's use_sim_time regression; live Gazebo findings

**Why.** The user reported, after the previous entry's change, that Shift no longer
worked as intended and the rover's speed seemed to have climbed. That previous entry
had, for the first time, wired teleop's params file (`--params-file` in
`rover_sim.launch.py`) into `teleop_rover.py`, and that file sets `use_sim_time: true`
for the two other nodes it already served — inherited by teleop without a second
thought. Read from the installed rclpy source directly: a ROS-time clock stays pinned
at exactly zero until the node's first `/clock` message, and afterward can advance no
faster than wall time, since every world file caps `real_time_factor` at 1.0. Teleop
times `hold_time` expiry (including the Shift boost window) against `self.get_clock()`,
so this made a held key's effect last longer than 0.4 s in real time, worst case
(no `/clock` reaching the node at all) not expiring at all — exactly "shift no longer
works as intended."

This is the first time this session actually had Gazebo available to test against
(Gazebo Harmonic, gz-sim 8.11, is installed; Gazebo Fortress, which `doc/VERIFICATION.md`
was measured on, is not). Running the real simulation surfaced two further findings
that go beyond this bug fix, reported to the user rather than acted on unilaterally:

- **A contact-solver jitter, independent of any code in this package.** Plain forward
  driving at a commanded 1.0 m/s shows instantaneous speed oscillating between roughly
  0.56 and 1.79 m/s throughout the drive, dozens of times a second, though the mean
  comes out correct (0.997 m/s). The same measurement in crab mode (wheels at 90°)
  is markedly worse: yaw-rate noise about 12× larger, peak speed up to 2.37 m/s. This
  is a real physical effect, not a measurement artifact — confirmed from a raw
  `/odom` time series, with a genuine step-and-decay signature — and is separate from
  and additional to the already-documented suspension-coupling residual. It plausibly
  explains or contributes to the user's reports of a climbing indicated speed, of
  drift compounding over time, and of vibration while steering.
- **Mimic joints work on this stack, but only under Bullet-Featherstone.** Confirmed
  with an isolated two-joint test model: DART explicitly logs
  `"the chosen physics engine does not support mimic constraints"` and the follower
  joint never moves; Bullet-Featherstone enforces `follower = multiplier * leader +
  offset` to six decimal places. Both physics engines are already installed; nothing
  needs to be added. Adopting this for the real suspension would need real validation
  work first: this version's Bullet-Featherstone rejects a joint straight from `world`
  unless it is fixed, and it is unconfirmed whether it accepts the rover's actual
  floating multi-branch chassis, whether `gz_ros2_control`'s hardware interface behaves
  the same under it, and whether the wheel-contact tuning would need to be redone.

**What.** `config/rover_control.yaml`'s `teleop_rover` block now sets
`use_sim_time: false` explicitly, with a comment explaining why it differs from the
other two blocks in the same file.

**Files.** `config/rover_control.yaml`.

**Verified.** Live, against a running Gazebo Harmonic instance for the first time in
this package's session history: the real `Teleop` class, loaded with its real params
file, tapped with a simulated Shift+W. Boost now cuts off at 0.35-0.36 s of real time
(previously would not reliably cut off at all), and holding plain `w` through and past
that point settles the drive at exactly the commanded 1.0 m/s. The contact-jitter and
mimic-engine findings above were captured with throwaway probe scripts in `/tmp`, not
added to this package; no change was made in response to either pending the user's
decision on scope.

## 2026-09-21 (1) — Key-leak on mode switch; documented the coupling-residual yaw drift

**Why.** Testing explicit mode, holding A far more than D and driving with the aim
pinned near 90 degrees produced a visible anticlockwise chassis drift. Investigating it
turned up two separate things.

The drift itself is not a bug: it is the suspension coupling's own residual, already
measured for crab mode (`doc/VERIFICATION.md`: 31.2 deg of yaw in 5 s with the coupling
on, versus 4.2 deg with it off, versus 0.00 deg driving straight), showing up in
explicit mode because pinning the aim near 90 degrees reaches the same wheel geometry
crab mode does. It has nothing to do with the steering joint's own limit — 90 degrees
commanded sits ten degrees inside the 100 degree hard stop, with no saturation
happening. `rover_kinematics_node.py`'s explicit branch was re-audited line by line and
confirmed to never compute or send a body-yaw command; the chassis motion comes from
Gazebo's physics downstream of the code, not from anything the code emits.

Separately, a real bug turned up while checking that code: `teleop_rover.py`'s
mode-switch keys (`1`–`4`) never cleared a currently held key, so a key held across a
switch got reinterpreted under the new mode's semantics for up to `hold_time` (0.4 s)
before it lapsed. Symmetric in A/D and bounded to that narrow window, so it did not
cause the drift above, but worth fixing on its own.

**What.** `press()`'s mode-switch branch now clears `self.held` before changing mode,
matching what the space-bar branch already did. Three docs gained a short addition
explaining the drift mechanism where explicit mode is already discussed; nothing about
how the mode drives changed.

**Files.**
- `rover_gazebo/teleop_rover.py` — `self.held.clear()` added to the mode-switch branch.
- `README.md` (both copies), `doc/INTEGRATION.md`, `doc/VERIFICATION.md` — a paragraph
  each on the coupling-residual mechanism and why it is unrelated to the steering
  joint's travel limit.
- `doc/updates.md` — this entry, and the `## Summary` section above it.

**Verified.** The key-leak fix by re-running the standalone `Teleop`-against-a-live-
`rover_kinematics_node.py` test used to verify explicit mode originally, with an added
case holding `a` across a switch into and out of explicit mode. The drift diagnosis
itself is code review plus the existing crab-mode measurements, not a new Gazebo run —
stated here plainly rather than implied.

## 2026-09-20 — Explicit (swerve) steering mode

**Why.** The rover has four independently steered wheels, but the only modes on offer
asked for a body motion and solved for the wheels. There was no way to simply aim the
wheels by hand and then roll them. A prior edit had started this — `teleop_rover.py`
already carried the mode's docstring, the `4` key, and a `max_steer_rate` parameter —
but `rover_kinematics_node.py` still only knew three modes, so pressing `4` silently
drove as Ackermann. This finishes it.

**What.** A fourth mode, `explicit`. A and D sweep all four wheels together about their
vertical axes at a fixed rate and hold wherever released; W and S spin all four wheels
at one shared speed, scaled only by each wheel's `drive_sign`; C sweeps the wheels back
to straight. No body kinematics are involved — nothing solves for where the rover ends
up.

`cmd_vel.angular.z` carries a steering *rate* (rad/s) in this mode rather than a body
yaw rate, and `rover_kinematics_node` integrates it into one shared angle. That keeps
the unit honest across all four modes and makes the existing command timeout do the
right thing: the rate drops to zero and the drive coasts down, while the aim stays
exactly where it was left. The node echoes the aim on the new `/rover/steer_aim` topic
(`std_msgs/Float64`, output only) so teleop can display it and drive the recentre
without maintaining its own copy that could drift from the node's.

**Files.**
- `rover_gazebo/rover_kinematics_node.py` — `explicit` mode, `explicit_step()`,
  `explicit_max_steer_angle` / `explicit_max_steer_rate` parameters, `/rover/steer_aim`.
- `rover_gazebo/teleop_rover.py` — `C` key, aim readout, recentre sweep, boost scoped to
  W/S only in this mode.
- `rover_gazebo/measure_rover.py` — `explicit` test and `--aim`/`--steer-rate` args:
  sweeps to a target angle via the node's own echo, then drives and reports
  `track_angle_deg` against `aim_reached_deg`.
- `config/rover_control.yaml` — new `explicit_max_steer_angle`/`explicit_max_steer_rate`
  under `rover_kinematics`; new `teleop_rover:` block for keyboard feel.
- `launch/rover_sim.launch.py` — passes `rover_control.yaml` to `teleop_rover.py` under
  `teleop:=true`, so the new block is actually read.
- `README.md` (both copies — the top-level and `src/rover_gazebo/` one are kept byte
  identical), `doc/INTEGRATION.md`, `doc/VERIFICATION.md` — mode table, topic table,
  and a verification section with the method written up and the results left blank.

**Verified.** No Gazebo available in-session. Exercised `rover_kinematics_node.py`
standalone (aim seeding/homing on mode entry, hold-then-release, command timeout, the
90° clamp, the rate ceiling, A/D overriding an in-progress recentre, wheel signs) and
the real `Teleop` class against it (all of the above plus Space-vs-C, boost scoping,
no regression in the other three modes) — 30 checks, all passing. Also dead-reckoned a
frictionless fake rover from the node's own commands and ran `measure_rover.py --test
explicit` through it, confirming the arithmetic: commanded and tracked angle matched at
0°, 45°, 90°, and the clamp correctly refused 120°. None of this touches Gazebo's
physics or the suspension coupling, so `doc/VERIFICATION.md`'s explicit-mode table is
left unfilled until it's run there.

**Known follow-up.** Holding a key in a terminal overshoots the intended angle by
roughly `hold_time × max_steer_rate` (about 6° at the defaults) because the key lapses
rather than releasing; noted in the teleop docstring. `ros2 run` under an active conda
environment fails to start these nodes at all (`ModuleNotFoundError: yaml`, from
`/opt/ros/jazzy`'s rclpy picking up Anaconda's Python) — pre-existing, unrelated to this
change, not fixed here.
