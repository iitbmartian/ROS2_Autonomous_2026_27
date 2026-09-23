# Change log

What changed, and why. Newest first. Each entry names the files touched and the
reasoning behind the change; the "how" lives in the code and the other docs, not here.

## Summary

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
