# Change log

What changed, and why. Newest first. Each entry names the files touched and the
reasoning behind the change; the "how" lives in the code and the other docs, not here.

## Summary

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

## 2026-09-21 (5) — Fixed a real wheel-angle snap behind crab mode's jerkiness

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
