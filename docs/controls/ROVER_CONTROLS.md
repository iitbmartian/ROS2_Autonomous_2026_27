# Rover Controls Specification

How the four-wheel-steer rocker-differential rover is driven: inputs, kinematics, the
per-joint PID mode, the suspension coupling, mode switching, the tuned numbers and the
reasons behind them, and the tests that decide whether it works.

The Gazebo implementation (`src/rover_gazebo/`) is the reference. This document exists
so that the Unity simulation (`RoverDriveController` and friends) can be kept in sync
with it. Not line for line, since the physics engines differ, but in **behaviour,
conventions, control laws and acceptance criteria**. Where a number is
physics-engine-specific, it is called out as such and the Unity side should re-tune it
against the same tests instead of copying it.

Last updated 2026-10-12, after the explicit_pid stability work.

---

## Contents

1. [The rover model](#1-the-rover-model)
2. [Control architecture](#2-control-architecture)
3. [Operator input (teleop)](#3-operator-input-teleop)
4. [Kinematics node: the common pipeline](#4-kinematics-node-the-common-pipeline)
5. [Modes 1-3: ackermann, crab, spot](#5-modes-1-3-ackermann-crab-spot)
6. [Mode 4: explicit](#6-mode-4-explicit)
7. [Mode 5: explicit_pid](#7-mode-5-explicit_pid)
8. [Suspension coupling](#8-suspension-coupling)
9. [Mode switching and controller hand-off](#9-mode-switching-and-controller-hand-off)
10. [Tuning history: what failed and why](#10-tuning-history-what-failed-and-why)
11. [Porting to Unity](#11-porting-to-unity)
12. [Acceptance tests](#12-acceptance-tests)
13. [Known limitations and open items](#13-known-limitations-and-open-items)
14. [File index](#14-file-index)

---

## 1. The rover model

### 1.1 Frames and conventions

- **Body frame:** ROS REP-103. +x forward, +y left, +z up. Positive yaw is counter-clockwise
  seen from above (turning left).
- **`Twist` semantics** (`cmd_vel`): `linear.x` forward m/s, `linear.y` left m/s,
  `angular.z` rad/s CCW. In explicit modes `angular.z` is **reinterpreted as a steering
  rate** (section 6).
- **Wheel steering angle (kinematic):** 0 = wheel rolling straight forward, positive =
  wheel pointing left. This is the angle all the maths below works in.
- **Joint angle ≠ kinematic angle.** Every joint has a sign, from the CAD:
  `joint_cmd = steer_sign × angle`, `wheel_joint_vel = drive_sign × wheel_speed / r`.
  Getting a sign wrong here is the most common porting bug. Verify every joint
  individually (section 11.6).

### 1.2 Geometry (`config/rover_kinematics.yaml`, generated from CAD)

| Quantity | Value |
|---|---|
| Wheel radius `r` | 0.149850 m |
| Wheel width | 0.140 m |
| Wheelbase | 0.744729 m |
| Track | 0.436 m |
| Ground below chassis origin | 0.72248 m |

| Station | Steer joint | Drive joint | x (m) | y (m) | steer_sign | drive_sign |
|---|---|---|---|---|---|---|
| FL | FLE_joint | FLD_joint | +0.372365 | +0.218 | −1 | +1 |
| FR | FRE_joint | FRD_joint | +0.372365 | −0.218 | −1 | +1 |
| BL | BLE_joint | BLD_joint | −0.372365 | +0.218 | −1 | +1 |
| BR | BRE_joint | BRD_joint | −0.371204 | −0.218 | −1 | **−1** |

Notes:
- **Canonical order is FL, FR, BL, BR** for every array: commands, targets, PID state.
- **BR's drive joint is mounted mirrored** (`drive_sign = −1`). Any code that compares
  or averages wheel speeds across stations **must** first multiply by `drive_sign`.
  An earlier version averaged raw values and injected a continuous wrong torque on BR.
- All four steering joints share one axis convention (`0 −1 0` in the CAD) and one
  `steer_sign`. There is no left/right mirroring, so if all four are torqued the same
  way, their reaction torques on the chassis **add up** instead of cancelling.

### 1.3 Joint limits and dynamics

| Joint | Range | URDF limit | Joint damping / friction |
|---|---|---|---|
| Steer (×4) | ±1.75 rad (hard stop ≈ 100°) | effort 500 N·m, vel 20 rad/s | damping 1.0 N·m·s/rad, **friction 0.5 N·m** |
| Drive (×4) | continuous, velocity cmd ±30 rad/s | — | — |
| Rocker FLS/FRS/BLS/BRS | passive, coupled (section 8) | — | — |

- Steered assembly: knuckle 0.33 kg plus wheel 9.87 kg. The wheel's inertia about the steer
  axis is ≈ 0.072 kg·m².
- Contact (`rover_gazebo.xacro`): μ = 1.0 isotropic, kp 1e5, kd 1e4, minDepth 1 mm.

**The steering friction of 0.5 N·m is a control-design input.** Any steering controller must
command well over 0.5 N·m (plus tyre scrub) at small errors, or the wheel sticks.
See section 10.2.

### 1.4 The suspension

A rocker-differential. Each side has a front rocker (FLS/FRS) and a rear rocker
(BLS/BRS) linked by a connector bar (a four-bar), and a differential links the two rear
rockers. These are **closed kinematic loops that URDF/articulations cannot represent**.
They are replaced by linear constraints between joint angles, fitted from CAD
(fit error < 0.4°):

```
left four-bar   BLS = −0.577165 × FLS
right four-bar  BRS = −0.572734 × FRS
differential    BLS + BRS = 0
```

That leaves one free degree of freedom, as on the real mechanism. Section 8 covers how
they're enforced.

---

## 2. Control architecture

```
 keyboard ─► teleop (teleop_common.py) ──cmd_vel (Twist)──┐
                 │                                         │
                 └── rover/steer_mode (latched String) ──┐ │
                                                         ▼ ▼
                                   rover_kinematics_node.py  (100 Hz)
                                   ├─ timeout, ramps
                                   ├─ modes 1-4 ──► steer_controller   (position, ideal)
                                   │               wheel_controller   (velocity, ideal)
                                   ├─ mode 5   ──► explicit_pid_control/steer_target
                                   │               explicit_pid_control/wheel_target
                                   ├─ rover/steer_aim (echo of the explicit aim)
                                   └─ controller hand-off on 4/5 boundary
                                                         │
                         control_node.py (200 Hz) ◄──────┘  (mode 5 only)
                         per-joint PID ──► steer_pid_controller / wheel_pid_controller (effort)

                         rocker_coupling_node.py (500 Hz, every mode)
                         constraint spring-damper ──► rocker_coupling_controller (effort)

 Gazebo + gz_ros2_control (1 kHz physics, controller_manager inside the sim)
 /joint_states (1 kHz), /odom (ground truth, 100 Hz), /imu/data (100 Hz)
```

Rules that matter:

- **Exactly one control law owns each joint at any time.** Modes 1-4 use the ideal
  position/velocity controllers. Mode 5 uses the effort controllers driven by
  `control_node.py`. The hand-off happens atomically, in a single controller-manager
  switch call (section 9).
- **`rover/steer_mode` is published only on change**, so it needs latched
  (TRANSIENT_LOCAL) QoS on *every* publisher and subscriber. Otherwise a late-joining
  node never learns the mode. The Unity equivalent: the mode is state, not an event.
  Any component that starts late must read the current value.
- `control_node.py` creates its `/joint_states`, `/odom` subscriptions and timer **only
  while mode 5 is active**. A 1 kHz Python subscriber left running costs a full CPU core and
  measurably worsened jitter in the other modes.

---

## 3. Operator input (teleop)

Shared by `teleop_rover.py` (terminal) and `teleop_rover_gui.py` (Tk window) via
`teleop_common.py`. Each tick (50 Hz) the frontend reduces the keys to
`fwd ∈ [−1,1]` (W − S), `turn ∈ [−1,1]` (A − D) and `boost` (Shift), then:

| Mode (key) | linear.x | linear.y | angular.z |
|---|---|---|---|
| 1 ackermann | fwd · max_speed · boost | 0 | turn · max_yaw_rate |
| 2 crab | fwd · max_speed · boost | turn · max_speed · boost | 0 |
| 3 spot | 0 | 0 | turn · max_spin_rate · boost |
| 4 explicit / 5 explicit_pid | fwd · max_speed · boost | 0 | turn · max_steer_rate (**steering rate**, no boost) |

| Key | Action |
|---|---|
| 1-5 | select mode (publishes latched `rover/steer_mode`) |
| W/S | forward / back |
| A/D | left / right (meaning depends on mode, table above) |
| Shift | boost ×2.5 on drive speed only |
| C | explicit modes: sweep the aim back to 0 at max_steer_rate |
| Space | stop, cancel recentre |

Teleop parameters (`config/rover_control.yaml`): max_speed 1.0 m/s, max_yaw_rate
1.0 rad/s, max_spin_rate 0.7854 rad/s, max_steer_rate 0.2618 rad/s (15°/s, deliberately
slow), boost 2.5, update_rate 50 Hz, hold_time 0.4 s (terminal key-repeat expiry),
key_release_debounce_ms 250 (GUI).

Teleop runs on **wall clock**, not sim time. Its key timing must not stretch or freeze
with the simulation clock.

---

## 4. Kinematics node: the common pipeline

`rover_kinematics_node.py`, 100 Hz on sim time. Every tick:

0. **Input validation.** A `cmd_vel` with any NaN or inf component is dropped and
   logged. It is not clamped: `max(-top, min(top, nan))` evaluates to `top`, so NaN used
   to mean full speed (stress test F2). The last good command then ages out via the
   timeout below.
1. **Command timeout.** If no `cmd_vel` arrived for `command_timeout` = 0.5 s, the wanted
   command becomes all zeros, so the rover coasts to a stop. In explicit modes this zeroes
   the steering *rate*, so the wheels stay pointed where they were left.
2. **Speed ramp:** `have.x`, `have.y` move toward the wanted values by at most
   `max_speed / speed_ramp_time × dt` (1.0 / 1.2 s), i.e. 0 → full in 1.2 s.
3. **Yaw-rate ramp** (modes 1-3): `angular.z` moves by at most
   `2 × range / steer_ramp_time × dt`, where range = max_spin_rate in spot and max_yaw_rate
   otherwise, and steer_ramp_time = 0.8 s lock-to-lock.
4. Branch on mode.

`ramp(cur, tgt, max)` clamps the step. `angle_ramp` does the same along the short way round
the circle, and keeps the result in (−π, π].

Kinematics parameters: max_speed 1.0 m/s, min_turn_radius 0.8 m, max_steer_angle
1.5708 rad, max_yaw_rate 1.0 rad/s, max_spin_rate 0.7854 rad/s, max_wheel_turn_rate
2.0 rad/s, explicit_max_steer_angle 1.5708 rad, explicit_max_steer_rate 1.0 rad/s,
speed_ramp_time 1.2 s, steer_ramp_time 0.8 s, command_timeout 0.5 s, update_rate 100 Hz.

---

## 5. Modes 1-3: ackermann, crab, spot

All three use **one velocity-field formula**. Each wheel gets the ground velocity its
station would have under the commanded body motion:

```
v_wheel = v_body + ω × r_station      →   (wx, wy) = (vx − ω·y, vy + ω·x)
```

| Mode | (vx, vy, ω) used |
|---|---|
| ackermann | (vx, 0, ω_capped), where ω is capped to \|vx\| / min_turn_radius (curvature cap, so the turn never asks for a tighter arc than the steering reaches) |
| crab | (vx, vy, 0): every wheel parallel, the body translates and never yaws |
| spot | (0, 0, clamp(ω, ±max_spin_rate)): pure spin |

Then, **per wheel** (this ordering matters, it is the result of two bug fixes):

1. `raw = atan2(wy, wx)`
2. **Fold before rate-limiting.** If `raw` is more than 90° from the wheel's *current* angle,
   use `raw ± π` instead (point the wheel the other way and roll it backwards). This
   keeps every wheel within a quarter turn of square-on. It is done before the
   rate limit, so the ±90° boundary never causes a same-tick jump.
3. **Rate-limit the angle.** `angle_ramp(current, folded, max_wheel_turn_rate × dt)`, at
   2 rad/s. Without this, a crab command from rest asks for a 90° snap on the first tick,
   and any lockstep mismatch between the four wheels kicks the chassis in yaw.
4. **Keep it reachable.** If the folded target lies beyond ±max_steer_angle, use the
   other candidate (target ∓ π), and clamp the *stored* angle to ±max_steer_angle after the
   rate limit. Without this a wheel left at the explicit aim's 90° clamp (1.5708, a hair
   past π/2), or crab going right → diagonal, folds to an unreachable ~180°. The state then
   sticks there and the rover drives sideways in every mode (stress test F1, fixed
   2026-10-14).
5. **Project the speed onto where the wheel actually points**, which is the clamped
   angle: `speed = wx·cos(angle) + wy·sin(angle)`. A wheel still mid-turn eases off
   instead of scrubbing. If the fold picked the reverse direction, the sign comes out
   negative by itself.
6. Publish `steer_sign·angle` (position) and `drive_sign·speed / r` (velocity).

Ackermann geometry note: front and rear counter-steer (the formula does this for you),
which roughly halves the turning circle. Reversing with yaw commanded yaws the rover in
the commanded direction (standard `Twist`), not car-style.

---

## 6. Mode 4: explicit

The inverse idea: **you aim the wheels and the body goes wherever they take it.** No body
kinematics.

- `angular.z` is a **steering rate**, clamped to ±explicit_max_steer_rate (1.0 rad/s),
  and integrated into one shared aim:
  `aim = clamp(aim + rate·dt, ±explicit_max_steer_angle)`.
- `linear.x` (after the speed ramp) is one wheel speed for all four.
- Output: every steering joint gets `steer_sign·aim`, and every wheel gets
  `drive_sign·speed / r`.
- **Entry sweep.** On entering explicit/explicit_pid *from modes 1-3*, the aim is seeded
  from the front-left wheel's current angle and then swept to 0 at
  explicit_max_steer_rate (`zeroing`). Any A/D input during the sweep takes over. Swapping
  between 4 and 5 does **not** re-trigger it, because both share the same aim.
- The aim is echoed on `rover/steer_aim` (Float64, rad) for teleop and tests.
- On the way back to modes 1-3, the per-wheel state is set to the aim, so they sweep
  from where the wheels really are.
- Explicit's joint limit margin: the joints stop hard at ±1.75 rad. The aim clamp is
  1.5708, which leaves margin so the joints do not chatter against the stop.

Expected behaviour: aim α, drive forward, and the rover translates at ≈ α relative
to its heading, with ≈ 0 yaw (measured: 45° aim → 44.1° track).

---

## 7. Mode 5: explicit_pid

**Same movement as mode 4.** `rover_kinematics_node` runs the identical `explicit_step()`.
The difference is only who actuates the joints. The per-joint targets go to
`control_node.py`, which closes **one independent torque loop per joint**: 4 position
loops for steering and 4 velocity loops for the wheels. This models a rover where every
steering and drive motor is its own servo, and exposes the dynamics the ideal
controllers hide.

### 7.1 Loop structure (200 Hz, sim time)

For each joint `j`, every step:

```
error = target_j − measured_j            # steer: position (rad); wheel: velocity (rad/s)

# derivative
steer:  d = −kd · measured_velocity_j    # derivative on measurement, no kick when the aim moves
wheel:  d =  kd · (error − prev_error)/dt  # kd = 0 for wheels (noisy signal), so unused

# wheel only: a stopped wheel keeps integrating (the integral of its speed error is
# how far it has rolled, so it acts as a brake), but the hold is bounded
if wheel and target_j == 0:  integral = clamp(integral, ±wheel_hold_max_effort / ki)

# conditional-integration anti-windup
candidate = kp·error + ki·integral + d
if |candidate| < limit  or  candidate·error < 0:   integral += error·dt

raw = clamp(kp·error + ki·integral + d, ±limit)

# optional extra terms (both shipped at 0, see 7.3)
sync  = sync_kp · sign_j · (mean_k(sign_k·meas_k) − sign_j·meas_j)
yaw   = −chassis_yaw_kd · chassis_yaw_rate     # steering only, same on all 4

out   = clamp(raw + sync + yaw, ±limit)
out   = rate_limit(prev_out, out, max_effort_rate·dt)
publish effort  (FL, FR, BL, BR)
```

On **entering** mode 5 all integrators, previous errors and previous outputs reset to
zero, so state from an earlier session cannot cause a kick.

### 7.2 Gains (shipped, `config/rover_control.yaml` → `explicit_pid_control`)

| Parameter | Value | Why |
|---|---|---|
| steer_kp | **60** N·m/rad | Must beat 0.5 N·m joint friction plus scrub at well under 1° of error |
| steer_ki | **20** N·m/(rad·s) | Removes the last 0.5-1° that friction leaves |
| steer_kd | **2** N·m/(rad/s) | Damping, on measured joint velocity |
| steer_max_effort | 60 N·m | Inside the 150 N·m interface backstop |
| steer_max_effort_rate | 1e5 N·m/s (effectively off) | A slew limit inside a loop is pure phase lag |
| steer_sync_kp | 0 | Measured no benefit (7.3) |
| chassis_yaw_kd | 0 | Measured no benefit once the steering holds (7.3) |
| wheel_kp | **5** N·m/(rad/s) | 0.4 ran 19% fast; ≥10 made the wheels fight each other |
| wheel_ki | **10** N·m/rad | Removes the steady speed error. 2 ran 21% slow up 20° and 1.6× fast down 25° (stress test F4) |
| wheel_hold_max_effort | 25 N·m | Bound on the integral term at a zero target: holds 25° (needs ~11), cannot squeeze the chassis |
| wheel_kd | 0 | Differentiating a noisy velocity only amplifies noise |
| wheel_max_effort | 60 N·m | |
| wheel_max_effort_rate | 1000 N·m/s | Not limiting in practice |
| wheel_sync_kp | 0 | Measured no benefit |
| update_rate | 200 Hz | |

These gains are **specific to Gazebo/DART with this model's inertias and the 0.5 N·m
friction**. They carry over as *ratios and reasons*, not as constants (section 11.3).

### 7.3 Terms that exist but are switched off

- **Gang sync** (`steer_sync_kp`, `wheel_sync_kp`): pulls each joint toward the
  others' *measured* mean, in a sign-normalized frame (`sign_j × value`, see BR in 1.2).
  It was added because four independent wheel loops that disagree behave like a
  mismatched skid-steer and yaw the chassis. With properly tuned loops it measured no
  benefit, and at its old values (5 and 2) it was larger than the tracking gains, so each
  joint mostly chased its neighbours' noise.
- **Chassis yaw damping** (`chassis_yaw_kd`): reads yaw rate from `/odom` and applies the same
  torque to all four knuckles to oppose it. It was added to stop the chassis "spinning out"
  after steering, but that spin was the steering *failing to hold its angle* (gain too low,
  section 10.2), not a separate loop. With the steering holding, +15, 0 and −15 all gave
  < 2° yaw drift. If it is ever re-enabled, its sign must be verified live.

Both remain as parameters, so they can be re-enabled for experiments.

### 7.4 Measured performance (flat ground, shipped config)

| Test | Result |
|---|---|
| Steering at a 30° aim | ~0.2° error while driving, 0.7° yaw drift over 4 s |
| Wheels at 0.5 m/s (3.337 rad/s) | 3.35 rad/s, ripple 0.01 rad/s, clean stop |
| 45° aim, drive 6 s | travel at 44.1°, < 1° yaw |
| 90° aim, −30° reversing, 15 s hold at 30° | all pass the suite (section 12) |
| Chassis vibration | IMU vertical accel std ≈ 0.02-0.09 m/s², same as the ideal modes |

---

## 8. Suspension coupling

`rocker_coupling_node.py`, 500 Hz, active in **every** mode. It enforces the three
constraints of 1.4 as a spring-damper on the constraint residual:

```
for each constraint (a, b, ratio):            # a − ratio·b should equal its rest value
    e    = (q_a − ratio·q_b) − rest
    edot =  v_a − ratio·v_b
    g    = k·e + d·edot
    τ_a −= g
    τ_b += g·ratio                             # virtual-work-consistent split
τ_j = clamp(τ_j, ±max_torque), then slew-limited by max_torque_rate
```

| Parameter | Value | Note |
|---|---|---|
| stiffness k | 8000 N·m/rad | Above ~20000 the loop rings (topic delay) |
| damping d | **50** N·m·s/rad | **was 250; see below** |
| max_torque | 400 N·m | Bigger leaks into crab/spot creep (residual pushes fore-aft) |
| max_torque_rate | 1e6 N·m/s (off) | Limiting it trades jitter for residual, never cleanly |
| velocity_filter_tau | 0 (off) | A filter is extra delay; it measured worse |
| capture_rest | true, after 2.5 s | Zero the constraint against the *settled* pose, not the CAD pose |
| rest_capture_max | 1° | A bigger captured residual means the rover was not level at capture time (uneven spawn), so the CAD rest is held instead (stress test F3) |

**Damping 50 is the most important number in this document.** This loop closes over a
ROS topic round trip (Python, ~480 Hz achieved), so the D term always acts on stale
velocity. Measured in Gazebo:

| damping | Result |
|---|---|
| 800 | shake worse |
| 250 (old) | **self-sustaining 2-3 g pitch shake** (IMU accel std ~27 m/s², pitch rate ~0.85 rad/s) whenever the wheels are not velocity-locked: explicit_pid always; crab, spot, arcs and explicit 45° too |
| 100, 70 | still diverges in explicit/explicit_pid at 45° (explicit_pid spun 100-170°) |
| **50** | calm in every mode (accel std ~0.02) |
| 0 / coupling off | calm, but the suspension is unconstrained |

Why "wheels not locked" matters: an ideal velocity controller on all four wheels pins
the wheelbase, which suppresses the mode where the rockers scissor and front and rear wheels
roll in opposite directions. Effort-driven wheels free that mode, and the delayed
spring-damper drives it.

Cost of d=50: the residual is larger when the wheels are turned across the chassis
(crab/spot ~8-18°, arcs ~8-9°, explicit 45° ~5-6°), against 1-2.5° driving straight.
Crab and spot are accepted as low priority. Section 13.

**Unity note:** if the Unity coupling runs *inside* the physics step (FixedUpdate, no
topic delay), the delay problem largely disappears and higher damping may be stable
there. Do not copy 50 blindly. Re-run the shake test (section 12) and pick the value
that is calm with torque-driven wheels.

---

## 9. Mode switching and controller hand-off

- Modes 1-4 share the ideal controllers (`steer_controller` position,
  `wheel_controller` velocity). Mode 5 uses `steer_pid_controller` and
  `wheel_pid_controller` (effort), which are spawned `--inactive`.
- When the mode crosses the 4/5 boundary in either direction, `rover_kinematics_node`
  makes **one** `switch_controller` call: activate one pair and deactivate the other,
  BEST_EFFORT, `activate_asap`, 2 s timeout. It is fire-and-forget, because blocking inside a
  subscription callback would deadlock the single-threaded node.
- Entering 5: `control_node.py` resets all PID state and starts its subscriptions and
  timer. Leaving 5: it tears them down, and anything it might still publish is dropped
  by the now-inactive effort controllers.

### 9.1 The stale-command bug (fixed)

Symptom: after using mode 5, **every other mode stopped responding** until the sim
restarted. The switch reported success, the ideal controllers were active and
commanding, and the joints sat limp at ~0 effort.

Cause: gz_ros2_control creates a `JointForceCmd` component the first time a joint runs
in effort mode and never removes it. gz-sim's Physics system applies `JointForceCmd`
**instead of** `JointVelocityCmd` whenever both exist ("Only set joint velocity if
joint force is not set"), so the position and velocity commands were silently ignored.

Fix: a patched gz_ros2_control overlay (`src/third_party/gz_ros2_control`, see its
README). Each `write()` removes the command component left by the *other* control mode.

**The general lesson, for Unity and anything else:** when a joint changes from
torque control to target control (or back), explicitly clear the old mode's command
state. Zero the applied torque, re-enable the drive's stiffness/damping, and reset its
target to the current state. Do not assume the engine forgets the old mode.

---

## 10. Tuning history: what failed and why

Read this before tuning anything in Unity. Several fixes that looked right in Gazebo
were treating symptoms.

### 10.1 The jitter was the suspension, not the PID

Explicit_pid was "jittering" heavily. A series of isolation tests, each changing one thing
from the original config, gave this:

| Change | Chassis shake |
|---|---|
| chassis_yaw_kd +15 / 0 / −15 | unchanged (~27 m/s²) |
| sync terms off | unchanged |
| wheel kp 0.4 → 5 → 20, ki off | unchanged |
| steering kp 1 → 60 | unchanged (steering error improved 5° → 1°) |
| **suspension coupling off** | **gone (0.017 m/s²)** |
| **coupling damping 250 → 50** | **gone** |

Lesson: **when a loop jitters, check whether something else is shaking it** before
tuning that loop. Measure chassis IMU vibration in the ideal mode and the PID mode under
the same manoeuvre. If the shake is in both, or survives every PID change, it is not the PID.

### 10.2 The old steering gains could not move the wheel

The original gains (kp 1, kd 3, ki 0) were chosen while the suspension was shaking. That
shake dithered the joints past their static friction, so steering appeared to work.
With the shake removed, a 30° aim commanded 0.51 N·m, below 0.5 N·m friction plus scrub,
and **the wheels never moved** (29.5° error, rover drove straight).

Rule of thumb from this: `kp × (acceptable error) ≫ joint friction + scrub torque`.
With friction 0.5 N·m and a 1° (0.017 rad) target, kp must be well above 30. The sweep:

| steer kp/ki/kd | error at 30° | yaw drift while driving |
|---|---|---|
| 1/0/3 | 29.5° (stuck) | — |
| 10/0/1 | ~4° | −13.5° |
| 30/0/2 | ~1.5° | −2.3° |
| 60/0/2 | ~0.7° | −1.2° |
| **60/20/2** | **~0.2°** | **−0.7°** |

The "chassis spins after steering" problem that the yaw-damping and sync terms were added
for came from the same cause: a steering joint that cannot hold its angle lets the
chassis drag it around.

### 10.3 The wheel loop: too soft, then too stiff

| wheel kp/ki | speed vs 3.34 rad/s target | notes |
|---|---|---|
| 0.4/0.4 | 3.97 (+19%), ripple 0.33 | rings for seconds after stop |
| **5/2** | **3.35**, ripple 0.01 | clean stop |
| 10/5, 20/10 | accurate | wheels fight each other through the ground (±14 N·m), yaw drift doubles |

Four independent stiff velocity loops on a rigid chassis over-constrain it: any tiny
geometric mismatch becomes an internal force fight. Moderate gains are better than
stiff ones here.

### 10.4 Integrator wind-up at standstill

With a zero speed target, the four wheel integrators each wound up against small
suspension creep. Front and rear ended up pushing ±17 N·m against each other, with
nothing commanded. Fix: clear the wheel integrator whenever its target is exactly zero,
so a stopped wheel holds on P alone.

**Superseded 2026-10-14.** P alone cannot hold a hill: parked on 20° the rover rolled
down at ~0.1 m/s (stress test F4). The integrator now keeps running at a zero target,
which acts as a brake, with its term clamped to `wheel_hold_max_effort` (25 N·m). That
holds 25° to within 1 cm. A 60 s flat hold shows 0.4–0.6 N·m per wheel, so the squeeze
did not come back. That squeeze was measured while the suspension was still shaking
(§10.1).

### 10.5 Things tried that did not help

- An output slew limit on steering torque (40 N·m/s). It adds phase lag inside the loop.
- A velocity low-pass on the coupling's D term. It is extra delay on a loop whose problem is delay.
- Raising damping to stop the shake. The opposite is true on a delayed loop.

---

## 11. Porting to Unity

The Unity simulation already has a `RoverDriveController` (the Gazebo kinematics node
was written as its counterpart). The goal is the same *behaviour*. Map the pieces as
follows. The Unity code was not reviewed while writing this, so treat class and API
names as guidance.

### 11.1 What must match exactly (engine-independent)

- Conventions: frames, signs, FL/FR/BL/BR order, `steer_sign`/`drive_sign`, BR's mirrored
  drive (section 1).
- Teleop semantics per mode, including explicit's `angular.z` = steering rate (section 3).
- The kinematics pipeline: timeout, ramps, velocity field, **fold → rate-limit →
  project** order, curvature cap, explicit aim integration and entry sweep (sections 4-6).
- The PID law structure: derivative on measurement for steering, conditional-integration
  anti-windup, bounded wheel integrator hold at zero target, reset on mode entry (section 7.1).
- Hand-off rule: one control law per joint, and clear the old mode's commands (section 9).
- The coupling constraints and the residual law (section 8).
- The acceptance tests (section 12).

### 11.2 Ideal modes (1-4) in Unity

With `ArticulationBody` joints:
- Steering (position): drive `target` = `steer_sign·angle` in degrees, with high
  stiffness and damping, generous `forceLimit`. This is the "ideal" position servo.
- Wheels (velocity): drive `targetVelocity` = `drive_sign·speed/r` in deg/s,
  stiffness 0, high damping (a velocity drive).
- Run the kinematics in `FixedUpdate` at a fixed 100 Hz equivalent (or at the physics rate
  with dt-correct ramps).

### 11.3 Explicit_pid (mode 5) in Unity

- Put the steering and drive joints into **torque mode**: drive stiffness = 0 and damping = 0
  (or `driveType = Force`), and apply the PID output as joint torque every `FixedUpdate`
  (e.g. `ArticulationBody.jointForce` / `AddTorque` about the joint axis).
- Set **joint friction** on the steering joints to the real or URDF value (0.5 N·m), or
  the steering is unrealistically easy and gains tuned in Unity will fail on the rover.
- Run the PID in `FixedUpdate`. This removes the topic delay Gazebo has, which helps.
- **Re-tune the gains in Unity** with the same method: start from the Gazebo values,
  then check the steering step error, the wheel speed error and ripple, and the absence of
  chassis vibration, against the section 12 thresholds. Keep the ratios and reasons:
  steering P strong enough to beat friction, small steering I, modest steering D,
  moderate wheel PI, no wheel D, sync and yaw terms off.
- Implement the bounded zero-target integrator hold and the reset-on-entry.

### 11.4 Suspension in Unity

Unity articulations cannot do closed loops either. Options, in order of preference:
1. If the Unity model already enforces the coupling some other way (e.g. a joint-space
   constraint or a scripted mimic), confirm it satisfies the three constraint equations
   of 1.4 and skip this section.
2. Otherwise apply the same spring-damper law as section 8 to the four rocker joints in
   `FixedUpdate`. Tune k/d in Unity. Because there is no delay, the stable damping may be much
   higher than 50. Choose it with the shake test, using **torque-driven wheels** (mode 5), since
   that is the case that exposed the instability here.
3. Capture the rest residual after settling (~2.5 s), not from the CAD pose.

### 11.5 Mode switching in Unity

On 4 → 5: zero all PID state, switch the steering and drive joints to torque mode, and start
the PID. On 5 → anything: stop the PID, **zero the applied torques**, restore the drive
stiffness/damping, and set each drive target to the joint's current position/velocity so
it does not snap. Then let the kinematics take over. Section 9.1 is what goes wrong if
any of this is skipped.

### 11.6 Porting checklist

- [ ] Each steering joint, commanded +10° kinematic, points its wheel **left**
- [ ] Each wheel, commanded +1 m/s, rolls the rover **forward** (watch BR)
- [ ] Ackermann left arc yaws CCW; front and rear counter-steer
- [ ] Crab from rest: no yaw kick at start (the rate limit is working)
- [ ] Reversing straight: no 180° wheel flip (the fold is working)
- [ ] Explicit: aim 45°, drive, and the body travels at ~45° with ~0 yaw
- [ ] Explicit_pid: a 30° aim is reached within ~1° (if stuck, steering kp is below friction)
- [ ] Explicit_pid standstill: wheel torques ≈ 0 with zero target on flat, and it holds on a 20° slope
- [ ] Explicit aim at its 90° clamp, then ackermann forward: the rover goes forward, not sideways
- [ ] NaN/inf in cmd_vel: ignored, the rover stops via the timeout
- [ ] 5 → 1 → 5 → 4: every mode still tracks after each switch
- [ ] No chassis shake in any mode (IMU test, section 12)

---

## 12. Acceptance tests

Gazebo: `ros2 run rover_gazebo run_stability_suite.sh` (flat, ledge and bars worlds,
headless), checks by `rover_gazebo/stability_test.py`. **Run it from
`source install/local_setup.bash`** so the patched gz_ros2_control is used.

Each scenario: select the mode, sweep the aim (explicit modes), hold and/or drive, then judge
the steady window (the first 2 s of driving skipped):

| Check | Pass criterion (flat) |
|---|---|
| shake | IMU vertical accel std < 1.0 m/s², roll/pitch-rate RMS < 0.05 rad/s |
| steer tracking | per-joint 95th percentile \|target − position\| < 3° |
| wheel tracking | \|mean error\| < 10% of \|target\| + 0.1 rad/s, ripple std < 0.3 rad/s |
| motion: straight/crab | speed within 15% of command, heading error < 6°, yaw change < 3° |
| motion: arc/spot | yaw rate within 20% of command, and spot translation < 0.15 m |
| motion: explicit | travel direction within 6° of aim, yaw change < 3°, speed within 15% |
| motion: hold | moved < 0.03 m, yaw < 1° |
| suspension | mean constraint residual < 3° (crab/spot: < 20°, accepted limitation) |

Obstacle worlds use relaxed limits (shake 6 m/s² / 0.6 rad/s, steer 6°, residual 6°,
yaw 10°) and check forward progress ≥ 60% of commanded and tilt < 35°.

Scenarios: settle; ackermann forward, reverse, arc left/right; crab right and diagonal;
spot cw/ccw; explicit 0° and 45°; explicit_pid 0°, 45°, 90°, −30° reversing, 15 s hold
at 30°; and **after leaving explicit_pid**: explicit 20°, explicit_pid 0°, ackermann
forward, spot cw. The hand-off scenarios are what catch section 9.1.

**The Unity side should implement the same scenarios and thresholds.** The calm-rover
baselines (accel std ~0.02 m/s², pitch rate ~0.0002 rad/s on flat ground) are physics-
engine-specific. Measure Unity's own calm baseline in the ideal mode at standstill and keep
the thresholds well above it, but far below "shaking" (tens of m/s²).

Beyond these acceptance tests, `src/rover_gazebo/doc/STRESS_TESTING.md` covers slopes,
side-slopes, rocks, washboard, low friction, rough terrain and command abuse. Its
findings F1-F8 are open, and F1 (stuck sideways after an explicit aim at the 90 deg clamp),
F2 (NaN/inf cmd_vel) and F4 (explicit_pid on hills) apply directly to a Unity port.

### 12.1 Latest results (2026-10-12, patched overlay, flat world)

All explicit_pid scenarios pass, all four after-switch scenarios pass, and ackermann
forward/reverse/arc-left, crab, spot and explicit 0° pass. Remaining failures:
ackermann arc-right (accel std 2.8 m/s², residual ~9°), explicit 45° (residual ~6°).
Ledge world passes both ways. Bars world: see section 13.

---

## 13. Known limitations and open items

- **Turned-wheel suspension residual.** With wheels turned across the chassis, the
  coupling's residual has nothing to roll against: crab/spot ~8-18°, arcs ~8-9°, explicit
  45° ~5-6°. This is a consequence of a delay-limited, damping-limited coupling. A
  coupling running inside the physics step (C++ controller, or FixedUpdate in Unity)
  is the real fix.
- **Bars world:** ackermann stalls at the 0.12 m bar (1.2 m progress) and the shake
  returns (~16 m/s²) while jammed. Explicit_pid gets back over it but at 6.6 m/s²
  vibration. Not investigated further.
- **Gains are Gazebo-specific.** No values were validated against hardware; steering
  friction (0.5 N·m) and inertias come from CAD/URDF, not measurement.
- **Python loops at high rate.** The kinematics (100 Hz), PID (200 Hz, achieved period p99 ≈ 9-11 ms)
  and coupling (500 Hz) loops run in Python over topics. The PID assumes a fixed dt. Moving
  the PID and coupling into C++ ros2_control controllers would remove the delay that
  limits the coupling damping.
- **gz_ros2_control overlay** must be kept until upstream removes stale command
  components on a mode switch (not fixed as of `rolling`, 2026-10-12).

---

## 14. File index

| File | Role |
|---|---|
| `src/rover_gazebo/config/rover_kinematics.yaml` | Geometry, joint names, signs, coupling ratios (generated from CAD) |
| `src/rover_gazebo/config/rover_control.yaml` | All runtime parameters: kinematics, teleop, explicit_pid gains, coupling |
| `src/rover_gazebo/config/controllers.yaml` | ros2_control controllers and joint order |
| `src/rover_gazebo/urdf/rover_ros2_control.xacro` | Command/state interfaces, effort backstops |
| `src/rover_gazebo/urdf/rover_body.xacro` | Links, joints, limits, friction/damping |
| `src/rover_gazebo/rover_gazebo/rover_kinematics_node.py` | Sections 4-6, 9 |
| `src/rover_gazebo/rover_gazebo/control_node.py` | Section 7 |
| `src/rover_gazebo/rover_gazebo/rocker_coupling_node.py` | Section 8 |
| `src/rover_gazebo/rover_gazebo/teleop_common.py`, `teleop_rover*.py` | Section 3 |
| `src/rover_gazebo/rover_gazebo/stability_test.py`, `tools/run_stability_suite.sh` | Section 12 |
| `src/rover_gazebo/doc/VERIFICATION.md` | Measurement record |
| `src/rover_gazebo/doc/updates.md` | Change log |
| `src/third_party/gz_ros2_control/`, `src/third_party/README.md` | Section 9.1 fix |
