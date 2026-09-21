# What was measured, and what was not

## Why there is only one coupling path

Gazebo has no solver-level way to enforce the suspension's closed-loop constraints on the
physics engine this package targets. Fortress's SDF version has no `<mimic>` element at
all. Harmonic's does, and it was confirmed, with an isolated two-joint test model, to
work correctly under gz-physics's Bullet-Featherstone engine (`follower = multiplier *
leader + offset`, matched to six decimal places) but to be explicitly rejected by DART,
the engine this package actually uses, which logs "the chosen physics engine does not
support mimic constraints" and leaves the follower joint unmoved. Both engines are
already part of the Gazebo install; nothing needs to be added to try this. It has not
been tried on the real rover: this version of Bullet-Featherstone refuses to attach a
joint straight to `world` unless it is fixed, and it is unconfirmed whether it accepts a
free-floating multi-branch chassis like this one's, or whether `gz_ros2_control`'s
hardware interface and the wheel contact tuning would carry over unchanged. So the
constraints are held entirely by `rocker_coupling_node`, applying them as joint torques,
and everything below measures that path.

Everything below the next section was run on **Gazebo Fortress**, ROS 2 Humble, 1 ms
physics step, DART. Each row is a separate simulator launched from scratch, because runs
left to share a simulator contaminate each other. Distances are in the rover's own start
frame: forward, then sideways.

Fortress is not available where this package now also runs: **Gazebo Harmonic**
(gz-sim 8.11, gz-physics 7.6, still DART) is, and the two do not agree. A dedicated
section below, "A jitter that Fortress never showed", covers what changed and why.
Where a manoeuvre was re-measured on Harmonic, its number sits alongside the Fortress
one; where it was not, the Fortress-only number stands and is marked as such rather than
assumed to carry over.

## Driving

| Manoeuvre | Fortress | Harmonic |
|---|---|---|
| Standing still, 3 s | moved 0.0 mm, tilted 0.19 deg | not re-measured |
| Forward, 5 s at 1 m/s | 4.731 m, 0.1 mm lateral, 0.00 deg yaw | 4.512 m, 33 mm lateral, 0.52 deg yaw, peak speed 1.83 m/s against a 1.0 m/s command |
| Arc left, 5 s | 256.1 deg of turn | not re-measured |
| Arc right, 5 s | 257.2 deg of turn | not re-measured |
| Reverse with yaw commanded, 5 s | 275.4 deg | not re-measured |
| Crab right, 5 s | 4.06 m sideways | 4.60 m distance, 36.45 deg yaw (versus 31.2 deg on Fortress), peak speed 2.66 m/s |
| Spot turn, 5 s | 269.6 deg spun, body moved 117 mm | not re-measured |

The Harmonic forward run is the one to look at first: nothing here asks the wheels to do
anything but point straight and roll, yet the peak speed comes in at nearly twice the
commanded value and the heading picks up half a degree of drift, where Fortress showed
none. Harmonic's crab number is in the same direction as Fortress's but larger across
every column. Neither of these is a driving-logic problem; see the jitter section below
for what it actually is.

Straight running on Fortress is as good as it gets: over 4.7 m the rover drifted a tenth
of a millimetre sideways and its heading did not change in the second decimal place.
Harmonic does not repeat this; see below. The two arcs mirror each other to within 1.1
degrees.

Reversing deserves a note. Commanding a yaw rate while reversing yaws the rover in the
commanded direction, not the opposite one. That is ordinary `Twist` semantics and what a
nav stack expects. It is not car behaviour, where holding the wheel and reversing swings
you the other way.

The steering itself was checked directly against the maths rather than inferred from
where the rover ended up:

| Mode | Commanded, deg | Measured, deg |
|---|---|---|
| Crab right | 90.0, 90.0, 90.0, 90.0 | 90.0, 90.0, 90.0, 90.0 |
| Spot, 45 deg/s | 59.7, -59.7, -59.7, 59.6 | 59.7, -59.7, -59.7, 59.6 |

Those are the exact angles the geometry calls for, and the joints hold them under load.

## A wheel-angle snap, unrelated to the coupling jitter

Reported directly: crab mode's wheel turning felt very quick, and visibly changed the
chassis's orientation while it happened. This turned out to be a real bug, separate from
the coupling loop above, and specific to how a *changing* command was solved rather than
to anything in the physics.

Body speed (`vx`, `vy`) is ramped smoothly toward a new command, but before this fix the
wheel angle solved from it, `atan2(vy, vx)`, was not. Ramping `vx` and `vy`
independently means their ratio, and so the angle, can swing through 90 degrees in a
single tick while both are still near zero: pressing a pure sideways crab command from a
stop asked for a 90 degree wheel snap on the very first control-loop tick, long before
the body had picked up any real speed. All four wheels solve the identical angle in crab
mode, but nothing guarantees they arrive at a snapped target in perfect lockstep, and
that brief mismatch is what showed up as a real yaw kick on the chassis.

The fix rate-limits the solved angle itself (`max_wheel_turn_rate`, 2 rad/s by default)
and drives each wheel at however much of the desired ground velocity actually lies along
wherever it is presently pointed, not the full magnitude, so a wheel still mid-turn eases
off rather than scrubbing sideways at full speed. This also smooths ackermann and spot,
which solve wheel angles the same way and share the same code path, though crab's
sideways-from-a-stop case was the one severe enough to be reported.

Measured directly in Gazebo, commanding a full crab reversal (`linear.y` from +1 to -1 at
speed 1.0 m/s):

| | Peak `angular.z` during the reversal |
|---|---|
| Before this fix | 1.632 rad/s |
| After this fix | 0.384 rad/s |

A 76% cut in the peak yaw rate during the exact manoeuvre that was reported, from an
unrelated cause to the jitter section below: this is a solved-angle discontinuity, not
the suspension coupling. Both can be present at once, and this fix does not touch or
depend on the coupling loop.

## A jitter that Fortress never showed

The Harmonic forward number above is not a fluke. Recording `/odom` continuously through
a plain 1.0 m/s forward drive on Harmonic, the instantaneous speed does not sit near
1.0 m/s: it oscillates between roughly 0.56 and 1.79 m/s, dozens of times a second, for
the whole drive, though the mean over the run comes out correct. Turning the wheels to
90 degrees, as crab mode does, makes it markedly worse rather than proportionally worse:
yaw-rate noise about twelve times larger, peak speed up to 2.37 m/s. This is a real,
physical effect confirmed from the raw time series, not a summary-statistic artefact:
individual samples show an unmistakable step-and-decay shape, not scattered noise.

Two things were tried, live, to find out what drives it, both against the same plain
forward drive so they are comparable to the baseline above and to each other. Neither
changes anything about the model as shipped; both were reverted after measuring.

| Condition | Speed, std dev | Speed range | Yaw-rate std dev | Peak \|yaw rate\| |
|---|---|---|---|---|
| As shipped (coupling on, default contact) | 0.069 m/s | 0.56 - 1.79 m/s | 0.0030 rad/s | 0.050 rad/s |
| `rocker_coupling`'s stiffness and damping forced to zero | 0.027 m/s | 0.76 - 0.97 m/s | 0.0000 rad/s | 0.000 rad/s |
| Wheel contact softened (`kp` 100000 to 20000, `kd` 10000 to 2000, `maxVel` 0.1 to 1.0), coupling left on | 0.157 m/s | 0.33 - 1.91 m/s | 0.0002 rad/s | 0.003 rad/s |

Zeroing the coupling essentially removes the jitter in both speed and yaw at once.
Softening the wheel contact instead does not: yaw-rate noise drops about as much as
zeroing the coupling does, but speed variance more than doubles and the range widens in
both directions. So the two knobs are not independent, and detuning wheel contact alone
trades one symptom for a different one rather than fixing anything: the coupling loop is
the thing driving the disturbance, and contact stiffness only decides which axis it comes
out on. This does not mean the coupling should simply be switched off; that is exactly
the constraint the suspension depends on to avoid folding under its own weight, and
switching it off was only ever a diagnostic, not a proposal.

This is consistent with, and probably a sharper version of, something already visible in
the Fortress numbers further down this page: the coupling-on/coupling-off comparison for
crab mode already showed coupling-on producing more forward creep despite holding the
suspension together better. What is new on Harmonic is that the same mechanism now
produces continuous high-frequency oscillation rather than a slower, steadier creep,
plausibly because Harmonic's version of DART interacts differently with a stiff 500 Hz
correction loop closed over ROS topics than Fortress's did.

## Trying to tune the coupling loop itself out of ringing

`rocker_coupling_node` gained two more parameters after the above, both aimed at the
loop itself rather than at wheel contact: `max_torque_rate`, a ceiling in N m per second
on how fast the published torque may change regardless of its size, and
`velocity_filter_tau`, a one-pole low-pass on the velocity feeding the D term. Both
default to off (an effectively unlimited rate, and a zero time constant), so shipped
behaviour does not change unless they are tuned. Full mechanism and code in
`rover_gazebo/rocker_coupling_node.py`.

One thing has to be said before the numbers: **run-to-run variance here is large enough
to matter.** Two back-to-back measurements of the identical as-shipped configuration, in
the same launch, gave speed std dev of 0.185 and 0.213 m/s -- a wider spread than some of
the differences being compared below. A separate earlier launch gave 0.069 m/s for the
same nominal configuration. So a single run at each setting, which is what the table
below is, shows real effects at the extremes but cannot be trusted to order two nearby
settings correctly. Treat this as a first pass, not a precisely mapped curve.

| Configuration | Speed std dev | Settled residual (left / right / diff), deg |
|---|---|---|
| As shipped (rate unlimited, no filter) | 0.185 - 0.213 m/s | 3.3 / 3.3 / 0.9 |
| `max_torque_rate` = 50,000 N m/s | 0.034 m/s | 6.8 / 6.8 / 10.2 |
| `max_torque_rate` = 150,000 N m/s | 0.280 m/s | 4.7 / 4.7 / 6.8 |
| `max_torque_rate` = 2,000 N m/s | ~0 (jitter gone) | 28 - 37, and did not recover |
| `velocity_filter_tau` = 0.02 s, rate unlimited | 0.240 m/s | 5.4 / 5.4 / 8.0 |

For reference, the residual this package already considered normal, from the Fortress
numbers further down this page, is one to two and a half degrees.

Three things came out of this that are worth trusting despite the noise:

**The 2,000 N m/s setting is a genuine hazard, not just a bad trade.** Tightened live on
an already-settled rover, it let the suspension sag into a visibly different, wrong
equilibrium (residual in the high thirties, the chassis sitting noticeably lower) that
did **not** recover when the limit was removed again. Whatever briefly held it there
kept holding it there. A limit this tight should not ship as a default.

**The velocity filter made both numbers worse, not better, at the one value tried.** This
matches the concern raised before building it: the loop's problem is that it is already
reacting to slightly stale information, and a filter is additional staleness. It is not
recommended, at least not at 0.02 s, and nothing here found a value where it helped.

**A torque-rate limit can trade jitter for residual, but not for free, and not cleanly.**
50,000 N m/s cut speed jitter by roughly 80% at a real cost: settled residual on the
differential constraint rose from under a degree to about ten. 150,000 N m/s held
residual closer to the original but showed worse jitter than the unlimited baseline in
this one run, which given the noise caveat above may just be an unlucky draw rather than
a real reversal. Nothing tried here reached both a materially quieter ride and a residual
still near one to two degrees at once.

No change was kept from any of this: `rover_control.yaml` ships with both new parameters
at their off defaults, exactly as before this investigation. Whether to accept a looser
residual for a quieter ride, and by how much, was left as an open decision rather than
picked here.

## Explicit mode, which has not been measured

Explicit mode was added after everything above was run. **Nothing in this section is
filled in from a run**, and the table below is deliberately left empty rather than
populated with numbers nobody took. The harness exists; running it is what is missing.

The test aims the wheels first with the drive at zero, waits for `/rover/steer_aim` to
confirm the angle the node actually reached, and only then takes the start pose, so
whatever the steering scrubs on its way round is not charged to the run:

```bash
ros2 run rover_gazebo measure_rover.py --test explicit --aim 45 --speed 0.5 --duration 5
```

Two numbers decide whether the mode works. `track_angle_deg` is the direction the body
actually travelled, in its own start frame, and it should match `aim_reached_deg`.
`yaw_deg` should be zero, because four parallel wheels have nothing to yaw the rover
with.

| Aim, deg | Track angle, deg | Track error, deg | Yaw drift, deg | Distance, m |
|---|---|---|---|---|
| 0 | | | | |
| 45 | | | | |
| 90 | | | | |

Expect the error to grow with the aim rather than stay flat, and say so in the table
when it does. The constraint residual measured below leaks out as a fore-aft shove, and
a wheel pointed sideways has nothing to resist it with. That is the same mechanism that
makes crab drift forward further down this page, so an aim of 90 degrees should look
much like the crab row, and an aim of 0 should be indistinguishable from driving
straight.

This has nothing to do with the steering joint's own travel. Ninety degrees commanded sits comfortably inside its 100 degree hard stop, with no saturation and no fight against a limit going on. The badness tracks how far the wheel's own heading has turned away from the chassis's fore-aft axis, not how close the command sits to anything mechanical: a wheel aligned with the chassis absorbs the suspension's fore-aft residual by simply rolling with it, and a wheel turned across the chassis cannot, so the residual pushes the chassis instead.

## The suspension, with the coupling and without

The "without" rows have the coupling torque set to zero at runtime, which is what the
rover would do if the loops were simply left open, as the export leaves them.

Driving straight on flat ground:

| | Chassis pitch | Rocker angles, deg | Worst constraint residual |
|---|---|---|---|
| Coupling on | 1.93 deg | -5.1, +0.5, +0.5, -5.2 | 2.5 deg |
| Coupling off | 15.04 deg | -25.8, +23.6, +23.6, -25.8 | 47.3 deg |

With the loops open the suspension folds up under the rover's own weight and reaches its
travel stops, and the chassis pitches fifteen degrees nose down just driving along.

Straddling a 10 cm ledge under the left wheels only, four metres in:

| | Chassis roll | Chassis pitch | Rocker angles, deg |
|---|---|---|---|
| Coupling on | 13.13 deg | 1.49 deg | -4.4, +0.2, +0.5, -4.2 |
| Coupling off | 13.85 deg | 14.70 deg | -25.8, +24.0, +23.6, -26.0 |

The roll is the same either way, and that is correct rather than a failure. A 10 cm step
across a 0.436 m track is 12.9 degrees of ground angle, and this suspension is compliant
in warp, not in roll, so the chassis lies down on the terrain and both numbers land on
13. What the coupling buys is everything else: the chassis stays level fore and aft and
the rockers stay near neutral instead of sitting on their stops with nothing left to
give.

Over the two-bar course, 8 cm under the right wheels and 12 cm under the left, which is
the diagonal disturbance the mechanism actually exists for:

| | Peak chassis tilt | Peak roll | Worst residual |
|---|---|---|---|
| Coupling on | 10.64 deg | 8.44 deg | 1.5 deg |
| Coupling off | 22.39 deg | 13.24 deg | 54.1 deg |

Halving the tilt over one-sided obstacles is the whole point of the differential, and it
is the same result the Unity build gave, which peaked at 11.8 degrees over the same
course.

## Where the torque coupling falls down

It closes a stiff constraint loop over ROS topics at 500 Hz instead of inside the
physics step. Two consequences, both measured.

**It cannot hold the constraint exactly.** A residual of one to two and a half degrees
survives while driving. Raising the stiffness does not fix it and past about 20000 N m
per radian the loop delay makes it ring.

**That residual leaks into crab and spot.** Whatever the coupling cannot correct it
keeps pushing against, and the push has a fore-and-aft component. With all four wheels
pointed sideways there is nothing to resist it, so the rover creeps and yaws:

| Crab right, 5 s | Sideways | Forward creep | Yaw drift |
|---|---|---|---|
| Coupling on | 4.06 m | 2.51 m | 31.2 deg |
| Coupling off | 4.77 m | 0.44 m | 4.2 deg |

A separate four-second run with the coupling torque zeroed gave 3.341 m sideways,
0.017 m forward and 0.4 degrees of yaw, which is what the kinematics alone deliver.
Capping the coupling torque helps: dropping the ceiling from 2000 to 300 N m cut the
creep by four fifths, and the shipped default of 400 N m is that compromise.

## Reproducing this

The harness is `rover_gazebo/measure_rover.py`. Point it at a running simulation, name a
manoeuvre, and it prints a line of JSON:

```bash
ros2 launch rover_gazebo rover_sim.launch.py gui:=false &
ros2 run rover_gazebo measure_rover.py --test forward --duration 5 --settle 6
```

`--coupling-stiffness 0 --coupling-damping 0` turns the coupling off for the
before-and-after rows. Start a fresh simulator for every measurement; reusing one
carries suspension state and rover position from the previous run into the next, which
was enough to make an early round of these numbers meaningless.
