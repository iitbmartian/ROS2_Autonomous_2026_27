# What was measured, and what was not

## Why there is only one coupling path

Gazebo has no solver-level way to enforce the suspension's closed-loop constraints.
Fortress's SDF version has no `<mimic>` element at all, and even where a `<mimic>`
element exists, whether it survives the URDF-to-SDF conversion and reaches the physics
engine is version-dependent and was found not to hold on the Gazebo this package
targets. So the constraints are held entirely by `rocker_coupling_node`, applying them
as joint torques, and everything below measures that path.

Everything was run on **Gazebo Fortress**, ROS 2 Humble, 1 ms physics step, DART. Each
row is a separate simulator launched from scratch, because runs left to share a
simulator contaminate each other. Distances are in the rover's own start frame:
forward, then sideways.

## Driving

| Manoeuvre | Result |
|---|---|
| Standing still, 3 s | moved 0.0 mm, tilted 0.19 deg |
| Forward, 5 s at 1 m/s | 4.731 m, 0.1 mm of lateral drift, 0.00 deg of yaw drift |
| Arc left, 5 s | 256.1 deg of turn |
| Arc right, 5 s | 257.2 deg of turn |
| Reverse with yaw commanded, 5 s | 275.4 deg |
| Crab right, 5 s | 4.06 m sideways |
| Spot turn, 5 s | 269.6 deg spun, body moved 117 mm |

Straight running is as good as it gets: over 4.7 m the rover drifted a tenth of a
millimetre sideways and its heading did not change in the second decimal place. The two
arcs mirror each other to within 1.1 degrees.

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
