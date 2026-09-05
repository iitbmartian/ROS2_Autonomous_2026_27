# What was wrong, and what was done about it

## The mechanism the CAD describes

Four rocker arms pivot on the chassis, one per wheel, each carrying a steered knuckle
and a driven wheel. On each side a connector bar joins the front rocker to the rear
one. A differential bar pivots on the chassis about the fore-aft axis and reaches each
rear rocker through a 150 mm pushrod.

That is four loop closures. Count the degrees of freedom: four rocker angles, minus two
connector constraints, minus one from the differential, leaves **one**.

The single remaining motion is worth stating plainly, because it decides what the
suspension can and cannot do. Each connector bar joins two cranks that both point
upward from their pivots, which makes the two rockers on that side a see-saw: push the
front wheel down and the rear wheel rises. That is what turns each side into a single
rocker beam. The differential then ties the two sides together so one side's rocker
turns as the other's turns back.

Put those together and the one free motion is diagonal. The front-left and rear-right
wheels drop together while the front-right and rear-left rise. Driving the free joint to
ten degrees in the forward kinematics moves them by -35, -21, +43 and +24 millimetres
respectively, which is that pattern.

So this is a warp-compliant suspension. Over diagonal unevenness, the kind that would
otherwise leave one wheel in the air with the rover rocking on the other three, it keeps
all four loaded and passes only part of the disturbance to the chassis. It does not
flatten a sustained step under one whole side; there the chassis rolls with the ground,
which is what a rocker differential does and what the measurements show.

## Why the export cannot say that

A URDF is a tree. It has no way to write a loop, so the SolidWorks exporter simply
stopped at the point where each loop would close. The connector bars hang off the front
rockers attached at one end only. The differential bar and its two pushrods hang off the
chassis attached to nothing. Load them and the suspension has four independent rockers
instead of one coupled one, and the rover sags onto its chassis.

This is the same wall the Unity build hit. There, the loops were closed by hand with
proxy bodies, and the differential ones tore themselves apart. The fix in Unity was to
delete the differential linkage and replace it with a coupling computed in joint space.

## What is done here instead

The constraints are recovered as arithmetic and written into the description.

A four-bar linkage is not linear, but this one is very nearly so over the travel that
matters. Solving it exactly and fitting a straight line over plus or minus twenty
degrees of front-rocker travel leaves under four tenths of a degree of error, which is
under three millimetres of wheel height. So each loop becomes one multiplier:

| Constraint | Relation | Worst error over the fit range |
|---|---|---|
| Left connector four-bar | `BLS = -0.5772 x FLS` | 0.24 deg, 1.0 mm of wheel travel |
| Right connector four-bar | `BRS = -0.5727 x FRS` | 0.36 deg, 1.4 mm of wheel travel |
| Differential | `BLS + BRS = 0` | exact |

The differential relation is worth a second look, because it is exact rather than
fitted. Both pushrods are vertical at rest and mirror images of each other. The left pin
on the differential bar rises at `+R x phidot` while the right falls at `-R x phidot`,
and each rocker's socket rises at `-r x thetadot`. Add the two rows and the bar's own
rate cancels, leaving the sum of the two rear rocker angles constant, whatever the arm
lengths are. The derivation tool checks the geometry closes by computing both pushrod
lengths from the joint origins: both come out at 0.150000 m against a modelled 0.15 m.

Taking the front-left rocker as the one free coordinate and chaining through, every
other joint in the suspension is a fixed multiple of it:

```
BLS = -0.5772 x FLS      the left connector
BRS = +0.5772 x FLS      the differential, applied to the above
FRS = -1.0077 x FLS      the right connector, worked backwards
```

Two things say this analysis is right rather than merely self-consistent. The
differential reduces to the same rule the Unity coupler enforced, arrived at by a
completely different route. And it predicts that the rear-right wheel's drive axis is
inverted relative to the other three, which is what the Unity controller printed on
startup after calibrating itself from the live transforms.

## Two more defects the export carries

**The model is Y-up.** SolidWorks exported with Y as the vertical axis, X pointing
backwards and Z to the left. Spawn that into Gazebo, whose gravity is along world -Z,
and the rover lies on its side. A ROS-convention `base_link` now sits above the chassis
with a fixed joint carrying `rpy="1.5708 0 3.1416"`, and a `base_footprint` above that
at ground level. Both joints are fixed, so sdformat folds all three into one body and
the simulation pays nothing for them.

**Every joint declares `effort="0"`.** In Gazebo that is a zero force limit, so nothing
could be driven at all. This is the same class of defect as the Unity importer's
Controller component zeroing every drive's force limit at startup: a value that means
"unset" in one tool and "locked solid" in another.

**Links and joints share names.** The exporter names a joint after its own child link.
URDF tolerates that; SDF does not, because it keeps links, joints and frames in one
namespace, and Gazebo refuses to load the model. Every joint therefore carries a
`_joint` suffix. Link and frame names are untouched, so they still match the CAD.

## How the constraints are enforced

Gazebo has no solver-level way to enforce a joint constraint like this: sdformat's
`<mimic>` element either does not exist or is not carried through to the physics engine,
depending on the version. That was tried and confirmed unworkable, so this package
enforces the coupling itself instead.

The four rockers get effort interfaces, and `rocker_coupling_node` applies each
constraint as a stiff spring and damper, pushing equally and oppositely on the two
joints it ties together:

```
e   = theta_a - ratio x theta_b
g   = K e + D edot
tau_a -= g
tau_b += g x ratio
```

Pushing on both is the whole point. It is what makes load on a rear wheel show up at
the front rocker, the way a steel bar would. Holding only the followers would leave the
front rockers carrying nothing.

This closes a stiff loop over ROS topics at 500 Hz rather than inside the physics step,
so the constraint is held to a degree or two rather than exactly, and the gains cannot
be raised much before the loop delay makes it ring. `doc/VERIFICATION.md` has the
measured residuals.

## What became decoration

Once the loops are arithmetic, the connector bars, the differential bar, the two
pushrods and the four ball ends carry no load. They are kept, because a rover with a
visible differential is easier to reason about than one without, but they are all fixed
joints now, since nothing drives them once the loop they closed is replaced by the
torque coupling on the rockers themselves:

- none of them has collision geometry, since they overlap their neighbours by design
- the four 8 mm ball ends and the two pushrods became fixed joints, so sdformat folds
  them away, taking with them six bodies whose 0.000268 kg mass and 1.7e-9 inertia would
  otherwise sit in the solver next to a 63.9 kg chassis
- the connector bars and the differential bar are fixed too, so they no longer move on
  screen with the free rocker; the linkage's motion has to be read off `/joint_states`
  instead

## Collision geometry

The export used the full mesh for collision on every link. That is replaced by
primitives that match the meshes to a tenth of a millimetre:

- wheels become cylinders, radius 0.14985 and length 0.140, read off the STL bounding
  box. The wheel mesh is a bare cylinder with no tread, so nothing is lost
- the chassis becomes a 0.4 m box, which is exactly what its mesh is
- rockers, knuckles and the whole decorative chain have no collision at all

`mesh_collision:=true` puts the meshes back on the chassis, rockers and knuckles.

## Numbers, and where they come from

Nothing above is hand-measured. `tools/derive_ratios.py` reads the original export,
solves the four-bars, derives the differential law, works out the wheel stations and
every joint's sense, and writes `config/rover_kinematics.yaml`. Both the model generator
and the two runtime nodes read that file. Re-run it after any CAD change and the ratios,
the joint signs and the driving geometry all follow.
