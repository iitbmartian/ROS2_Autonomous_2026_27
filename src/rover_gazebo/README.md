# Rover in Gazebo

Everything needed to run the four-wheel-steer rocker-differential rover in Gazebo
under ROS 2 Jazzy. Drop `rover_gazebo` into a workspace and build it.

```bash
cp -r rover_gazebo ~/your_ws/src/
cd ~/your_ws && colcon build --packages-select rover_gazebo
source install/setup.bash
```

## Run it

```bash
ros2 launch rover_gazebo rover_sim.launch.py
```

and in another terminal:

```bash
ros2 run rover_gazebo teleop_rover.py
```

| Key | Action |
|---|---|
| W / S | forward / reverse |
| A / D | left / right, meaning depends on the mode |
| C | explicit mode only: sweep the wheels back to straight ahead |
| Space | stop driving; the wheels stay pointed where they are |
| Shift plus a letter | boost |
| 1 / 2 / 3 / 4 | Ackermann / crab / spot / explicit |

Ackermann follows an arc, front and rear counter-steering. Crab slides the rover
sideways without changing its heading. Spot turns it in place; W and S do nothing there.

Explicit is the odd one out. The other three ask for a body motion and work out what the
wheels must do to deliver it. Explicit skips that step entirely. A and D sweep all four
wheels together about their own vertical axes, slowly, and the wheels hold that angle
when you let go. W and S then simply spin the wheels, and the rover goes wherever they
drag it. C sweeps them back to straight, and the current angle is shown on screen as you
steer. Nothing here works out where the rover will end up, which is rather the point: it
is the mode for lining the rover up by hand. Shift boosts W and S but never the aiming,
which stays at 15 degrees a second so you can stop where you meant to.

Hold the aim near 90 degrees while driving and the chassis will creep and yaw on its own, worse the closer the aim sits to sideways. That is not a steering-joint limit: the joint has margin to spare at 90 degrees. It is the suspension coupling's own residual, already measured for crab mode in `doc/VERIFICATION.md`, finding nothing to resist it once the wheels stop pointing along the chassis.

Other worlds: `world:=ledge.sdf` ramps the left-hand wheels onto a 10 cm ledge and
keeps them there for four metres, and `world:=bars.sdf` lays two bars across the path,
8 cm under the right and 12 cm under the left. Both exist to make the suspension work
for its living.

`world:=husarion_world.sdf` is Husarion's open world, vendored from their ROSbot
simulation assets. Its mesh lives in `models/HusarionLogo`, which the package's
environment hook puts on `GZ_SIM_RESOURCE_PATH` so that `model://` URIs resolve.

The logo in it is not a floor decal. Every node in `husarion_logo.dae` carries a x100
scale matrix, so at the world's `<scale>6</scale>` the logo is 24.6 x 24.6 m and stands
1.80 m tall, covering x -11.96..12.66 and y -12.32..12.31. The world origin is inside it,
so **spawn the rover clear of the plate or it starts 1.8 m underground**, invisible and
unable to move:

```bash
ros2 launch rover_gazebo rover_sim.launch.py world:=husarion_world.sdf x:=-13.0
```

The ground plane is 60 m here rather than Husarion's 25 m, because their plane is smaller
than the logo standing on it and left nowhere to put the rover. A plane collision is
infinite either way, so only the visible square changes.

## Sensors

Three onboard sensors, bridged from Gazebo to ROS 2 topics:

| Topic | Type | Sensor | Rate |
|---|---|---|---|
| `/imu/data` | `sensor_msgs/Imu` | IMU, centre of the chassis | 100 Hz |
| `/lidar/points` | `sensor_msgs/PointCloud2` | 3D GPU lidar, roof mount, 360 deg | 5 Hz |
| `/lidar/scan` | `sensor_msgs/LaserScan` | same lidar, re-bridged as a 2D scan | 5 Hz |
| `/camera/image_raw` | `sensor_msgs/Image` | RGBD camera, front mount, colour | 15 Hz |
| `/camera/depth_image` | `sensor_msgs/Image` | same camera, depth | 15 Hz |
| `/camera/points` | `sensor_msgs/PointCloud2` | same camera, coloured point cloud | 15 Hz |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | same camera, intrinsics | 15 Hz |

Ground-truth odometry is also bridged, on `/odom` (`nav_msgs/Odometry`), taken directly
from Gazebo rather than from any sensor. See `doc/INTEGRATION.md` for the full topic
list, including the driving and control topics. Sensor definitions live in
`urdf/rover_sensors.xacro`, and the topic bridge in `config/bridge.yaml`.

## What this had to fix

The SolidWorks export could not be loaded into Gazebo at all, for four separate reasons.

**The suspension has four closed loops.** A connector bar per side ties the front rocker
to the rear one, and a differential bar reaches both rear rockers through pushrods.
Between them they leave the suspension a single degree of freedom, a diagonal warp that
keeps all four wheels loaded over uneven ground. URDF is a tree and cannot say any of
it, so the exporter left the bars attached at one end, and without them the suspension
folds up under the rover's own weight. The constraints are recovered here as arithmetic
and enforced at runtime as joint torques, by `rocker_coupling_node`. The physics engine
this package runs on, DART, has no solver-level way to enforce a constraint like this
and explicitly refuses to, so this is the only coupling mechanism this package ships. A
different engine, Bullet-Featherstone, was confirmed on an isolated test to enforce
`<mimic>` constraints correctly and could in principle replace this, but it has real
open questions for a model this shape; see `doc/VERIFICATION.md`.

**The model is Y-up.** Spawned as exported, the rover lies on its side.

**Every joint declares `effort="0"`,** which Gazebo reads as a zero force limit, so
nothing could be driven.

**Links and joints share names.** URDF allows it, SDF does not, and Gazebo refuses to
load the model. Joints therefore carry a `_joint` suffix here; link and frame names are
unchanged from the CAD.

`doc/DESIGN.md` works through the mechanism and the arithmetic.
`doc/INTEGRATION.md` covers topics, frames and how to fit this to an existing stack.
`doc/VERIFICATION.md` says what was measured, and what could not be.
`doc/updates.md` is the change log: what changed, in which files, and why.

## Read this before trusting it

Everything, including the suspension coupling, the driving, the steering modes and the
obstacle behaviour, was run end to end on Gazebo Fortress, ROS 2 Humble. Numbers are in
`doc/VERIFICATION.md`. The one exception is explicit mode, which was added after those
runs and has not been measured on Fortress: the harness for it is there, the numbers are
not. The `sim:=harmonic`/`sim:=fortress` argument was meant to only pick plugin names
between the two Gazebo versions, on the assumption that nothing else about the model
depends on which one you run. That assumption does not hold: Gazebo Harmonic, which is
what `sim:=harmonic` actually targets and the only version this package has since been
tested on, shows a real jitter that Fortress did not, visible even driving straight
ahead. `doc/VERIFICATION.md` has the numbers for both versions side by side.

The suspension coupling is a stiff torque loop closed over ROS topics at 500 Hz, not a
physics-engine constraint, because DART, the physics engine this package runs on, has no
solver-level way to enforce this kind of joint coupling. That means it holds the
suspension to within a degree or two rather than exactly, and on Gazebo Harmonic that
loop is also the confirmed dominant source of a continuous speed and yaw jitter present
even driving straight. `doc/VERIFICATION.md` has the measured residuals, the jitter
numbers, and where both show up.

## Layout

```
rover_gazebo/
  urdf/       rover.urdf.xacro is the entry point; rover_body.xacro is generated
  meshes/     the 22 STLs, copied from the export unchanged
  config/     controllers, driving limits, generated geometry, topic bridge
  launch/     rover_sim.launch.py
  worlds/     flat, one-sided step, two-bar course
  tools/      derive_ratios.py, generate_description.py
  doc/        design notes, integration guide, verification results, change log
reference/    the original export, untouched, for provenance
```

Nothing in `urdf/rover_body.xacro` or `config/rover_kinematics.yaml` is hand-written.
Both come out of `tools/`, which reads the original export in `reference/`. After a CAD
change, re-run the two generators and the ratios, joint signs and driving geometry all
follow.
