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
| Space | stop |
| Shift plus a letter | boost |
| 1 / 2 / 3 | Ackermann / crab / spot |

Ackermann follows an arc, front and rear counter-steering. Crab slides the rover
sideways without changing its heading. Spot turns it in place; W and S do nothing there.

Other worlds: `world:=ledge.sdf` ramps the left-hand wheels onto a 10 cm ledge and
keeps them there for four metres, and `world:=bars.sdf` lays two bars across the path,
8 cm under the right and 12 cm under the left. Both exist to make the suspension work
for its living.

`world:=husarion_world.sdf` is Husarion's open world, vendored from their ROSbot
simulation assets: a 25 m grey plane with the Husarion logo laid into the floor. It is
flat, so it asks nothing of the suspension. Its mesh lives in `models/HusarionLogo`,
which the package's environment hook puts on `GZ_SIM_RESOURCE_PATH` so that `model://`
URIs resolve.

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
and enforced at runtime as joint torques, by `rocker_coupling_node`. Gazebo has no
solver-level way to enforce a constraint like this, so this is the only coupling
mechanism this package ships; there is no `<mimic>`-based alternative.

**The model is Y-up.** Spawned as exported, the rover lies on its side.

**Every joint declares `effort="0"`,** which Gazebo reads as a zero force limit, so
nothing could be driven.

**Links and joints share names.** URDF allows it, SDF does not, and Gazebo refuses to
load the model. Joints therefore carry a `_joint` suffix here; link and frame names are
unchanged from the CAD.

`doc/DESIGN.md` works through the mechanism and the arithmetic.
`doc/INTEGRATION.md` covers topics, frames and how to fit this to an existing stack.
`doc/VERIFICATION.md` says what was measured, and what could not be.

## Read this before trusting it

Everything, including the suspension coupling, the driving, the steering modes and the
obstacle behaviour, was run end to end on Gazebo Fortress, ROS 2 Humble. Numbers are in
`doc/VERIFICATION.md`. The `sim:=harmonic`/`sim:=fortress` argument only picks plugin
names between the two Gazebo versions; nothing else about the model depends on which
one you run.

The suspension coupling is a stiff torque loop closed over ROS topics at 500 Hz, not a
physics-engine constraint, because Gazebo has no solver-level way to enforce this kind
of joint coupling. That means it holds the suspension to within a degree or two rather
than exactly. `doc/VERIFICATION.md` has the measured residuals and where they show up.

## Layout

```
rover_gazebo/
  urdf/       rover.urdf.xacro is the entry point; rover_body.xacro is generated
  meshes/     the 22 STLs, copied from the export unchanged
  config/     controllers, driving limits, generated geometry, topic bridge
  launch/     rover_sim.launch.py
  worlds/     flat, one-sided step, two-bar course
  tools/      derive_ratios.py, generate_description.py
  doc/        design notes, integration guide, verification results
reference/    the original export, untouched, for provenance
```

Nothing in `urdf/rover_body.xacro` or `config/rover_kinematics.yaml` is hand-written.
Both come out of `tools/`, which reads the original export in `reference/`. After a CAD
change, re-run the two generators and the ratios, joint signs and driving geometry all
follow.
