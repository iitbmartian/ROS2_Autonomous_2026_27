# Fitting this into an existing pipeline

## What the package puts on the graph

| Topic | Type | Direction | Where it comes from |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | in | you, teleop, or a nav stack |
| `/rover/steer_mode` | `std_msgs/String` | in | `ackermann`, `crab`, `spot` or `explicit` |
| `/joint_states` | `sensor_msgs/JointState` | out | `joint_state_broadcaster` |
| `/odom` | `nav_msgs/Odometry` | out | Gazebo, bridged. Ground truth |
| `/tf`, `/tf_static` | | out | `robot_state_publisher` |
| `/steer_controller/commands` | `std_msgs/Float64MultiArray` | in | four steering angles, radians |
| `/wheel_controller/commands` | `std_msgs/Float64MultiArray` | in | four wheel speeds, rad/s |
| `/rover/steer_aim` | `std_msgs/Float64` | out | explicit mode's shared wheel angle, radians |

Onboard sensors, all bridged from Gazebo:

| Topic | Type | Sensor | Rate |
|---|---|---|---|
| `/imu/data` | `sensor_msgs/Imu` | IMU, centre of the chassis | 100 Hz |
| `/lidar/points` | `sensor_msgs/PointCloud2` | 3D GPU lidar, roof mount, 360 deg | 5 Hz |
| `/lidar/scan` | `sensor_msgs/LaserScan` | same lidar, re-bridged as a 2D scan | 5 Hz |
| `/camera/image_raw` | `sensor_msgs/Image` | RGBD camera, front mount, colour | 15 Hz |
| `/camera/depth_image` | `sensor_msgs/Image` | same camera, depth | 15 Hz |
| `/camera/points` | `sensor_msgs/PointCloud2` | same camera, coloured point cloud | 15 Hz |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | same camera, intrinsics | 15 Hz |

`/odom` is ground truth taken from the simulator, not wheel odometry. Treat it as the
thing you check your estimator against, not as an input to it. Real wheel odometry can
be integrated from `/joint_states`, which carries all eight driven joints plus the four
rocker joints.

## Driving it from your own code

Publish a `Twist` on `/cmd_vel`. What the fields mean depends on the mode:

| Mode | `linear.x` | `linear.y` | `angular.z` |
|---|---|---|---|
| `ackermann` | forward speed | ignored | yaw rate, capped by the tightest turn radius |
| `crab` | forward speed | sideways speed | ignored |
| `spot` | ignored | ignored | spin rate |
| `explicit` | wheel speed, all four | ignored | steering sweep rate |

Ackermann is the mode a nav stack wants: it is ordinary differential-drive `Twist`
semantics, with the yaw rate limited so the turn stays inside what the steering can
reach. Crab and spot are there because the rover can do them and a planner might want
them, not because anything needs them by default.

Explicit is not a body-velocity mode at all, so read its row carefully. `angular.z` is
still rad/s, but it is the rate the steering sweeps at rather than the rate the body
yaws at, and `rover_kinematics_node` integrates it into one angle shared by all four
wheels. `linear.x` is then handed to every wheel as its own speed, scaled only by that
wheel's `drive_sign`. Nothing solves for where the rover will go.

Two things follow from that. The aim persists: stop publishing and the 0.5 s
`command_timeout` zeroes the rate, so the drive coasts down while the wheels stay
pointed where you left them. And because the node owns the angle rather than the
publisher, it echoes it on `/rover/steer_aim`, which is how you close the loop if you
want to command an absolute angle rather than a rate. Switching into explicit mode
sweeps the wheels back to straight ahead first, so you always start from a known pose.

One consequence follows you wherever the aim comes from, hand-driven or published by your own code: hold a large aim while driving and the chassis creeps and yaws on its own, worse the closer the aim sits to 90 degrees. This is not the steering joint running out of travel; at 90 degrees it still has ten degrees of margin before its hard stop. It is the suspension coupling's own residual, which exists at every aim and every mode, finding nothing to resist it once the wheels stop pointing along the chassis rather than across it. `doc/VERIFICATION.md` has the numbers, measured under crab mode, which reaches the same wheel geometry. Nothing here saturates, so a tighter aim clamp trades away range for less exposure to this, it does not remove a limit fight that was never happening.

To bypass the kinematics entirely, stop `rover_kinematics_node` and publish the two
`Float64MultiArray` command topics yourself. Joint order is front-left, front-right,
back-left, back-right in both. Signs are in `config/rover_kinematics.yaml`; one of the
four wheel joints turns the opposite way to the others because its drive axis is
inverted in the CAD, so do not assume they are uniform.

## Swapping the controllers

`config/controllers.yaml` is an ordinary ros2_control configuration. The steering
controller is a position group controller and the wheel controller a velocity group
controller, both plain forward controllers with no feedback of their own, since the
feedback lives in the simulator's hardware interface.

If you would rather drive the joints from a trajectory controller, or add a
diff-drive-style controller, replace the entries in that file and spawn yours instead.
The hardware interface exports `position`, `velocity` and `effort` state on all eight
driven joints, `position` command on the four steering joints and `velocity` command on
the four wheels, so most controllers will find what they need.

## Frames

```
odom
  base_footprint          on the ground, under the rover centre
    base_link             ROS convention: x forward, y left, z up
      chassis_link        the CAD frame: x back, y up, z left
        FLS ... FRD       everything else, in CAD frames
```

`base_footprint` to `base_link` is a fixed 0.7225 m lift, which is where the ground sits
relative to the chassis origin. `base_link` to `chassis_link` is the rotation that
undoes the export's Y-up convention. Point your nav stack at `base_link` or
`base_footprint` and ignore the rest.

Nothing publishes `odom` to `base_footprint`. That transform is your estimator's job.
The `rover_ekf` package in the ROS workspace next door already consumes `/imu` and
`/odom`, which is why both are bridged with those names.

## Adding sensors

Sensors go in `urdf/rover_sensors.xacro` next to the IMU, as a `<gazebo reference=...>`
block with a `<sensor>` inside, and then a line in `config/bridge.yaml` to carry the
topic across. The worlds deliberately declare no plugins, so Gazebo loads its default
system set, which already includes the sensor systems. If you write your own world,
either leave its plugin list empty or copy the default set in, or sensors will go quiet
with no error.

## Things that will bite

**Joint names carry a `_joint` suffix.** `FLD_joint`, not `FLD`. SDF will not load a
model whose joint shares a name with a link, and the CAD export named them identically.
Link and frame names are unchanged.

**The rocker joints already have an owner.** All four carry effort command interfaces,
claimed by `rocker_coupling_controller` and driven by `rocker_coupling_node`. Do not
spawn a second controller against them.

**The suspension has one degree of freedom, not four.** If you are reading
`/joint_states` to work out ride height, the four rocker angles are not independent;
three of them are fixed multiples of `FLS_joint`.

**Mass is as exported.** 63.9 kg of chassis and 9.87 kg per wheel, about 107 kg all in.
That is almost certainly SolidWorks default density on solid bodies rather than the real
rover. It matches the Unity simulation, so results carry across, but if you want the
real thing pass `mass_scale:=` to bring everything down together.
