# third_party

## gz_ros2_control (patched overlay)

Upstream: https://github.com/ros-controls/gz_ros2_control, tag `1.2.20` (the version
`ros-jazzy-gz-ros2-control` shipped when this was vendored). Only the `gz_ros2_control`
package is vendored; upstream's `gz_ros2_control_demos` and `gz_ros2_control_tests` are
left out so a workspace build doesn't compile them.
Building this workspace overlays it on top of the system package.

**The one change** is in `gz_ros2_control/src/gz_system.cpp`, `GazeboSimSystem::write()`,
marked `rover_gazebo patch`. At the start of each joint's update it removes the command
component left behind by the *other* control mode: `JointForceCmd` unless the joint
is effort-only, and `JointVelocityCmd` when it is.

**Why.** Upstream creates a `JointForceCmd` the first time a joint runs in EFFORT mode
and never removes it. gz-sim's Physics system applies `JointForceCmd` in preference to
`JointVelocityCmd` whenever both exist ("Only set joint velocity if joint force is not
set", gz-sim `Physics.cc`). So after rover_gazebo's explicit_pid mode hands the
steer/drive joints back to the position/velocity controllers, the switch reports
success but every later command is silently ignored and the joints go limp until the
sim restarts. See `src/rover_gazebo/doc/VERIFICATION.md`.

Drop this overlay once upstream removes stale command components on a mode switch
(not fixed as of the `rolling` branch, 2026-10-12).
