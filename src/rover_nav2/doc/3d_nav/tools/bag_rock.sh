#!/bin/bash
# One rock_field ahead_10m run (3D, ground-truth odom) with a bag of the planner, controller,
# hazard and collision monitor topics, to find why the collision monitor keeps holding the rover.
E=$HOME/.ros/rover_3d_nav; OUT=$E/bags/rock_ahead_$(date +%H%M)
source $E/tools/simenv.sh
export GZ_PARTITION=rover_bag
cd /home/ram/ROS2_Autonomous_2026_27
mkdir -p $E/bags
setsid ros2 launch rover_bringup pipeline_3d.launch.py world:=rock_field.sdf gui:=false viz:=false \
  x:=0.0 y:=0.0 z:=0.02 yaw:=0.0 odom_source:=ground_truth > $E/bags/rock_launch.log 2>&1 &
LP=$!
for i in $(seq 120); do ros2 lifecycle get /bt_navigator 2>/dev/null | grep -q active && break; sleep 5; done
ros2 bag record -o $OUT /terrain/hazard_points /plan /local_plan /global_costmap/costmap \
  /local_costmap/costmap /collision_monitor_state /odom /cmd_vel_nav /cmd_vel_smoothed /cmd_vel \
  /tf /tf_static /behavior_tree_log > $E/bags/rock_bag.log 2>&1 &
BP=$!
sleep 3
python3 $E/tools/video/send_goal.py 10.0 0.0 0.0 700 > $E/bags/rock_goal.out 2>&1
kill -INT $BP; wait $BP
kill -INT -$LP; sleep 10; kill -9 -$LP 2>/dev/null
echo "$OUT $(cat $E/bags/rock_goal.out)"
