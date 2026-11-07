#!/bin/bash
# One urc_mix far_south run (3D Ackermann, confirm config, ground truth) with a bag of the plan,
# controller output, speed limit, local costmap and BT log, to see why the rover parks 0.5 m out.
E=$HOME/.ros/rover_3d_nav; OUT=$E/bags/far_south_$(date +%m%d_%H%M)
source $E/tools/simenv.sh
export GZ_PARTITION=rover_bag
cd /home/ram/ROS2_Autonomous_2026_27
mkdir -p $E/bags
setsid ros2 launch rover_bringup pipeline_3d.launch.py world:=urc_mix.sdf gui:=false viz:=false \
  x:=0.0 y:=0.0 z:=0.02 yaw:=0.0 odom_source:=ground_truth governor_v_nominal:=0.5 governor_u_free:=0.0 \
  > $E/bags/far_south_launch.log 2>&1 &
LP=$!
for i in $(seq 120); do ros2 lifecycle get /bt_navigator 2>/dev/null | grep -q active && break; sleep 5; done
ros2 bag record -o $OUT /plan /local_plan /local_costmap/costmap /speed_limit /odom /cmd_vel_nav \
  /cmd_vel_smoothed /cmd_vel /behavior_tree_log /tf /tf_static > $E/bags/far_south_bag.log 2>&1 &
BP=$!
sleep 3
python3 $E/tools/video/send_goal.py 30.0 -20.0 -0.6 1500 > $E/bags/far_south_goal.out 2>&1
kill -INT $BP; for i in $(seq 20); do kill -0 $BP 2>/dev/null || break; sleep 1; done; kill -TERM $BP 2>/dev/null; wait $BP  # the recorder ignored one SIGINT on 2026-11-07
kill -INT -$LP; sleep 10; kill -9 -$LP 2>/dev/null
echo "$OUT $(tr '\n' ' ' < $E/bags/far_south_goal.out)"
