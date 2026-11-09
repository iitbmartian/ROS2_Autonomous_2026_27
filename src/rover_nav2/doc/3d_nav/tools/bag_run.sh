#!/bin/bash
# One recorded 3D run (plugin suspension, ground truth, u_free 0.3) with a bag of the IMU, joints,
# command chain, plans and speed limit. Args: tag world x y yaw drive(ackermann/swerve).
# Hard wall cap 1200 s on the goal.
E=$HOME/.ros/rover_3d_nav; OUT=$E/bags/run_$1_$(date +%m%d_%H%M)
source $E/tools/simenv.sh
export GZ_PARTITION=rover_bag ROVER_BENCH_RUN=bag_run
cd /home/ram/ROS2_Autonomous_2026_27
mkdir -p $E/bags
setsid ros2 launch rover_bringup pipeline_3d.launch.py world:=$2 gui:=false viz:=false \
  odom_source:=ground_truth drive:=$6 suspension:=plugin governor_v_nominal:=0.5 governor_u_free:=0.3 \
  $( [ "$6" = swerve ] && echo nav_extra_overlay:=$E/exp/nav2_3d_swerve_v2.yaml ) > $OUT.launch.log 2>&1 &
LP=$!
for i in $(seq 120); do ros2 lifecycle get /bt_navigator 2>/dev/null | grep -q active && break; sleep 5; done
sleep 10
ros2 bag record -o $OUT /imu/data /odom /joint_states /cmd_vel_nav /cmd_vel_smoothed /cmd_vel \
  /rover/steer_mode /plan /local_plan /speed_limit /behavior_tree_log /tf_static > $OUT.bag.log 2>&1 &
BP=$!
sleep 3
timeout 1300 python3 $E/tools/video/send_goal.py $3 $4 $5 1200 > $OUT.goal.out 2>&1
kill -INT $BP; for i in $(seq 20); do kill -0 $BP 2>/dev/null || break; sleep 1; done; kill -TERM $BP 2>/dev/null; wait $BP
kill -INT -$LP; sleep 10; kill -9 -$LP 2>/dev/null
for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null; done
python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1
grep -h "tip risk" $OUT.launch.log | head -3 >> $OUT.goal.out
echo "$(date '+%F %H:%M') bag_run: $OUT $(tr '\n' ' ' < $OUT.goal.out)" >> $HOME/rover_3d_nav_results/batch.log
