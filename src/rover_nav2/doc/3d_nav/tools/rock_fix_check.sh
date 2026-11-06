#!/bin/bash
# rock_field after the escape-cap fix (983e315), ground-truth odom, fresh runner.
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log; B=$HOME/.ros/rover_benchmark
source $E/tools/simenv.sh; cd /home/ram/ROS2_Autonomous_2026_27
echo "$(date '+%F %H:%M') rock fix check, commit $(git rev-parse --short HEAD)" >> $L
ros2 run rover_nav2 benchmark --worlds rock_field --runs 1 --config 3d --tag 3d_gt_v4 \
  --launch-args odom_source:=ground_truth --out $B/2026-11-05_3d_gt_rockfix >> $E/evidence/rockfix.out 2>&1
echo "$(date '+%F %H:%M') rock fix check done" >> $L
