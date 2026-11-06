#!/bin/bash
# Ram 2026-11-04: tune navigation on ground-truth odometry until robust, rerun on the EKF once
# rover_ekf fixes the slip drift. 3D and 2d_bounded on all worlds, one run per goal.
E=$HOME/.ros/rover_3d_nav
L=$HOME/rover_3d_nav_results/batch.log
B=$HOME/.ros/rover_benchmark
W="slopes crater_field rock_field urc_mix marsyard ledge bars"
source $E/tools/simenv.sh
cd /home/ram/ROS2_Autonomous_2026_27
echo "$(date '+%F %H:%M') gt batch start, commit $(git rev-parse --short HEAD)" >> $L
ros2 run rover_nav2 benchmark --worlds $W --runs 1 --config 3d --tag 3d_gt \
  --launch-args odom_source:=ground_truth --out $B/2026-11-04_3d_gt >> $E/evidence/gt.out 2>&1
python3 $E/tools/fig_benchmark.py check_3d_gt $B/2026-11-04_3d_gt 3d_gt=3d >> $L 2>&1
echo "$(date '+%F %H:%M') gt 3d done" >> $L
ros2 run rover_nav2 benchmark --worlds $W --runs 1 --config 2d --tag 2d_bounded_gt \
  --params-file /home/ram/ROS2_Autonomous_2026_27/install/rover_nav2/share/rover_nav2/config/nav2_params_sim_2d_bounded.yaml \
  --launch-args odom_source:=ground_truth --out $B/2026-11-04_2d_bounded_gt >> $E/evidence/gt.out 2>&1
python3 $E/tools/fig_benchmark.py compare_gt $B/2026-11-04_2d_bounded_gt $B/2026-11-04_3d_gt \
  2d_bounded_gt=swerve 3d_gt=3d >> $L 2>&1
echo "$(date '+%F %H:%M') gt batch done" >> $L
