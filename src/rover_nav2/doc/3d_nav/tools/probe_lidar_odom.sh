#!/bin/bash
# 3D stack with RTAB-Map ICP lidar odometry in place of the wheel EKF, on the worlds where the
# night runs showed metres of odometry drift (wheel slip on rocks and bars).
E=$HOME/.ros/rover_3d_nav
L=$HOME/rover_3d_nav_results/batch.log
B=$HOME/.ros/rover_benchmark
source $E/tools/simenv.sh
cd /home/ram/ROS2_Autonomous_2026_27
echo "$(date '+%F %H:%M') lidar odom probe (ICP L2 rate fix), commit $(git rev-parse --short HEAD)" >> $L
ros2 run rover_nav2 benchmark --worlds urc_mix rock_field bars crater_field marsyard \
  --runs 1 --config 3d --tag 3d_lidarodom --launch-args odom_source:=lidar \
  --out $B/2026-11-04_3d_lidarodom >> $E/evidence/lidarodom.out 2>&1
python3 $E/tools/fig_benchmark.py lidarodom $B/2026-11-04_3d_check3 $B/2026-11-04_3d_lidarodom \
  3d_v3=3d 3d_lidarodom=swerve >> $L 2>&1
echo "$(date '+%F %H:%M') lidar odom probe done" >> $L
