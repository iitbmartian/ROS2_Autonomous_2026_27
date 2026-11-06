#!/bin/bash
# After the rock_field rerun: 3D check again with the step hybrid (68caf06) and the runner's
# fresh participant per stack (b56d5af), then figures against the first check and the 2D baseline.
E=$HOME/.ros/rover_3d_nav
L=$HOME/rover_3d_nav_results/batch.log
until grep -q "after_batch3 done" $L; do sleep 60; done
source $E/tools/simenv.sh
cd /home/ram/ROS2_Autonomous_2026_27
OUT=$HOME/.ros/rover_benchmark/2026-11-03_3d_check2
echo "$(date '+%F %H:%M') 3D check 2 (step hybrid, runner fix), commit $(git rev-parse --short HEAD)" >> $L
ros2 run rover_nav2 benchmark --worlds slopes crater_field rock_field urc_mix marsyard ledge bars \
  --runs 1 --config 3d --tag 3d_v2 --out $OUT >> $E/evidence/check2.out 2>&1
python3 $E/tools/fig_benchmark.py check_3d_v2 $OUT 3d_v2=3d >> $L 2>&1
python3 $E/tools/fig_benchmark.py compare_3d_v1_v2 ~/.ros/rover_benchmark/2026-11-03_3d_check $OUT \
  3d=2d 3d_v2=3d >> $L 2>&1
echo "$(date '+%F %H:%M') after_batch4 done" >> $L
