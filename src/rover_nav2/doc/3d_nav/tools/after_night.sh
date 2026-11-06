#!/bin/bash
# After batch_night.sh: install the escape_radius overlay, then the third 3D check (discovery
# fix, escape radius, EKF and RTAB-Map end-pose split, velocity chain counts, refusal scoring).
E=$HOME/.ros/rover_3d_nav
L=$HOME/rover_3d_nav_results/batch.log
B=$HOME/.ros/rover_benchmark
until grep -q "night batch done" $L; do sleep 60; done
source $E/tools/simenv.sh
cd /home/ram/ROS2_Autonomous_2026_27
colcon build --packages-select rover_nav2 > $E/evidence/after_night_build.out 2>&1
echo "$(date '+%F %H:%M') 3D check 3, commit $(git rev-parse --short HEAD)" >> $L
ros2 run rover_nav2 benchmark --worlds slopes crater_field rock_field urc_mix marsyard ledge bars \
  --runs 1 --config 3d --tag 3d_v3 --out $B/2026-11-04_3d_check3 >> $E/evidence/check3.out 2>&1
python3 $E/tools/fig_benchmark.py check_3d_v3 $B/2026-11-04_3d_check3 3d_v3=3d >> $L 2>&1
python3 $E/tools/fig_benchmark.py compare_all $B/2026-11-03_baseline_2d_all $B/2026-11-04_2d_bounded \
  $B/2026-11-04_3d_check3 2d=2d 2d_bounded=swerve 3d_v3=3d >> $L 2>&1
echo "$(date '+%F %H:%M') after_night done" >> $L
