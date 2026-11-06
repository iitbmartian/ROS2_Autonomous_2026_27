#!/bin/bash
# 2026-11-04 night: the 3D check 2 runs lost to startup hangs, then 2d_bounded (unknown free) on
# all worlds, then the comparison figures. Progress in batch.log.
E=$HOME/.ros/rover_3d_nav
L=$HOME/rover_3d_nav_results/batch.log
B=$HOME/.ros/rover_benchmark
PF=/home/ram/ROS2_Autonomous_2026_27/install/rover_nav2/share/rover_nav2/config/nav2_params_sim_2d_bounded.yaml
WORLDS="slopes crater_field rock_field urc_mix marsyard ledge bars"
source $E/tools/simenv.sh
cd /home/ram/ROS2_Autonomous_2026_27
step() { echo "$(date '+%F %H:%M') $*" >> $L; }
step "night batch start, commit $(git rev-parse --short HEAD)"
ros2 run rover_nav2 benchmark --worlds $WORLDS --runs 1 --config 3d --tag 3d_v2 \
  --out $B/2026-11-03_3d_check2 >> $E/evidence/night.out 2>&1
python3 $E/tools/fig_benchmark.py check_3d_v2 $B/2026-11-03_3d_check2 3d_v2=3d >> $L 2>&1
step "3D check 2 reruns done"
ros2 run rover_nav2 benchmark --worlds $WORLDS --runs 1 --config 2d --params-file $PF \
  --tag 2d_bounded --out $B/2026-11-04_2d_bounded >> $E/evidence/night.out 2>&1
python3 $E/tools/fig_benchmark.py compare_2d_2db_3d $B/2026-11-03_baseline_2d_all \
  $B/2026-11-04_2d_bounded $B/2026-11-03_3d_check2 2d=2d 2d_bounded=swerve 3d_v2=3d >> $L 2>&1
step "night batch done"
