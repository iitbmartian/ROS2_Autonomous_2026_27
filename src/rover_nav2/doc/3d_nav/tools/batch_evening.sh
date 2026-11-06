#!/bin/bash
# 2026-11-03 evening: 2d_bounded smoke run, 3D check 2 (step hybrid, runner leak fix), then
# 2d_bounded on all worlds, 1 run per goal like the 3D check. Progress in batch.log.
E=$HOME/.ros/rover_3d_nav
L=$HOME/rover_3d_nav_results/batch.log
B=$HOME/.ros/rover_benchmark
PF=/home/ram/ROS2_Autonomous_2026_27/install/rover_nav2/share/rover_nav2/config/nav2_params_sim_2d_bounded.yaml
WORLDS="slopes crater_field rock_field urc_mix marsyard ledge bars"
source $E/tools/simenv.sh
cd /home/ram/ROS2_Autonomous_2026_27
step() { echo "$(date '+%F %H:%M') $*" >> $L; }
step "evening batch start, commit $(git rev-parse --short HEAD)"
ros2 run rover_nav2 benchmark --worlds urc_mix --goals far_east --runs 1 --config 2d \
  --params-file $PF --tag 2d_bounded --out $B/2026-11-03_2d_bounded_smoke >> $E/evidence/evening.out 2>&1
step "2d_bounded smoke: $(tail -1 $B/2026-11-03_2d_bounded_smoke/runs.csv | cut -d, -f3-8)"
step "3D check 2"
ros2 run rover_nav2 benchmark --worlds $WORLDS --runs 1 --config 3d --tag 3d_v2 \
  --out $B/2026-11-03_3d_check2 >> $E/evidence/evening.out 2>&1
python3 $E/tools/fig_benchmark.py check_3d_v2 $B/2026-11-03_3d_check2 3d_v2=3d >> $L 2>&1
step "3D check 2 done"
step "2d_bounded, all worlds"
ros2 run rover_nav2 benchmark --worlds $WORLDS --runs 1 --config 2d --params-file $PF \
  --tag 2d_bounded --out $B/2026-11-03_2d_bounded >> $E/evidence/evening.out 2>&1
python3 $E/tools/fig_benchmark.py compare_evening $B/2026-11-03_baseline_2d_all \
  $B/2026-11-03_2d_bounded $B/2026-11-03_3d_check2 2d=2d 2d_bounded=swerve 3d_v2=3d >> $L 2>&1
step "evening batch done"
