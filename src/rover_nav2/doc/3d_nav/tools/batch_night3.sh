#!/bin/bash
# Replaces batch_night2.sh (stopped 01:30 to add the BT fix): finish 3D v4 (escape fix only) on
# all worlds, install the consistent-replanning tree (18b8adc), 3D v5 on all worlds, then the
# 3-run head-to-head with v5. Ground-truth odometry, one runner per world.
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log; B=$HOME/.ros/rover_benchmark
PF=/home/ram/ROS2_Autonomous_2026_27/install/rover_nav2/share/rover_nav2/config/nav2_params_sim_2d_bounded.yaml
W="marsyard crater_field slopes urc_mix rock_field ledge bars"
# The stopped script's runner finishes its current world first.
while pgrep -f "rover_nav2/benchmark --worlds" > /dev/null; do sleep 20; done
source $E/tools/simenv.sh; cd /home/ram/ROS2_Autonomous_2026_27
source $E/tools/run_fn.sh
echo "$(date '+%F %H:%M') night3: finish 3D v4, commit $(git rev-parse --short HEAD) (v4 install has no BT change)" >> $L
for w in $W; do run 3d 3d_gt_v4 2026-11-05_3d_gt_v4 1 $w; done
python3 $E/tools/fig_benchmark.py check_3d_gt_v4 $B/2026-11-05_3d_gt_v4 3d_gt_v4=3d >> $L 2>&1
colcon build --packages-select rover_nav2 > $E/evidence/night3_build.out 2>&1
echo "$(date '+%F %H:%M') night3: v4 done, rover_nav2 built with the 3D tree, 3D v5" >> $L
for w in $W; do run 3d 3d_gt_v5 2026-11-05_3d_gt_v5 1 $w; done
python3 $E/tools/fig_benchmark.py check_3d_gt_v5 $B/2026-11-05_3d_gt_v5 3d_gt_v5=3d >> $L 2>&1
echo "$(date '+%F %H:%M') night3: v5 done, head-to-head" >> $L
for w in $W; do
  run 3d 3d_gt_h2h 2026-11-05_h2h_3d 3 $w
  run 2d 2d_bounded_gt_h2h 2026-11-05_h2h_2db 3 $w --params-file $PF
done
python3 $E/tools/fig_benchmark.py h2h_gt $B/2026-11-05_h2h_2db $B/2026-11-05_h2h_3d \
  2d_bounded_gt_h2h=swerve 3d_gt_h2h=3d >> $L 2>&1
echo "$(date '+%F %H:%M') night3 done" >> $L
