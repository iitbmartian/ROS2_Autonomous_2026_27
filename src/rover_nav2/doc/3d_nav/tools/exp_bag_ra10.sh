#!/bin/bash
# Three bags of rock_field ahead_10m (10, 0) with the v1.1 defaults (fine step, re-approach tree):
# 7/12 with the re-approach tree across today's runs, 3/3 in every run without it. Aborts sit 1.1 m
# short after 13 "Failed to make progress". Local costmap, governor diagnostics, BT log. Args: pid.
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log
while kill -0 $1 2>/dev/null; do sleep 30; done
source $E/tools/simenv.sh; cd /home/ram/ROS2_Autonomous_2026_27; unset ROVER_COUPLING_K ROVER_COUPLING_D ROVER_COUPLING_MAX
export EXTRA_TOPICS="/terrain_speed_governor/diagnostics /local_costmap/costmap /global_costmap/costmap"
for i in 1 2 3; do
  BURN=8 $E/tools/guard.sh 1200 $E/tools/bag_run.sh ra10_$i rock_field.sdf 10.0 0.0 0.0 ackermann
  for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null; done
  python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1
  b=$(ls -td $E/bags/run_ra10_${i}_* | grep -v '[.][a-z]*$' | head -1)
  echo "$(date '+%F %H:%M') exp_bag_ra10: $i $(tr '\n' ' ' < $b.goal.out 2>/dev/null), bag $b" >> $L
done
echo "$(date '+%F %H:%M') exp_bag_ra10: done" >> $L
