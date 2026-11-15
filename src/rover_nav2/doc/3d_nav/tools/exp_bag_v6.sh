#!/bin/bash
# Bags of swerve v6 where it does worse than v4: urc_mix far_south (0/2 vs 3/3) and slopes
# sidehill_uphill (0/2 vs 1/3). Global costmap and governor diagnostics. Args: pid to wait for.
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log
while kill -0 $1 2>/dev/null; do sleep 30; done
source $E/tools/simenv.sh; cd /home/ram/ROS2_Autonomous_2026_27; unset ROVER_COUPLING_K ROVER_COUPLING_D ROVER_COUPLING_MAX
export EXTRA_TOPICS="/terrain_speed_governor/diagnostics /global_costmap/costmap"
for g in "v6south urc_mix.sdf 30.0 -20.0 -0.6" "v6side slopes.sdf 11.0 7.5 1.5708"; do set -- $g
  BURN=8 $E/tools/guard.sh 1500 $E/tools/bag_run.sh $1 $2 $3 $4 $5 swerve $E/exp/nav2_3d_swerve_v6.yaml
  for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null; done
  python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1
  b=$(ls -td $E/bags/run_$1_* | grep -v '[.][a-z]*$' | head -1)
  echo "$(date '+%F %H:%M') exp_bag_v6: $1 $(tr '\n' ' ' < $b.goal.out 2>/dev/null), bag $b" >> $L
done
echo "$(date '+%F %H:%M') exp_bag_v6: done" >> $L
