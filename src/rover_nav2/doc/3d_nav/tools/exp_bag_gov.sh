#!/bin/bash
# Two bags of marsyard behind_left with the governor diagnostics (u_ahead, reason, map age), to find
# why the governor holds its 0.05 m/s floor for about 65 s 2.4 to 1.2 m before the goal. Default
# tree, D 250. Args: pid to wait for.
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log
while kill -0 $1 2>/dev/null; do sleep 30; done
source $E/tools/simenv.sh; cd /home/ram/ROS2_Autonomous_2026_27; unset ROVER_COUPLING_K ROVER_COUPLING_D ROVER_COUPLING_MAX
export EXTRA_TOPICS="/terrain_speed_governor/diagnostics"
for i in 1 2; do
  BURN=8 $E/tools/guard.sh 1800 $E/tools/bag_run.sh gov$i marsyard.sdf -5.0 4.0 2.5 ackermann
  for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null; done
  python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1
  b=$(ls -td $E/bags/run_gov${i}_* | grep -v '[.][a-z]*$' | head -1)
  echo "$(date '+%F %H:%M') exp_bag_gov: $i $(tr '\n' ' ' < $b.goal.out 2>/dev/null), bag $b" >> $L
done
