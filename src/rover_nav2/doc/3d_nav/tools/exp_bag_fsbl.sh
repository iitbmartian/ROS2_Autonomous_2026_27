#!/bin/bash
# Three bags of marsyard behind_left with global_fine_step, governor diagnostics and both
# costmaps: every 3D timeout reaches 0.5 m at 50 to 67 s and then sits 0.33 to 0.48 m out for
# 180 s with no recovery and no error. Args: pid to wait for.
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log
while kill -0 $1 2>/dev/null; do sleep 30; done
source $E/tools/simenv.sh; cd /home/ram/ROS2_Autonomous_2026_27; unset ROVER_COUPLING_K ROVER_COUPLING_D ROVER_COUPLING_MAX
export EXTRA_LAUNCH="global_fine_step:=true"
export EXTRA_TOPICS="/terrain_speed_governor/diagnostics /local_costmap/costmap /global_costmap/costmap"
for i in 1 2 3; do
  BURN=8 $E/tools/guard.sh 1800 $E/tools/bag_run.sh fsbl$i marsyard.sdf -5.0 4.0 2.5 ackermann
  for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null; done
  python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1
  b=$(ls -td $E/bags/run_fsbl${i}_* | grep -v '[.][a-z]*$' | head -1)
  echo "$(date '+%F %H:%M') exp_bag_fsbl: $i $(tr '\n' ' ' < $b.goal.out 2>/dev/null), bag $b" >> $L
done
echo "$(date "+%F %H:%M") exp_bag_fsbl: done" >> $L
