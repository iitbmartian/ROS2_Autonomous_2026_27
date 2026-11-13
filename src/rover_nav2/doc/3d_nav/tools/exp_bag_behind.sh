#!/bin/bash
# Bag of marsyard behind_left (-5, 4, yaw 2.5) on the default 3D tree at D 250, which times out
# 0/3 about 0.46 m out at D 250 (2/3 at D 200). Shows the near-goal behaviour. Args: pid to wait for.
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log
while kill -0 $1 2>/dev/null; do sleep 30; done
source $E/tools/simenv.sh; cd /home/ram/ROS2_Autonomous_2026_27; unset ROVER_COUPLING_K ROVER_COUPLING_D ROVER_COUPLING_MAX
BURN=8 $E/tools/guard.sh 1800 $E/tools/bag_run.sh behind marsyard.sdf -5.0 4.0 2.5 ackermann
for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null; done
python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1
b=$(ls -td $E/bags/run_behind_* | grep -v '[.][a-z]*$' | head -1)
echo "$(date '+%F %H:%M') exp_bag_behind: $(tr '\n' ' ' < $b.goal.out 2>/dev/null), bag $b" >> $L
