#!/bin/bash
# v2 start: bags of swerve (v4 overlay, fine step, D 250) on urc_mix mid_20m, 164 s against 47 s in
# Ackermann, with governor diagnostics and the steer mode, to see where swerve loses its time
# (wheel re-aiming, governor, controller model). Args: pid to wait for.
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log
while kill -0 $1 2>/dev/null; do sleep 30; done
source $E/tools/simenv.sh; cd /home/ram/ROS2_Autonomous_2026_27; unset ROVER_COUPLING_K ROVER_COUPLING_D ROVER_COUPLING_MAX
export EXTRA_TOPICS="/terrain_speed_governor/diagnostics /joint_states"
for i in 1 2; do
  BURN=8 $E/tools/guard.sh 1800 $E/tools/bag_run.sh swslow$i urc_mix.sdf 20.0 0.0 0.0 swerve $E/exp/nav2_3d_swerve_v4.yaml
  for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null; done
  python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1
  b=$(ls -td $E/bags/run_swslow${i}_* | grep -v '[.][a-z]*$' | head -1)
  echo "$(date '+%F %H:%M') exp_bag_swslow: $i $(tr '\n' ' ' < $b.goal.out 2>/dev/null), bag $b" >> $L
done
echo "$(date '+%F %H:%M') exp_bag_swslow: done" >> $L
