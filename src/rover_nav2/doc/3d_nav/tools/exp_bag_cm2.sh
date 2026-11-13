#!/bin/bash
# Re-run of the far_south costmap bag that guard.sh refused at 9 GB free (bag is about 0.3 GB, so
# the floor is lowered to 6 GB for this one run). Default 3D tree at D 250.
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log
source $E/tools/simenv.sh; cd /home/ram/ROS2_Autonomous_2026_27; unset ROVER_COUPLING_K ROVER_COUPLING_D ROVER_COUPLING_MAX
export EXTRA_TOPICS="/global_costmap/costmap /local_costmap/costmap /terrain/hazard_points"
GUARD_MIN_GB=6 BURN=8 $E/tools/guard.sh 1800 $E/tools/bag_run.sh cm_farsouth urc_mix.sdf 30.0 -20.0 -0.6 ackermann
for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null; done
python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1
b=$(ls -td $E/bags/run_cm_farsouth_* | grep -v '[.][a-z]*$' | head -1)
echo "$(date '+%F %H:%M') exp_bag_cm2: farsouth: $(tail -1 $b.goal.out 2>/dev/null), bag $b" >> $L
