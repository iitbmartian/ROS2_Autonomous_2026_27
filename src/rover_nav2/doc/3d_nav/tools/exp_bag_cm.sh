#!/bin/bash
# Bags with the global and local costmaps and hazard points of the two stalls where a wheel sits on
# a rock over or at step_max: rock_field far_right (rock_061, 0.12 m) and urc_mix far_south
# (rock_080, 0.107 m), default 3D tree at D 250. Shows whether the map rates the rock lethal.
# Args: deadline (epoch), pid to wait for.
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log
log() { echo "$(date '+%F %H:%M') exp_bag_cm: $*" >> $L; }
clean() { for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null; done
  python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1; }
while kill -0 $2 2>/dev/null; do [ "$(date +%s)" -ge "$1" ] && { log "deadline while waiting"; exit 1; }; sleep 60; done
source $E/tools/simenv.sh; cd /home/ram/ROS2_Autonomous_2026_27; unset ROVER_COUPLING_K ROVER_COUPLING_D ROVER_COUPLING_MAX
export EXTRA_TOPICS="/global_costmap/costmap /local_costmap/costmap /terrain/hazard_points"
DL=$1
for g in "rockfr rock_field.sdf 16.5 -4.0 0.0" "farsouth urc_mix.sdf 30.0 -20.0 -0.6"; do set -- $g
  [ "$(date +%s)" -ge "$DL" ] && { log "deadline before $1"; break; }
  BURN=8 $E/tools/guard.sh 1800 $E/tools/bag_run.sh cm_$1 $2 $3 $4 $5 ackermann; clean
  b=$(ls -td $E/bags/run_cm_$1_* | grep -v '[.][a-z]*$' | head -1)
  log "$1: $(tr '\n' ' ' < $b.goal.out 2>/dev/null), bag $b"
done
log "done"
