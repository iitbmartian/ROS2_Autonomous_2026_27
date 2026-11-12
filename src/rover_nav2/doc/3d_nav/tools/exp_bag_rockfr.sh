#!/bin/bash
# Bag of rock_field far_right (16.5, -4), which stalls ("Failed to make progress") about 2.4 m out in 3D,
# 14.1 m short every time, with the lethal-free 3D tree at D 250. Args: deadline (epoch), pid to wait for.
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log; X=$E/exp
log() { echo "$(date '+%F %H:%M') exp_bag_rockfr: $*" >> $L; }
clean() { for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null; done
  python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1; }
while kill -0 $2 2>/dev/null; do [ "$(date +%s)" -ge "$1" ] && { log "deadline while waiting"; exit 1; }; sleep 60; done
source $E/tools/simenv.sh; cd /home/ram/ROS2_Autonomous_2026_27; unset ROVER_COUPLING_K ROVER_COUPLING_D ROVER_COUPLING_MAX
for g in "rockfr rock_field.sdf 16.5 -4.0 0.0"; do set -- $g $1; [ "$(date +%s)" -ge "$6" ] && { log "deadline before $1"; break; }
  BURN=8 $E/tools/guard.sh 1800 $E/tools/bag_run.sh open_$1 $2 $3 $4 $5 ackermann $X/nav2_3d_lethalfree.yaml; clean
  b=$(ls -td $E/bags/run_open_$1_* | grep -v '\.' | head -1)
  log "$1: $(tail -1 $b.goal.out 2>/dev/null), bag $b"
  set -- $6 0
done
log "done"
