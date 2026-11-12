#!/bin/bash
# Bags of the goals every stack still fails, with the lethal-free 3D tree at the default D 250:
# urc_mix far_south (30, -20) and bars past_bars_4m (4, 0). Shows where the stall sits (planner,
# controller or command chain, tools/bag_timeline.py). Args: deadline (epoch), pid to wait for.
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log; X=$E/exp
log() { echo "$(date '+%F %H:%M') exp_bag_open: $*" >> $L; }
clean() { for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null; done
  python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1; }
while kill -0 $2 2>/dev/null; do [ "$(date +%s)" -ge "$1" ] && { log "deadline while waiting"; exit 1; }; sleep 60; done
source $E/tools/simenv.sh; cd /home/ram/ROS2_Autonomous_2026_27; unset ROVER_COUPLING_K ROVER_COUPLING_D ROVER_COUPLING_MAX
for g in "farsouth urc_mix.sdf 30.0 -20.0 -0.6" "bars4m bars.sdf 4.0 0.0 0.0"; do set -- $g $1; [ "$(date +%s)" -ge "$6" ] && { log "deadline before $1"; break; }
  BURN=8 $E/tools/guard.sh 1800 $E/tools/bag_run.sh open_$1 $2 $3 $4 $5 ackermann $X/nav2_3d_lethalfree.yaml; clean
  b=$(ls -td $E/bags/run_open_$1_* | grep -v '\.' | head -1)
  log "$1: $(tail -1 $b.goal.out 2>/dev/null), bag $b"
  set -- $6 0
done
log "done"
