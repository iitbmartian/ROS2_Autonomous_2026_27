#!/bin/bash
# Why both stacks slowed on the final rocker coupling (2D 2 to 4x, 3D 15 to 50 percent, body pitch
# rate p95 4 to 45 deg/s on flat marsyard, PROGRESS 2026-11-10 13:05). marsyard ahead_8m, 2D and
# plain 3D, over coupling gains set by env (rover_gazebo.xacro optenv), 2 runs each:
#   old K 8000 D 200 MAX 400, final K 30000 D 500 MAX 3000, and one gain changed at a time.
# Args: deadline (epoch), pid to wait for.
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log; B=$HOME/.ros/rover_benchmark
log() { echo "$(date '+%F %H:%M') exp_coupling_diag: $*" >> $L; }
clean() { for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null; done
  python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1; }
while kill -0 $2 2>/dev/null; do [ "$(date +%s)" -ge "$1" ] && { log "deadline while waiting"; exit 1; }; sleep 60; done
source $E/tools/simenv.sh; cd /home/ram/ROS2_Autonomous_2026_27
log "start"
O=$B/2026-11-10_coupling_diag
for cfg in "old 8000 200 400" "final 30000 500 3000" "k8k_max3k 8000 200 3000" "k30k_d200 30000 200 3000" "k30k_d1500 30000 1500 3000" "k15k_d500 15000 500 3000"; do
  set -- $cfg $1; n=$1; [ "$(date +%s)" -ge "$5" ] && { log "deadline before $n"; break; }
  for st in 2d 3d; do
    [ $st = 2d ] && P="--config 2d --launch-args odom_source:=ground_truth suspension:=plugin" \
      || P="--config 3d --launch-args odom_source:=ground_truth suspension:=plugin governor_v_nominal:=0.5 governor_u_free:=0.3"
    ROVER_COUPLING_K=$2 ROVER_COUPLING_D=$3 ROVER_COUPLING_MAX=$4 $E/tools/guard.sh 1800 ros2 run rover_nav2 benchmark \
      --worlds marsyard --goals ahead_8m --runs 2 --tag cpl_${n}_$st $P --out $O >> $E/evidence/exp_coupling_diag.out 2>&1 || log "$n $st exited $?"
    clean
  done
  set -- $5 0
done
python3 $E/tools/coupling_diag.py $O > $HOME/rover_3d_nav_results/analysis/2026-11-10_coupling_diag.md 2>&1
log "done: $(tail -n +3 $HOME/rover_3d_nav_results/analysis/2026-11-10_coupling_diag.md | tr -s ' \n' ' ' | cut -c1-400)"
