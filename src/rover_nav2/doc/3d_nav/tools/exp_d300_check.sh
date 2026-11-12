#!/bin/bash
# D 300 passed the loaded swerve spin bar (4.7 deg, D 200 was 6.6). Check it does not shake the body:
# same marsyard ahead_8m pitch-jitter test as the 2026-11-10 sweep, D 300 and D 250.
# Args: deadline (epoch), pid to wait for.
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log; B=$HOME/.ros/rover_benchmark
log() { echo "$(date '+%F %H:%M') exp_d300_check: $*" >> $L; }
clean() { for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null; done
  python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1; }
while kill -0 $2 2>/dev/null; do [ "$(date +%s)" -ge "$1" ] && { log "deadline while waiting"; exit 1; }; sleep 60; done
source $E/tools/simenv.sh; cd /home/ram/ROS2_Autonomous_2026_27
log "start"
O=$B/2026-11-12_coupling_d300
for cfg in "k30k_d300 30000 300 3000" "k30k_d250 30000 250 3000"; do
  set -- $cfg $1; n=$1; [ "$(date +%s)" -ge "$5" ] && { log "deadline before $n"; break; }
  for st in 2d 3d; do
    [ $st = 2d ] && P="--config 2d --launch-args odom_source:=ground_truth suspension:=plugin" \
      || P="--config 3d --launch-args odom_source:=ground_truth suspension:=plugin governor_v_nominal:=0.5 governor_u_free:=0.3"
    ROVER_COUPLING_K=$2 ROVER_COUPLING_D=$3 ROVER_COUPLING_MAX=$4 $E/tools/guard.sh 1800 ros2 run rover_nav2 benchmark \
      --worlds marsyard --goals ahead_8m --runs 2 --tag cpl_${n}_$st $P --out $O >> $E/evidence/exp_d300_check.out 2>&1 || log "$n $st exited $?"
    clean
  done
  set -- $5 0
done
python3 $E/tools/coupling_diag.py $O > $HOME/rover_3d_nav_results/analysis/2026-11-12_coupling_d300.md 2>&1
log "done: $(tail -n +3 $HOME/rover_3d_nav_results/analysis/2026-11-12_coupling_d300.md | tr -s ' \n' ' ' | cut -c1-400)"
