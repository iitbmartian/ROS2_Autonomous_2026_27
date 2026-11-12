#!/bin/bash
# Plain 3D with IsPathLethalFree in place of IsPathValid (PROGRESS 2026-11-12), all 6 worlds x3 at
# D 200, a full Phase 3 gate attempt against 2026-11-11_2d_base_d200.
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log; B=$HOME/.ros/rover_benchmark; X=$E/exp
log() { echo "$(date '+%F %H:%M') exp_lethalfree: $*" >> $L; }
clean() { for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null; done
  python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1; }
tally() { awk -F, -v w=$2.sdf 'NR>1&&$3==w{print $4":"$6}' $1/runs.csv 2>/dev/null | sort | uniq -c | tr -s ' \n' ' '; }
while kill -0 $2 2>/dev/null; do [ "$(date +%s)" -ge "$1" ] && { log "deadline while waiting"; exit 1; }; sleep 60; done
source $E/tools/simenv.sh; cd /home/ram/ROS2_Autonomous_2026_27
export ROVER_COUPLING_K=30000 ROVER_COUPLING_D=200 ROVER_COUPLING_MAX=3000
G3="odom_source:=ground_truth suspension:=plugin governor_v_nominal:=0.5 governor_u_free:=0.3"
O=$B/2026-11-12_3d_gt_lethalfree_d200; log "start ($(git rev-parse --short HEAD))"
for w in marsyard rock_field urc_mix slopes crater_field bars; do [ "$(date +%s)" -ge "$1" ] && { log "deadline before $w"; break; }
  $E/tools/guard.sh 9000 ros2 run rover_nav2 benchmark --worlds $w --runs 3 --tag 3d_gt_lethalfree_d200 --config 3d \
    --launch-args $G3 nav_extra_overlay:=$X/nav2_3d_lethalfree.yaml --out $O >> $E/evidence/exp_lethalfree.out 2>&1 || log "$w exited $?"; clean
  log "$w: $(tally $O $w)"
done

python3 $E/tools/phase3_gate.py $O/runs.csv $B/2026-11-11_2d_base_d200/runs.csv > $E/evidence/phase3_gate_lethalfree_d200.txt 2>&1
log "gate: $(grep -c FAIL $E/evidence/phase3_gate_lethalfree_d200.txt) FAIL items"
log "done"
