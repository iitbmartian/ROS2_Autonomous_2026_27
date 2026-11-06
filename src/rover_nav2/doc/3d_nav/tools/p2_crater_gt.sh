#!/bin/bash
# Phase 2 gate on crater_field with ground-truth odometry (the 2026-11-03 run failed on EKF yaw
# drift, not mapping), plus the crater check. Runs after exp_vnom ends or gives up at $1 (epoch).
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log; P=$E/evidence/phase2
log() { echo "$(date '+%F %H:%M') p2_crater_gt: $*" >> $L; }
until grep -q "exp_vnom: done\|exp_vnom: exited\|exp_vnom: deadline" $L; do
  [ "$(date +%s)" -ge "$1" ] && { log "deadline before exp_vnom finished, nothing run"; exit 1; }
  sleep 60
done
mkdir -p $P/2026-11-03_ekf; for f in $P/*crater_field*; do [ -f "$f" ] && mv "$f" $P/2026-11-03_ekf/; done
log "start"
P2_EXTRA=odom_source:=ground_truth $E/tools/guard.sh 5400 $E/tools/phase2_eval.sh crater_field ahead_10m far_left far_right \
  > $E/evidence/p2_crater_gt.out 2>&1 || log "exited $?"
$E/tools/sim_down.sh p2_crater_field > /dev/null 2>&1
log "done: $(tail -3 $P/eval_crater_field.txt 2>/dev/null | tr '\n' ' ')"
