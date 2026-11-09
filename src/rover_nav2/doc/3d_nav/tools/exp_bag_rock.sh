#!/bin/bash
# After exp_critic_mt: two recorded plain 3D runs of rock_field ahead_10m under benchmark load.
# In 2026-11-10_3d_gt_gate_final the plain stack froze 0.7 to 2.8 m from the goals beside 0.3 m rocks
# (wheels 0.1 to 0.2 m clear) and BackUp, DriveOnHeading did not move it. The bag shows whether
# commands reached the wheels. Args: deadline (epoch), pid to wait for.
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log
log() { echo "$(date '+%F %H:%M') exp_bag_rock: $*" >> $L; }
while kill -0 $2 2>/dev/null; do [ "$(date +%s)" -ge "$1" ] && { log "deadline while waiting"; exit 1; }; sleep 60; done
for i in 1 2; do [ "$(date +%s)" -ge "$1" ] && break
  BURN=8 $E/tools/guard.sh 1800 $E/tools/bag_run.sh rock_ahead10_plain rock_field 10.0 0.0 0.0 ackermann
  log "bag $i: $(tail -1 $(ls -t $E/bags/run_rock_ahead10_plain*.goal.out | head -1))"; done
log "done"
