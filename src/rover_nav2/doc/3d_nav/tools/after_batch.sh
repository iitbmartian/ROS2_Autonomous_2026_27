#!/bin/bash
# Runs once batch_overnight.sh has exited: redo the Phase 2 evaluations lost to tooling bugs.
while pgrep -f batch_overnight.sh > /dev/null; do sleep 60; done
L=$HOME/rover_3d_nav_results/batch.log
E=$HOME/.ros/rover_3d_nav
echo "$(date '+%F %H:%M') rerun phase 2 slopes, rock_field" >> $L
for W in slopes rock_field; do
  echo "=== $W $(date +%H:%M)" >> $E/evidence/phase2/all.out
  $E/tools/phase2_eval.sh $W >> $E/evidence/phase2/all.out 2>&1
done
echo "$(date '+%F %H:%M') after_batch done" >> $L
