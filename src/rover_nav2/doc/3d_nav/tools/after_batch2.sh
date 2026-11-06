#!/bin/bash
# After the Phase 2 reruns: the DDS discovery trial.
L=$HOME/rover_3d_nav_results/batch.log
until grep -q "after_batch done" $L; do sleep 60; done
echo "$(date '+%F %H:%M') dds trial" >> $L
bash $HOME/.ros/rover_3d_nav/tools/dds_trial.sh 4 > $HOME/.ros/rover_3d_nav/evidence/dds_trial.out 2>&1
echo "$(date '+%F %H:%M') dds trial done" >> $L
