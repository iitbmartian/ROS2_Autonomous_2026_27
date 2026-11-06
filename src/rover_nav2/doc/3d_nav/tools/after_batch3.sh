#!/bin/bash
# After the DDS trial: rock_field Phase 2 rerun (the last one lost to an rclpy take error).
L=$HOME/rover_3d_nav_results/batch.log
E=$HOME/.ros/rover_3d_nav
until grep -q "dds trial done" $L; do sleep 60; done
echo "$(date '+%F %H:%M') rerun phase 2 rock_field" >> $L
echo "=== rock_field $(date +%H:%M)" >> $E/evidence/phase2/all.out
$E/tools/phase2_eval.sh rock_field >> $E/evidence/phase2/all.out 2>&1
echo "$(date '+%F %H:%M') after_batch3 done" >> $L
