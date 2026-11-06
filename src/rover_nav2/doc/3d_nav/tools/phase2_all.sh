#!/bin/bash
# All Phase 2 evaluation runs in sequence. Results in evidence/phase2/.
E=$HOME/.ros/rover_3d_nav
for W in slopes marsyard ledge bars rock_field urc_mix; do
  echo "=== $W $(date +%H:%M)"; $E/tools/phase2_eval.sh $W
done
echo "=== crater_field $(date +%H:%M)"; $E/tools/phase2_eval.sh crater_field ahead_10m far_left far_right
echo "=== done $(date +%H:%M)"
