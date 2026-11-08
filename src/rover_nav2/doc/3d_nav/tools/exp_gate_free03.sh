#!/bin/bash
# Phase 3 gate run: 3D Ackermann with governor u_free 0.3 (v_nominal 0.5), all 7 worlds, 3 runs,
# then the gate against the fresh 2D baseline (2026-11-07_2d_base). Starts now, guard 9 h.
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log; B=$HOME/.ros/rover_benchmark; R=$HOME/rover_3d_nav_results
O=$B/2026-11-08_3d_gt_gate_free03
log() { echo "$(date '+%F %H:%M') exp_gate_free03: $*" >> $L; }
source $E/tools/simenv.sh; cd /home/ram/ROS2_Autonomous_2026_27
log "start ($(git rev-parse --short HEAD))"
$E/tools/guard.sh 32400 ros2 run rover_nav2 benchmark --worlds marsyard crater_field slopes urc_mix rock_field ledge bars --runs 3 \
  --config 3d --tag 3d_gt_gate_free03 --launch-args odom_source:=ground_truth governor_v_nominal:=0.5 governor_u_free:=0.3 \
  --out $O >> $E/evidence/exp_gate_free03.out 2>&1 || log "exited $?"
for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null; done
python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1
python3 $E/tools/phase3_gate.py $O/runs.csv $B/2026-11-07_2d_base/runs.csv $R/analysis/phase3_gate_free03_vs_2d_base.csv > $E/evidence/phase3_gate_free03.txt 2>&1
log "done: $(cut -d, -f6 $O/runs.csv | tail -n +2 | sort | uniq -c | tr '\n' ' '), gate in evidence/phase3_gate_free03.txt"
