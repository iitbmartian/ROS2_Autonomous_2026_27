#!/bin/bash
# Confirming Phase 3 gate run: 3D with the chosen governor (v_nominal 0.5, u_free 0, Ackermann),
# all 7 worlds, 3 runs per goal on ground truth, compared with the 2026-11-06 2d_bounded rows.
# Starts at once (2026-11-07 03:40, ahead of swerve so Phase 3 closes first). Then the gate table and a dated backup.
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log; B=$HOME/.ros/rover_benchmark
R=$HOME/rover_3d_nav_results; O=$B/2026-11-07_3d_gt_confirm
log() { echo "$(date '+%F %H:%M') exp_confirm: $*" >> $L; }
source $E/tools/simenv.sh; cd /home/ram/ROS2_Autonomous_2026_27
log "start ($(git rev-parse --short HEAD))"
$E/tools/guard.sh 21600 ros2 run rover_nav2 benchmark --worlds marsyard crater_field slopes urc_mix rock_field ledge bars --runs 3 --config 3d --tag 3d_gt_confirm \
  --launch-args odom_source:=ground_truth governor_v_nominal:=0.5 governor_u_free:=0.0 \
  --out $O >> $E/evidence/exp_confirm.out 2>&1 || log "exited $?"
for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null; done
python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1  # kill -9 can leave broken SHM ports
python3 $E/tools/phase3_gate.py $O/runs.csv $B/2026-11-06_h2h_2db/runs.csv $R/analysis/phase3_gate_confirm.csv \
  > $E/evidence/phase3_gate_confirm.txt 2>&1
log "done: $(cut -d, -f6 $O/runs.csv | tail -n +2 | sort | uniq -c | tr '\n' ' ')"
# Dated backup, everything lives on one laptop SSD.
tar -C $HOME -I 'zstd -T0 -3' -chf $HOME/rover_3d_nav_results_backup_2026-11-07.tar.zst rover_3d_nav_results \
  && log "backup written" || log "backup failed"
