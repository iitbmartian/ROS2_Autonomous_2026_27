#!/bin/bash
# After exp_tonight: 3D Ackermann gate run on the plugin suspension with the plain 3D tree (approach
# removed 2026-11-08 18:15), u_free 0.3, 6 worlds x3 (ledge waits on Ram's data). Attributes the
# suspension effect alone against 2026-11-08_3d_gt_gate_free03 (torque node, same tree).
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log; B=$HOME/.ros/rover_benchmark; R=$HOME/rover_3d_nav_results
log() { echo "$(date '+%F %H:%M') exp_gate_clean: $*" >> $L; }
n0=$(wc -l < $L)
until tail -n +$((n0+1)) $L | grep -q "exp_tonight: done\|exp_tonight: deadline"; do
  [ "$(date +%s)" -ge "$1" ] && { log "deadline before exp_tonight finished, nothing run"; exit 1; }; sleep 30; done
source $E/tools/simenv.sh; cd /home/ram/ROS2_Autonomous_2026_27
log "start ($(git rev-parse --short HEAD))"
G=$B/2026-11-09_3d_gt_gate_clean
$E/tools/guard.sh 28800 ros2 run rover_nav2 benchmark --worlds marsyard crater_field slopes urc_mix rock_field bars --runs 3 --config 3d \
  --tag 3d_gt_gate_clean --launch-args odom_source:=ground_truth suspension:=plugin governor_v_nominal:=0.5 governor_u_free:=0.3 \
  --out $G >> $E/evidence/exp_gate_clean.out 2>&1 || log "exited $?"
for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null; done
python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1
python3 $E/tools/phase3_gate.py $G/runs.csv $B/2026-11-09_2d_base_susp/runs.csv $R/analysis/phase3_gate_clean_vs_2d_susp.csv > $E/evidence/phase3_gate_clean.txt 2>&1
log "done: $(cut -d, -f6 $G/runs.csv | tail -n +2 | sort | uniq -c | tr '\n' ' ')"
