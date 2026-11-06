#!/bin/bash
# 2026-11-06: after v5 bars, install the roughness fix (8b4c10e), check it on rock_field (v6), then
# the 3-run head-to-head. Every runner is capped by guard.sh and stray stacks are cleared after.
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log; B=$HOME/.ros/rover_benchmark
PF=/home/ram/ROS2_Autonomous_2026_27/install/rover_nav2/share/rover_nav2/config/nav2_params_sim_2d_bounded.yaml
W="marsyard crater_field slopes urc_mix rock_field ledge bars"
log() { echo "$(date '+%F %H:%M') night4: $*" >> $L; }
clear_stack() {  # any process launched by a benchmark run (ROVER_BENCH_RUN in its environment)
  for p in /proc/[0-9]*; do
    grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null
  done; sleep 5
}
grun() {  # config tag out runs world cap [extra], one retry pass for startup failures
  for i in 1 2; do
    python3 - $B/$3/runs.csv <<'PY' 2>/dev/null
import csv, os, sys
p = sys.argv[1]
if os.path.exists(p):
    rows = list(csv.DictReader(open(p))); keep = [r for r in rows if r['status'] != 'startup_failed']
    if len(keep) != len(rows):
        w = csv.DictWriter(open(p, 'w', newline=''), fieldnames=rows[0].keys()); w.writeheader(); w.writerows(keep)
PY
    $E/tools/guard.sh $6 ros2 run rover_nav2 benchmark --worlds $5 --runs $4 --config $1 --tag $2 \
      --launch-args odom_source:=ground_truth --out $B/$3 "${@:7}" >> $E/evidence/night4.out 2>&1 \
      || log "$2 $5 pass $i exited $?"
    clear_stack
  done
}
while pgrep -f "bag_terrain.sh|rover_nav2/benchmark --worlds" > /dev/null; do sleep 20; done
clear_stack
source $E/tools/simenv.sh; cd /home/ram/ROS2_Autonomous_2026_27
colcon build --packages-select rover_traversability rover_terrain_mapping > $E/evidence/night4_build.out 2>&1 \
  || { log "build failed, stopping"; exit 1; }
log "roughness fix installed ($(git rev-parse --short HEAD)), rock_field v6"
grun 3d 3d_gt_v6 2026-11-06_3d_gt_v6 1 rock_field 5400
log "v6 rock_field: $(grep -c ',1,' $B/2026-11-06_3d_gt_v6/runs.csv 2>/dev/null) lines with success, head-to-head"
for w in $W; do
  grun 3d 3d_gt_h2h 2026-11-06_h2h_3d 3 $w 14400
  grun 2d 2d_bounded_gt_h2h 2026-11-06_h2h_2db 3 $w 14400 --params-file $PF
  log "h2h $w done"
done
python3 $E/tools/fig_benchmark.py h2h_gt $B/2026-11-06_h2h_2db $B/2026-11-06_h2h_3d \
  2d_bounded_gt_h2h=swerve 3d_gt_h2h=3d >> $L 2>&1
log "done"
