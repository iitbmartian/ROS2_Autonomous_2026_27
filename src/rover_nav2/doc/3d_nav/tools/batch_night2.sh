#!/bin/bash
# 2026-11-05 night: after the rock fix check, 3D on all worlds (ground truth, escape fix 983e315),
# then 3 runs per goal of 3D and 2d_bounded for the head-to-head. One runner process per world,
# because a long-lived runner lost discovery after about 12 runs on 2026-11-04.
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log; B=$HOME/.ros/rover_benchmark
PF=/home/ram/ROS2_Autonomous_2026_27/install/rover_nav2/share/rover_nav2/config/nav2_params_sim_2d_bounded.yaml
W="marsyard crater_field slopes urc_mix rock_field ledge bars"
until grep -q "rock fix check done" $L; do sleep 60; done
source $E/tools/simenv.sh; cd /home/ram/ROS2_Autonomous_2026_27
run() {  # config tag out runs world [extra]
  for i in 1 2; do  # a second pass picks up runs lost to startup failures
    python3 - $B/$3/runs.csv <<'PY' 2>/dev/null
import csv, os, sys
p = sys.argv[1]
if os.path.exists(p):
    rows = list(csv.DictReader(open(p))); keep = [r for r in rows if r['status'] != 'startup_failed']
    if len(keep) != len(rows):
        w = csv.DictWriter(open(p, 'w', newline=''), fieldnames=rows[0].keys()); w.writeheader(); w.writerows(keep)
PY
    ros2 run rover_nav2 benchmark --worlds $5 --runs $4 --config $1 --tag $2 \
      --launch-args odom_source:=ground_truth --out $B/$3 "${@:6}" >> $E/evidence/night2.out 2>&1
  done
}
echo "$(date '+%F %H:%M') night2: 3D all worlds, commit $(git rev-parse --short HEAD)" >> $L
for w in $W; do run 3d 3d_gt_v4 2026-11-05_3d_gt_v4 1 $w; done
python3 $E/tools/fig_benchmark.py check_3d_gt_v4 $B/2026-11-05_3d_gt_v4 3d_gt_v4=3d >> $L 2>&1
echo "$(date '+%F %H:%M') night2: 3D all worlds done" >> $L
for w in $W; do
  run 3d 3d_gt_h2h 2026-11-05_h2h_3d 3 $w
  run 2d 2d_bounded_gt_h2h 2026-11-05_h2h_2db 3 $w --params-file $PF
done
python3 $E/tools/fig_benchmark.py h2h_gt $B/2026-11-05_h2h_2db $B/2026-11-05_h2h_3d \
  2d_bounded_gt_h2h=swerve 3d_gt_h2h=3d >> $L 2>&1
echo "$(date '+%F %H:%M') night2 done" >> $L
