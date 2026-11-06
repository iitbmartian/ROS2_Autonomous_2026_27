#!/bin/bash
# After batch_gt.sh: the 3D runner lost discovery after 12 runs (Nav2 active, runner blind), so
# startup_failed runs are dropped and rerun with one runner process per world.
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log; B=$HOME/.ros/rover_benchmark
PF=/home/ram/ROS2_Autonomous_2026_27/install/rover_nav2/share/rover_nav2/config/nav2_params_sim_2d_bounded.yaml
W="slopes crater_field rock_field urc_mix marsyard ledge bars"
until grep -q "gt batch done" $L; do sleep 60; done
source $E/tools/simenv.sh
cd /home/ram/ROS2_Autonomous_2026_27
echo "$(date '+%F %H:%M') gt rerun of startup failures, one runner per world" >> $L
for d in 2026-11-04_3d_gt 2026-11-04_2d_bounded_gt; do
  python3 - $B/$d/runs.csv <<'PY'
import csv, sys
p = sys.argv[1]; rows = list(csv.DictReader(open(p))); f = rows[0].keys()
keep = [r for r in rows if r['status'] != 'startup_failed']
w = csv.DictWriter(open(p, 'w', newline=''), fieldnames=f); w.writeheader(); w.writerows(keep)
print(p, 'dropped', len(rows) - len(keep))
PY
done >> $L
for w in $W; do
  ros2 run rover_nav2 benchmark --worlds $w --runs 1 --config 3d --tag 3d_gt \
    --launch-args odom_source:=ground_truth --out $B/2026-11-04_3d_gt >> $E/evidence/gt.out 2>&1
  ros2 run rover_nav2 benchmark --worlds $w --runs 1 --config 2d --tag 2d_bounded_gt --params-file $PF \
    --launch-args odom_source:=ground_truth --out $B/2026-11-04_2d_bounded_gt >> $E/evidence/gt.out 2>&1
done
python3 $E/tools/fig_benchmark.py check_3d_gt $B/2026-11-04_3d_gt 3d_gt=3d >> $L 2>&1
python3 $E/tools/fig_benchmark.py compare_gt $B/2026-11-04_2d_bounded_gt $B/2026-11-04_3d_gt \
  2d_bounded_gt=swerve 3d_gt=3d >> $L 2>&1
echo "$(date '+%F %H:%M') gt fix done" >> $L
