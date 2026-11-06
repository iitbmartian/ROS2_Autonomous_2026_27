#!/bin/bash
# After night4: rerun head-to-head rows whose stack never activated a drive controller (scored
# before de6abaa made that a startup failure), and rows that failed to start (stale SHM port
# mutexes, 2026-11-06 11:51 to 13:55). Waits for "night4: done" or the deadline ($1, epoch).
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log; B=$HOME/.ros/rover_benchmark
PF=/home/ram/ROS2_Autonomous_2026_27/install/rover_nav2/share/rover_nav2/config/nav2_params_sim_2d_bounded.yaml
log() { echo "$(date '+%F %H:%M') backfill: $*" >> $L; }
until grep -q "night4: done" $L; do
  [ "$(date +%s)" -ge "$1" ] && { log "deadline before night4 finished, nothing done"; exit 1; }
  sleep 60
done
source $E/tools/simenv.sh; cd /home/ram/ROS2_Autonomous_2026_27
for spec in "3d 3d_gt_h2h 2026-11-06_h2h_3d" "2d 2d_bounded_gt_h2h 2026-11-06_h2h_2db --params-file $PF"; do
  set -- $spec; cfg=$1 tag=$2 out=$B/$3; shift 3
  worlds=$(python3 - $out <<'PY'
import csv, os, sys
out = sys.argv[1]; p = os.path.join(out, 'runs.csv')
if not os.path.exists(p): sys.exit()
rows = list(csv.DictReader(open(p))); keep, worlds = [], set()
for r in rows:
    try: text = open(os.path.join(out, 'logs', r['run_id'] + '.log'), errors='replace').read()
    except OSError: text = ''
    if r['status'] == 'startup_failed' or 'Configured and activated wheel_controller' not in text:
        worlds.add(r['world'].replace('.sdf', ''))
    else: keep.append(r)
# Worlds short of goals x 3 rows (a dropped startup_failed row that pass 2 never scored).
import yaml, collections
n = collections.Counter(r['world'].replace('.sdf', '') for r in keep)
gdir = '/home/ram/ROS2_Autonomous_2026_27/install/rover_nav2/share/rover_nav2/benchmark/goals'
for wname in n:
    if n[wname] < 3 * len(yaml.safe_load(open(f'{gdir}/{wname}.yaml'))['goals']):
        worlds.add(wname)
if len(keep) != len(rows):
    w = csv.DictWriter(open(p, 'w', newline=''), fieldnames=rows[0].keys()); w.writeheader(); w.writerows(keep)
print(' '.join(sorted(worlds)))
PY
)
  [ -z "$worlds" ] && continue
  log "$tag rerunning rows without wheels or start in: $worlds"
  $E/tools/guard.sh 10800 ros2 run rover_nav2 benchmark --worlds $worlds --runs 3 --config $cfg --tag $tag \
    --launch-args odom_source:=ground_truth --out $out "$@" >> $E/evidence/backfill.out 2>&1 || log "$tag exited $?"
  for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null; done
  python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1  # kill -9 can leave port mutexes locked
done
python3 $E/tools/fig_benchmark.py h2h_gt $B/2026-11-06_h2h_2db $B/2026-11-06_h2h_3d \
  2d_bounded_gt_h2h=swerve 3d_gt_h2h=3d >> $L 2>&1
log "done"
