#!/bin/bash
# MPPI tracking candidate (exp/nav2_3d_mppi_track.yaml) on slopes, marsyard, rock_field, bars, ledge, 1 run each.
# Starts after "backfill: done" or at the deadline ($1, epoch) gives up.
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log; B=$HOME/.ros/rover_benchmark
log() { echo "$(date '+%F %H:%M') exp_mppi_track: $*" >> $L; }
until grep -q "backfill: done\|backfill: deadline" $L; do
  [ "$(date +%s)" -ge "$1" ] && { log "deadline before backfill finished, nothing run"; exit 1; }
  sleep 60
done
source $E/tools/simenv.sh; cd /home/ram/ROS2_Autonomous_2026_27
log "start"
$E/tools/guard.sh 14400 ros2 run rover_nav2 benchmark --worlds slopes marsyard rock_field bars ledge --runs 1 --config 3d \
  --tag 3d_gt_mppitrack --launch-args odom_source:=ground_truth nav_overlay:=$E/exp/nav2_3d_mppi_track.yaml \
  --out $B/2026-11-06_3d_gt_mppitrack >> $E/evidence/exp_mppi_track.out 2>&1 || log "exited $?"
for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null; done
python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1  # kill -9 can leave broken SHM ports
log "done: $(cut -d, -f1,6,9 $B/2026-11-06_3d_gt_mppitrack/runs.csv | tail -n +2 | tr '\n' ' ')"
