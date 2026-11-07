#!/bin/bash
# Candidate C (exp/nav2_3d_goal_approach_c.yaml) after exp_night: far_south x2, behind_left x2, left_diag x1, mid_20m x1.
# Start deadline $1, guard caps.
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log; B=$HOME/.ros/rover_benchmark; O=$B/2026-11-08_3d_gt_approach_c
DL=$1
log() { echo "$(date '+%F %H:%M') exp_approach_c: $*" >> $L; }
n0=$(wc -l < $L)
until tail -n +$((n0+1)) $L | grep -q "exp_night: done\|exp_night: deadline"; do
  [ "$(date +%s)" -ge "$DL" ] && { log "deadline before exp_night finished, nothing run"; exit 1; }
  sleep 30
done
source $E/tools/simenv.sh; cd /home/ram/ROS2_Autonomous_2026_27
log "start ($(git rev-parse --short HEAD))"
for spec in "urc_mix far_south 2" "marsyard behind_left 2" "marsyard left_diag 1" "urc_mix mid_20m 1"; do
  set -- $spec
  $E/tools/guard.sh 5400 ros2 run rover_nav2 benchmark --worlds $1 --goals $2 --runs $3 --config 3d --tag 3d_gt_approach_c \
    --launch-args odom_source:=ground_truth governor_v_nominal:=0.5 governor_u_free:=0.0 \
    nav_extra_overlay:=$E/exp/nav2_3d_goal_approach_c.yaml --out $O >> $E/evidence/exp_approach_c.out 2>&1 || log "$1 exited $?"
done
for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null; done
python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1
log "done: $(cut -d, -f1,6,8,9 $O/runs.csv | tail -n +2 | tr '\n' ' ')"
