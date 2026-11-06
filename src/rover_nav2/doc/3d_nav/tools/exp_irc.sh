#!/bin/bash
# IRC mode: heading free (yaw tolerance 3.14, GoalAngleCritic off, goals at the bearing heading),
# Ackermann, fast governor (v_nominal 0.5, u_free 0.3), 1 run per goal on marsyard, crater_field,
# slopes, urc_mix. Compared with exp_vnom B, which differs only in the goal heading handling.
# Starts after exp_swerve ends or gives up at $1.
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log; B=$HOME/.ros/rover_benchmark
log() { echo "$(date '+%F %H:%M') exp_irc: $*" >> $L; }
until grep -q "exp_swerve: done\|exp_swerve: exited\|exp_swerve: deadline" $L; do
  [ "$(date +%s)" -ge "$1" ] && { log "deadline before exp_swerve finished, nothing run"; exit 1; }
  sleep 60
done
source $E/tools/simenv.sh; cd /home/ram/ROS2_Autonomous_2026_27
log "start ($(git rev-parse --short HEAD))"
$E/tools/guard.sh 9000 ros2 run rover_nav2 benchmark --worlds marsyard crater_field slopes urc_mix --runs 1 \
  --config 3d --tag 3d_gt_irc --goal-yaw bearing --launch-args odom_source:=ground_truth \
  governor_v_nominal:=0.5 governor_u_free:=0.3 nav_extra_overlay:=$E/exp/nav2_3d_irc.yaml \
  --out $B/2026-11-07_3d_gt_irc >> $E/evidence/exp_irc.out 2>&1 || log "exited $?"
for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null; done
python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1  # kill -9 can leave broken SHM ports
log "done: $(cut -d, -f1,6,9,11 $B/2026-11-07_3d_gt_irc/runs.csv | tail -n +2 | tr '\n' ' ')"
