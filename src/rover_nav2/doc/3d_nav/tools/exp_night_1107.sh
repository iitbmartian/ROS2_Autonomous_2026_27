#!/bin/bash
# Night 7 to 8 Nov, after exp_2d_base: (1) far_south bag, (2) swerve v2 smoke then full run,
# (3) Ackermann speed candidate. Start deadline $1 (epoch), each step under guard.sh.
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log; B=$HOME/.ros/rover_benchmark
DL=$1
log() { echo "$(date '+%F %H:%M') exp_night: $*" >> $L; }
clean() { for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null; done
  python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1; }
n0=$(wc -l < $L)
until tail -n +$((n0+1)) $L | grep -q "exp_2d_base: done\|exp_2d_base: exited\|exp_2d_base: deadline"; do
  [ "$(date +%s)" -ge "$DL" ] && { log "deadline before exp_2d_base finished, nothing run"; exit 1; }
  sleep 30
done
source $E/tools/simenv.sh; cd /home/ram/ROS2_Autonomous_2026_27
log "start ($(git rev-parse --short HEAD))"
# 1. far_south diagnostic bag
r=$($E/tools/guard.sh 2700 $E/tools/bag_far_south.sh 2>&1 | tail -1); log "far_south bag: $r"
clean
# 2. swerve v2: smoke on one goal first, full run only if it reaches the goal
SW="--config 3d --launch-args odom_source:=ground_truth drive:=swerve governor_v_nominal:=0.5 governor_u_free:=0.0 nav_extra_overlay:=$E/exp/nav2_3d_swerve_v2.yaml"
$E/tools/guard.sh 1800 ros2 run rover_nav2 benchmark --worlds marsyard --goals ahead_8m --runs 1 --tag 3d_gt_swerve_v2_smoke \
  $SW --out $B/2026-11-07_3d_gt_swerve_v2_smoke >> $E/evidence/exp_night.out 2>&1 || log "swerve smoke exited $?"
clean
if grep -q ",succeeded,1," $B/2026-11-07_3d_gt_swerve_v2_smoke/runs.csv 2>/dev/null; then
  log "swerve v2 smoke ok, full run"
  $E/tools/guard.sh 12600 ros2 run rover_nav2 benchmark --worlds marsyard slopes rock_field urc_mix --runs 1 --tag 3d_gt_swerve_v2 \
    $SW --out $B/2026-11-07_3d_gt_swerve_v2 >> $E/evidence/exp_night.out 2>&1 || log "swerve v2 exited $?"
  clean
  log "swerve v2: $(cut -d, -f1,6,9,11 $B/2026-11-07_3d_gt_swerve_v2/runs.csv | tail -n +2 | tr '\n' ' ')"
else
  log "swerve v2 smoke failed, full run skipped: $(tail -n +2 $B/2026-11-07_3d_gt_swerve_v2_smoke/runs.csv 2>/dev/null | cut -d, -f6,8,9,20)"
fi
# 3. Ackermann speed: u_free 0.3 (5 to 20% faster on 2026-11-06, lost two final approaches) on the easy goals, 2 runs
$E/tools/guard.sh 9000 ros2 run rover_nav2 benchmark --worlds marsyard crater_field urc_mix --runs 2 --config 3d --tag 3d_gt_free03 \
  --launch-args odom_source:=ground_truth governor_v_nominal:=0.5 governor_u_free:=0.3 \
  --out $B/2026-11-07_3d_gt_free03 >> $E/evidence/exp_night.out 2>&1 || log "free03 exited $?"
clean
log "done: $(cut -d, -f1,6,9 $B/2026-11-07_3d_gt_free03/runs.csv | tail -n +2 | tr '\n' ' ')"
