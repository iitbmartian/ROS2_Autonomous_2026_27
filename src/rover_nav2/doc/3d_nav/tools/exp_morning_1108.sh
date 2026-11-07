#!/bin/bash
# 8 Nov early morning, after exp_night. Decides from the night's rows, no idle time before Ram's 09:00 look.
# A. swerve: v2 good (>= 9/14 and <= 1 tip cancel) -> 2 more runs on slopes, rock_field, urc_mix.
#    Otherwise -> swerve v3 (slower yaw) one run on all 4 worlds.
# B. Ackermann u_free 0.3 on slopes and rock_field x2, to complete the speed comparison.
# Start deadline $1, no new stage after $2, hard caps through guard.sh.
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log; B=$HOME/.ros/rover_benchmark
DL=$1; LAST=$2
log() { echo "$(date '+%F %H:%M') exp_morning: $*" >> $L; }
clean() { for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null; done
  python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1; }
left() { echo $(( LAST - $(date +%s) )); }
n0=$(wc -l < $L)
until tail -n +$((n0+1)) $L | grep -q "exp_approach_c: done\|exp_approach_c: deadline"; do
  [ "$(date +%s)" -ge "$DL" ] && { log "deadline before exp_approach_c finished, nothing run"; exit 1; }
  sleep 30
done
source $E/tools/simenv.sh; cd /home/ram/ROS2_Autonomous_2026_27
log "start ($(git rev-parse --short HEAD))"
V2=$B/2026-11-07_3d_gt_swerve_v2/runs.csv
ok=0; cn=99
[ -f $V2 ] && { ok=$(awk -F, 'NR>1 && $7==1' $V2 | wc -l); cn=$(awk -F, 'NR>1 && $6=="canceled"' $V2 | wc -l); }
SW="--config 3d --launch-args odom_source:=ground_truth drive:=swerve governor_v_nominal:=0.5 governor_u_free:=0.0"
if [ $(left) -gt 3600 ]; then
  if [ "$ok" -ge 9 ] && [ "$cn" -le 1 ]; then
    log "A: swerve v2 $ok/14, $cn tip cancels, adding runs 2 and 3 on slopes rock_field urc_mix"
    $E/tools/guard.sh $(left) ros2 run rover_nav2 benchmark --worlds slopes rock_field urc_mix --runs 3 --tag 3d_gt_swerve_v2 \
      $SW nav_extra_overlay:=$E/exp/nav2_3d_swerve_v2.yaml --out $B/2026-11-07_3d_gt_swerve_v2 >> $E/evidence/exp_morning.out 2>&1 || log "A exited $?"
    O=$B/2026-11-07_3d_gt_swerve_v2
  else
    log "A: swerve v2 $ok/14, $cn tip cancels, trying swerve v3"
    $E/tools/guard.sh $(left) ros2 run rover_nav2 benchmark --worlds marsyard slopes rock_field urc_mix --runs 1 --tag 3d_gt_swerve_v3 \
      $SW nav_extra_overlay:=$E/exp/nav2_3d_swerve_v3.yaml --out $B/2026-11-08_3d_gt_swerve_v3 >> $E/evidence/exp_morning.out 2>&1 || log "A exited $?"
    O=$B/2026-11-08_3d_gt_swerve_v3
  fi
  clean; log "A done: $(cut -d, -f1,6,9 $O/runs.csv 2>/dev/null | tail -n +2 | tr '\n' ' ')"
else log "A skipped, under 1 h left"; fi
if [ $(left) -gt 3600 ]; then
  $E/tools/guard.sh $(left) ros2 run rover_nav2 benchmark --worlds slopes rock_field --runs 2 --config 3d --tag 3d_gt_free03 \
    --launch-args odom_source:=ground_truth governor_v_nominal:=0.5 governor_u_free:=0.3 \
    --out $B/2026-11-07_3d_gt_free03 >> $E/evidence/exp_morning.out 2>&1 || log "B exited $?"
  clean
else log "B skipped, under 1 h left"; fi
log "done"
