#!/bin/bash
# Re-approach tree (exp/nav2_3d_reapproach.yaml) on marsyard and urc_mix x3, the two worlds where
# the rover saws beside the goal (behind_left, far_south), then rock_field x3 to check it does not
# hurt there (the 2026-11-08 approach variant did). Smoke first: one marsyard behind_left run must
# finish without a startup failure. Plugin suspension, u_free 0.3, ground truth.
# Arg 1: PID to wait for. Arg 2: deadline (epoch).
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log; B=$HOME/.ros/rover_benchmark
log() { echo "$(date '+%F %H:%M') exp_reapproach: $*" >> $L; }
clean() { for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null; done
  python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1; }
while kill -0 $1 2>/dev/null; do [ "$(date +%s)" -ge "$2" ] && { log "deadline while waiting"; exit 1; }; sleep 20; done
source $E/tools/simenv.sh; cd /home/ram/ROS2_Autonomous_2026_27
source install_dev/rover_traversability/share/rover_traversability/local_setup.bash   # BT node and critic
log "start ($(git rev-parse --short HEAD))"
O=$B/2026-11-09_3d_gt_reapproach
A="--config 3d --tag 3d_gt_reapproach --launch-args odom_source:=ground_truth suspension:=plugin governor_v_nominal:=0.5 governor_u_free:=0.3 nav_extra_overlay:=$E/exp/nav2_3d_reapproach.yaml"
$E/tools/guard.sh 1800 ros2 run rover_nav2 benchmark --worlds marsyard --goals behind_left --runs 1 $A --out ${O}_smoke >> $E/evidence/exp_reapproach.out 2>&1; clean
st=$(awk -F, 'NR>1{print $6}' ${O}_smoke/runs.csv 2>/dev/null)
grep -h "NearGoalTooLong\|Failed to load\|could not\|Error loading" ${O}_smoke/logs/*.log 2>/dev/null | head -3 | cut -c1-200 >> $E/evidence/exp_reapproach.out
case "$st" in succeeded|timeout|aborted|canceled) log "smoke $st, fired: $(grep -hc 'NearGoalTooLong:' ${O}_smoke/logs/*.log 2>/dev/null)";;
  *) log "smoke failed to start ($st), stop"; exit 1;; esac
for w in marsyard urc_mix rock_field; do
  [ "$(date +%s)" -ge "$2" ] && { log "deadline before $w"; break; }
  $E/tools/guard.sh 9000 ros2 run rover_nav2 benchmark --worlds $w --runs 3 $A --out $O >> $E/evidence/exp_reapproach.out 2>&1 || log "$w exited $?"; clean
  log "$w: $(awk -F, -v w=$w.sdf '$3==w{print $4":"$6}' $O/runs.csv | sort | uniq -c | tr -s ' \n' ' ')"
done
log "done"
