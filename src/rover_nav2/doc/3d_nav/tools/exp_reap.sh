#!/bin/bash
# Targeted test: re-approach tree (NearGoalTooLong, 1.5 m back-up) with global_fine_step at D 250.
# marsyard behind_left arrives 0.35 m beside the goal and saws for 900 s (bags run_fsbl2/3).
# marsyard all goals x3, regressions rock_field x2 and urc_mix far_south x3. Args: deadline, pid.
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log; B=$HOME/.ros/rover_benchmark; X=$E/exp
log() { echo "$(date '+%F %H:%M') exp_reap: $*" >> $L; }
clean() { for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null; done
  python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1; }
tally() { awk -F, -v w=$2.sdf 'NR>1&&$3==w{print $4":"$6}' $1/runs.csv 2>/dev/null | sort | uniq -c | tr -s ' \n' ' '; }
DL=$1
while kill -0 $2 2>/dev/null; do [ "$(date +%s)" -ge "$1" ] && { log "deadline while waiting"; exit 1; }; sleep 60; done
cd /home/ram/ROS2_Autonomous_2026_27; source /opt/ros/jazzy/setup.bash
source $E/tools/simenv.sh; unset ROVER_COUPLING_K ROVER_COUPLING_D ROVER_COUPLING_MAX
log "start ($(git rev-parse --short HEAD))"
G3="odom_source:=ground_truth suspension:=plugin governor_v_nominal:=0.5 governor_u_free:=0.3"
t=3d_gt_reap_fs_d250; O=$B/2026-11-14_$t
A="--config 3d --launch-args $G3 global_fine_step:=true nav_extra_overlay:=$X/nav2_3d_reapproach.yaml"
for spec in "marsyard 3" "rock_field 2" "urc_mix 3 far_south"; do set -- $spec; w=$1; n=$2; g=${3:+--goals $3}
  [ "$(date +%s)" -ge "$DL" ] 2>/dev/null && break
  $E/tools/guard.sh 9000 ros2 run rover_nav2 benchmark --worlds $w $g --runs $n --tag $t $A --out $O >> $E/evidence/exp_reap.out 2>&1 || log "$w exited $?"; clean
  log "$w: $(tally $O $w)"
done
log "done"
