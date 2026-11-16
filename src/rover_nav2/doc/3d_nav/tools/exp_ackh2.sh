#!/bin/bash
# Ackermann heading planner (ackh) on the other worlds, interleaved per world with a same-night
# v1.1 repeat (ck6) as its drift-checked baseline. First lane25_top x3 more for ackh (n 6).
# Builds rover_nav2 for --plan-planner-id (the timing plan used an empty id, invalid with two
# planners). Args: deadline (epoch), pid to wait for.
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log; B=$HOME/.ros/rover_benchmark; X=$E/exp
log() { echo "$(date '+%F %H:%M') exp_ackh2: $*" >> $L; }
clean() { for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null; done
  python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1; }
tally() { awk -F, -v w=$2.sdf 'NR>1&&$3==w{print $4":"$6}' $1/runs.csv 2>/dev/null | sort | uniq -c | tr -s ' \n' ' '; }
while kill -0 $2 2>/dev/null; do [ "$(date +%s)" -ge "$1" ] && { log "deadline while waiting"; exit 1; }; sleep 60; done
cd /home/ram/ROS2_Autonomous_2026_27; source /opt/ros/jazzy/setup.bash; source install/setup.bash
clean
log "build rover_nav2 ($(git rev-parse --short HEAD))"
nice colcon build --packages-select rover_nav2 > $E/evidence/exp_ackh2_build.log 2>&1 || { log "build failed"; exit 1; }
source install/setup.bash; source $E/tools/simenv.sh; unset ROVER_COUPLING_K ROVER_COUPLING_D ROVER_COUPLING_MAX
log "start"
V="--config 3d --launch-args odom_source:=ground_truth suspension:=plugin governor_v_nominal:=0.5 governor_u_free:=0.3 global_fine_step:=true"
th=3d_gt_ackh_d250; Oh=$B/$(date +%F)_${th}2
tc=3d_gt_ck6_d250; Oc=$B/$(date +%F)_$tc
$E/tools/guard.sh 3600 ros2 run rover_nav2 benchmark --worlds slopes --goals lane25_top --runs 3 --tag $th --plan-planner-id Heading $V nav_extra_overlay:=$X/nav2_3d_ack_heading.yaml --out $Oh >> $E/evidence/exp_ackh2.out 2>&1 || log "$th lane25 exited $?"; clean
log "$th lane25: $(tally $Oh slopes)"
for w in rock_field urc_mix bars; do
  [ "$(date +%s)" -ge "$1" ] && { log "deadline before $w"; break; }
  $E/tools/guard.sh 9000 ros2 run rover_nav2 benchmark --worlds $w --runs 3 --tag $th --plan-planner-id Heading $V nav_extra_overlay:=$X/nav2_3d_ack_heading.yaml --out $Oh >> $E/evidence/exp_ackh2.out 2>&1 || log "$th $w exited $?"; clean
  log "$th $w: $(tally $Oh $w)"
  [ "$(date +%s)" -ge "$1" ] && { log "deadline before ck6 $w"; break; }
  $E/tools/guard.sh 9000 ros2 run rover_nav2 benchmark --worlds $w --runs 3 --tag $tc $V nav_extra_overlay:=$X/nav2_3d_reapproach.yaml --out $Oc >> $E/evidence/exp_ackh2.out 2>&1 || log "$tc $w exited $?"; clean
  log "$tc $w: $(tally $Oc $w)"
done
log "done"
