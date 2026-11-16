#!/bin/bash
# Is the governor behind swerve v6's systematic timeouts (slopes lane25, sidehill, crater
# far_left, 0/5 each)? Near the tilt limits it caps speed at the 0.05 m/s floor, which could
# crawl past the 300 s limit. v6 with u_free 0.5 on those goals x3, then the Ackermann v1.1 drift
# repeat on slopes and bars at u_free 0.3 x3 (baseline for the governor decision).
# Args: deadline (epoch), pid to wait for.
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log; B=$HOME/.ros/rover_benchmark; X=$E/exp
log() { echo "$(date '+%F %H:%M') exp_sw6gov: $*" >> $L; }
clean() { for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null; done
  python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1; }
tally() { awk -F, -v w=$2.sdf 'NR>1&&$3==w{print $4":"$6}' $1/runs.csv 2>/dev/null | sort | uniq -c | tr -s ' \n' ' '; }
while kill -0 $2 2>/dev/null; do [ "$(date +%s)" -ge "$1" ] && { log "deadline while waiting"; exit 1; }; sleep 60; done
cd /home/ram/ROS2_Autonomous_2026_27; source /opt/ros/jazzy/setup.bash
source $E/tools/simenv.sh; unset ROVER_COUPLING_K ROVER_COUPLING_D ROVER_COUPLING_MAX
log "start ($(git rev-parse --short HEAD))"
G="odom_source:=ground_truth suspension:=plugin governor_v_nominal:=0.5"
t=3d_gt_sw6gov05_d250; O=$B/$(date +%F)_$t
A="--config 3d --launch-args $G governor_u_free:=0.5 drive:=swerve nav_extra_overlay:=$X/nav2_3d_swerve_v6.yaml"
for spec in "slopes --goals lane25_top sidehill_uphill" "crater_field --goals far_left"; do [ "$(date +%s)" -ge "$1" ] && break
  $E/tools/guard.sh 9000 ros2 run rover_nav2 benchmark --worlds $spec --runs 3 --tag $t $A --out $O >> $E/evidence/exp_sw6gov.out 2>&1 || log "$t ${spec%% *} exited $?"; clean
  log "$t ${spec%% *}: $(tally $O ${spec%% *})"
done
t=3d_gt_ck5_d250; O=$B/$(date +%F)_$t
A="--config 3d --launch-args $G governor_u_free:=0.3 global_fine_step:=true nav_extra_overlay:=$X/nav2_3d_reapproach.yaml"
for w in slopes bars; do [ "$(date +%s)" -ge "$1" ] && break
  $E/tools/guard.sh 9000 ros2 run rover_nav2 benchmark --worlds $w --runs 3 --tag $t $A --out $O >> $E/evidence/exp_sw6gov.out 2>&1 || log "$t $w exited $?"; clean
  log "$t $w: $(tally $O $w)"
done
log "done"
