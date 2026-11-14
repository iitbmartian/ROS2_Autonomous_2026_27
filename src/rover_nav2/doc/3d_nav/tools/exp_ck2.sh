#!/bin/bash
# Targeted after the v1.1 checkpoint (fine step + re-approach, D 250): rock_field ahead_10m x6 (1/3
# in the checkpoint, 3/3 with the fine step alone, is it the tree or noise) and marsyard
# behind_left x3 (3/3 but 60 s against 2D 25 s, ttg 1.73). Args: deadline (epoch), pid to wait for.
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log; B=$HOME/.ros/rover_benchmark; X=$E/exp
log() { echo "$(date '+%F %H:%M') exp_ck2: $*" >> $L; }
clean() { for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null; done
  python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1; }
DL=$1; while kill -0 $2 2>/dev/null; do sleep 30; done
cd /home/ram/ROS2_Autonomous_2026_27; source /opt/ros/jazzy/setup.bash
source $E/tools/simenv.sh; unset ROVER_COUPLING_K ROVER_COUPLING_D ROVER_COUPLING_MAX
log "start ($(git rev-parse --short HEAD))"
G3="odom_source:=ground_truth suspension:=plugin governor_v_nominal:=0.5 governor_u_free:=0.3"
t=3d_gt_ck2_d250; O=$B/2026-11-14_$t
A="--config 3d --launch-args $G3 global_fine_step:=true nav_extra_overlay:=$X/nav2_3d_reapproach.yaml"
for spec in "rock_field ahead_10m 6" "marsyard behind_left 3"; do set -- $spec
  [ "$(date +%s)" -ge "$DL" ] && { log "deadline"; break; }
  $E/tools/guard.sh 9000 ros2 run rover_nav2 benchmark --worlds $1 --goals $2 --runs $3 --tag $t $A --out $O >> $E/evidence/exp_ck2.out 2>&1 || log "$1 exited $?"; clean
  log "$1 $2: $(awk -F, -v g=$2 'NR>1&&$4==g{print $6}' $O/runs.csv | sort | uniq -c | tr -s ' \n' ' ')"
done
log "done"
