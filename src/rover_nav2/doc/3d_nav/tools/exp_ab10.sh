#!/bin/bash
# Same-day A/B on rock_field ahead_10m, 6 runs each, D 250, fine step on in both: A the v1.1 tree
# (re-approach), B the tree without it (nav2_3d_plaintree overlay). Today A is 10/15 and the
# tree-free runs 3/3 each time. Args: deadline (epoch), pid to wait for.
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log; B=$HOME/.ros/rover_benchmark; X=$E/exp
log() { echo "$(date '+%F %H:%M') exp_ab10: $*" >> $L; }
clean() { for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null; done
  python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1; }
DL=$1; while kill -0 $2 2>/dev/null; do sleep 30; done
cd /home/ram/ROS2_Autonomous_2026_27; source /opt/ros/jazzy/setup.bash
source $E/tools/simenv.sh; unset ROVER_COUPLING_K ROVER_COUPLING_D ROVER_COUPLING_MAX
log "start ($(git rev-parse --short HEAD))"
G3="odom_source:=ground_truth suspension:=plugin governor_v_nominal:=0.5 governor_u_free:=0.3"
for v in A B; do [ "$(date +%s)" -ge "$DL" ] && { log deadline; break; }
  if [ $v = A ]; then t=3d_gt_ab10_reap_d250; X2=""; else t=3d_gt_ab10_plaintree_d250; X2="nav_extra_overlay:=$X/nav2_3d_plaintree.yaml"; fi
  O=$B/2026-11-14_$t
  $E/tools/guard.sh 9000 ros2 run rover_nav2 benchmark --worlds rock_field --goals ahead_10m --runs 6 --tag $t --config 3d --launch-args $G3 $X2 --out $O >> $E/evidence/exp_ab10.out 2>&1 || log "$v exited $?"; clean
  log "$v $t: $(awk -F, 'NR>1{print $6}' $O/runs.csv | sort | uniq -c | tr -s ' \n' ' ')"
done
log "done"
