#!/bin/bash
# urc_mix far_south only, 6 runs with global_fine_step and 6 plain, both at D 250. far_south is the
# goal left in the gate (recoveries and ttg), and 3 runs per side are too few to tell a fix from
# noise. Args: deadline (epoch), pid to wait for.
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log; B=$HOME/.ros/rover_benchmark
log() { echo "$(date '+%F %H:%M') exp_south: $*" >> $L; }
clean() { for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null; done
  python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1; }
tally() { awk -F, 'NR>1{print $6}' $1/runs.csv 2>/dev/null | sort | uniq -c | tr -s ' \n' ' '; }
while kill -0 $2 2>/dev/null; do [ "$(date +%s)" -ge "$1" ] && { log "deadline while waiting"; exit 1; }; sleep 60; done
cd /home/ram/ROS2_Autonomous_2026_27; source /opt/ros/jazzy/setup.bash
source $E/tools/simenv.sh; unset ROVER_COUPLING_K ROVER_COUPLING_D ROVER_COUPLING_MAX
log "start ($(git rev-parse --short HEAD))"
G3="odom_source:=ground_truth suspension:=plugin governor_v_nominal:=0.5 governor_u_free:=0.3"
for st in fs plain; do
  [ "$(date +%s)" -ge "$1" ] && { log "deadline before $st"; break; }
  if [ $st = fs ]; then t=3d_gt_south_fs_d250; X="global_fine_step:=true"; else t=3d_gt_south_plain_d250; X=""; fi
  O=$B/2026-11-14_$t
  $E/tools/guard.sh 12000 ros2 run rover_nav2 benchmark --worlds urc_mix --goals far_south --runs 6 --tag $t --config 3d --launch-args $G3 $X --out $O >> $E/evidence/exp_south.out 2>&1 || log "$t exited $?"; clean
  log "$t: $(tally $O)"
done
log "done"
