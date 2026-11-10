#!/bin/bash
# What exp_critic_mt lost or ran out of time for: critic v2 threaded on slopes x3 (refused at 10:41,
# an orphaned swerve runner blocked it), then swerve v4 threaded if exp_critic_mt did not finish it.
# Args: deadline (epoch), pid to wait for.
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log; B=$HOME/.ros/rover_benchmark; X=$E/exp
log() { echo "$(date '+%F %H:%M') exp_critic_mt_rest: $*" >> $L; }
clean() { for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null; done
  python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1; }
tally() { awk -F, -v w=$2.sdf 'NR>1&&$3==w{print $4":"$6}' $1/runs.csv 2>/dev/null | sort | uniq -c | tr -s ' \n' ' '; }
while kill -0 $2 2>/dev/null; do [ "$(date +%s)" -ge "$1" ] && { log "deadline while waiting"; exit 1; }; sleep 60; done
source $E/tools/simenv.sh; cd /home/ram/ROS2_Autonomous_2026_27
G3="odom_source:=ground_truth suspension:=plugin governor_v_nominal:=0.5 governor_u_free:=0.3"
O=$B/2026-11-10_3d_gt_ack_critic_v2_mt
if [ "$(awk -F, 'NR>1&&$3=="slopes.sdf"' $O/runs.csv 2>/dev/null | wc -l)" -lt 15 ]; then
  $E/tools/guard.sh 9000 ros2 run rover_nav2 benchmark --worlds slopes --runs 3 --config 3d --tag 3d_gt_ack_critic_v2_mt --launch-args $G3 nav_extra_overlay:=$X/nav2_3d_ack_terrain_critic_v2.yaml --out $O >> $E/evidence/exp_critic_mt_rest.out 2>&1 || log "slopes exited $?"; clean
  log "critic v2 mt slopes: $(tally $O slopes)"; fi
O=$B/2026-11-10_3d_gt_swerve_v4_mt
for x in marsyard slopes; do [ "$(date +%s)" -ge "$1" ] && { log "deadline before swerve $x"; break; }
  have=$(awk -F, -v w=$x.sdf 'NR>1&&$3==w' $O/runs.csv 2>/dev/null | wc -l); [ "$have" -ge 6 ] && continue
  $E/tools/guard.sh 9000 ros2 run rover_nav2 benchmark --worlds $x --runs 2 --config 3d --tag 3d_gt_swerve_v4_mt --launch-args $G3 drive:=swerve nav_extra_overlay:=$X/nav2_3d_swerve_v4.yaml --out ${O}_rest >> $E/evidence/exp_critic_mt_rest.out 2>&1 || log "swerve $x exited $?"; clean
  log "swerve v4 mt $x: $(tally ${O}_rest $x)"; done
log "done"
