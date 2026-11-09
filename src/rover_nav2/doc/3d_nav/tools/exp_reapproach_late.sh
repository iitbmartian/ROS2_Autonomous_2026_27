#!/bin/bash
# Fallback for exp_final_susp's last stage: if the 09:00 deadline skipped the re-approach tree
# (2026-11-10_3d_gt_reapproach has fewer than 27 rows), run the missing worlds after exp_bag_rock.
# Args: deadline (epoch), pid to wait for.
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log; B=$HOME/.ros/rover_benchmark; X=$E/exp
log() { echo "$(date '+%F %H:%M') exp_reapproach_late: $*" >> $L; }
clean() { for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null; done
  python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1; }
while kill -0 $2 2>/dev/null; do [ "$(date +%s)" -ge "$1" ] && { log "deadline while waiting"; exit 1; }; sleep 60; done
O=$B/2026-11-10_3d_gt_reapproach_late
source $E/tools/simenv.sh; cd /home/ram/ROS2_Autonomous_2026_27
G3="odom_source:=ground_truth suspension:=plugin governor_v_nominal:=0.5 governor_u_free:=0.3"
for x in marsyard urc_mix rock_field; do
  n=$(awk -F, -v w=$x.sdf 'NR>1&&$3==w' $B/2026-11-10_3d_gt_reapproach/runs.csv 2>/dev/null | wc -l)
  [ "$n" -ge 9 ] && continue
  [ "$(date +%s)" -ge "$1" ] && { log "deadline before $x"; break; }
  $E/tools/guard.sh 9000 ros2 run rover_nav2 benchmark --worlds $x --runs 3 --tag 3d_gt_reapproach --config 3d --launch-args $G3 nav_extra_overlay:=$X/nav2_3d_reapproach.yaml --out $O >> $E/evidence/exp_reapproach_late.out 2>&1 || log "$x exited $?"; clean
  log "$x: $(awk -F, -v w=$x.sdf 'NR>1&&$3==w{print $4":"$6}' $O/runs.csv | sort | uniq -c | tr -s ' \n' ' ')"
done
log "done"
