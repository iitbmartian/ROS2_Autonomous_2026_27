#!/bin/bash
# v2 targeted, second half: swerve v6 on slopes, crater_field and the rest of urc_mix x2, same out
# dir as exp_sw5. Skips if exp_sw5 logged a shim failure. Args: deadline (epoch), pid to wait for.
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log; B=$HOME/.ros/rover_benchmark; X=$E/exp
log() { echo "$(date '+%F %H:%M') exp_sw6b: $*" >> $L; }
clean() { for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null; done
  python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1; }
DL=$1; while kill -0 $2 2>/dev/null; do sleep 30; done
grep -q "exp_sw6: shim failed" $L && { log "skipped, shim failed in exp_sw5"; exit 0; }
cd /home/ram/ROS2_Autonomous_2026_27; source /opt/ros/jazzy/setup.bash; source $E/tools/simenv.sh; unset ROVER_COUPLING_K ROVER_COUPLING_D ROVER_COUPLING_MAX
log "start"
t=3d_gt_sw6_d250; O=$B/2026-11-15_$t
A="--config 3d --launch-args odom_source:=ground_truth drive:=swerve suspension:=plugin governor_v_nominal:=0.5 governor_u_free:=0.3 nav_extra_overlay:=$X/nav2_3d_swerve_v6.yaml"
for spec in "slopes" "crater_field" "urc_mix --goals far_south far_east"; do [ "$(date +%s)" -ge "$DL" ] && break
  $E/tools/guard.sh 9000 ros2 run rover_nav2 benchmark --worlds $spec --runs 2 --tag $t $A --out $O >> $E/evidence/exp_sw6.out 2>&1; clean
  log "${spec%% *}: done"
done
log "done"
