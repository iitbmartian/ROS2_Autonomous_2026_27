#!/bin/bash
# v2 targeted: swerve v6 (rotation shim at start and goal, PreferForward, TwirlingCritic kept at 10)
# against v4. v5 lowered the twirl penalty and rolled to -13 deg mid-route. urc_mix mid_20m x3, then
# marsyard and rock_field x2. Args: deadline (epoch), pid to wait for.
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log; B=$HOME/.ros/rover_benchmark; X=$E/exp
log() { echo "$(date '+%F %H:%M') exp_sw6: $*" >> $L; }
clean() { for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null; done
  python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1; }
DL=$1; while kill -0 $2 2>/dev/null; do sleep 30; done
cd /home/ram/ROS2_Autonomous_2026_27; source /opt/ros/jazzy/setup.bash; source $E/tools/simenv.sh; unset ROVER_COUPLING_K ROVER_COUPLING_D ROVER_COUPLING_MAX
log "start ($(git rev-parse --short HEAD))"
t=3d_gt_sw6_d250; O=$B/2026-11-15_$t
A="--config 3d --launch-args odom_source:=ground_truth drive:=swerve suspension:=plugin governor_v_nominal:=0.5 governor_u_free:=0.3 nav_extra_overlay:=$X/nav2_3d_swerve_v6.yaml"
$E/tools/guard.sh 5000 ros2 run rover_nav2 benchmark --worlds urc_mix --goals mid_20m --runs 3 --tag $t $A --out $O >> $E/evidence/exp_sw6.out 2>&1; clean
log "mid_20m: $(awk -F, 'NR>1{print $6":"$9}' $O/runs.csv | tr '\n' ' ')"
if grep -qE "RotationShim.*(rror|xception)|Failed to create controller" $O/logs/*.log; then log "shim failed to load, stopping"; exit 1; fi
for w in marsyard rock_field; do [ "$(date +%s)" -ge "$DL" ] && break
  $E/tools/guard.sh 7000 ros2 run rover_nav2 benchmark --worlds $w --runs 2 --tag $t $A --out $O >> $E/evidence/exp_sw6.out 2>&1; clean
  log "$w: $(awk -F, -v w=$w.sdf 'NR>1&&$3==w{print $4":"$6}' $O/runs.csv | sort | uniq -c | tr -s ' \n' ' ')"
done
log "done"
