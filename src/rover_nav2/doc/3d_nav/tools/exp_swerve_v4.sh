#!/bin/bash
# Swerve v4 (exp/nav2_3d_swerve_v4.yaml: slower lateral and yaw acceleration, less lateral noise) on
# the plugin suspension: recorded marsyard left_diag runs (v2 and v4 under 8 burners, v4 at light load) to measure the
# variation (v2: 124 deg/s after the smoother), then 14 goals x2 against 2026-11-09_3d_gt_swerve_susp_x2
# (13/28). Arg 1: PID to wait for. Arg 2: deadline (epoch).
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log; B=$HOME/.ros/rover_benchmark
log() { echo "$(date '+%F %H:%M') exp_swerve_v4: $*" >> $L; }
clean() { for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null; done
  python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1; }
while kill -0 $1 2>/dev/null; do [ "$(date +%s)" -ge "$2" ] && { log "deadline while waiting"; exit 1; }; sleep 20; done
cd /home/ram/ROS2_Autonomous_2026_27
log "start ($(git rev-parse --short HEAD))"
for o in v2 v4; do BURN=8 $E/tools/bag_run.sh swerve_$o marsyard.sdf 6.0 5.0 0.7 swerve $E/exp/nav2_3d_swerve_$o.yaml; done
$E/tools/bag_run.sh swerve_v4 marsyard.sdf 6.0 5.0 0.7 swerve $E/exp/nav2_3d_swerve_v4.yaml
source $E/tools/simenv.sh
O=$B/2026-11-09_3d_gt_swerve_v4
[ "$(date +%s)" -ge "$2" ] && { log "deadline before the benchmark"; exit 1; }
$E/tools/guard.sh 21600 ros2 run rover_nav2 benchmark --worlds marsyard slopes rock_field urc_mix --runs 2 --tag 3d_gt_swerve_v4 \
  --config 3d --launch-args odom_source:=ground_truth drive:=swerve suspension:=plugin governor_v_nominal:=0.5 governor_u_free:=0.3 \
  nav_extra_overlay:=$E/exp/nav2_3d_swerve_v4.yaml --out $O >> $E/evidence/exp_swerve_v4.out 2>&1 || log "exited $?"; clean
log "done: $(awk -F, 'NR>1{print $6}' $O/runs.csv | sort | uniq -c | tr -s ' \n' ' '), success $(awk -F, 'NR>1&&$7==1' $O/runs.csv | wc -l)/$(awk -F, 'NR>1' $O/runs.csv | wc -l)"
