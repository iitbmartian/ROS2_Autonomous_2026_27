#!/bin/bash
# v2: swerve (v4 overlay, v1.1 defaults) on urc_mix mid_20m x3 through the benchmark, so poses and
# times are in sim time (the bags stamp wall time, inflated 4 to 5x by the recording load). Args: deadline, pid.
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log; B=$HOME/.ros/rover_benchmark; X=$E/exp
DL=$1; while kill -0 $2 2>/dev/null; do sleep 30; done
cd /home/ram/ROS2_Autonomous_2026_27; source /opt/ros/jazzy/setup.bash; source $E/tools/simenv.sh; unset ROVER_COUPLING_K ROVER_COUPLING_D ROVER_COUPLING_MAX
O=$B/2026-11-14_3d_gt_swmid_d250
echo "$(date '+%F %H:%M') exp_swmid: start" >> $L
$E/tools/guard.sh 5000 ros2 run rover_nav2 benchmark --worlds urc_mix --goals mid_20m --runs 3 --tag 3d_gt_swmid_d250 --config 3d --launch-args odom_source:=ground_truth drive:=swerve suspension:=plugin governor_v_nominal:=0.5 governor_u_free:=0.3 nav_extra_overlay:=$X/nav2_3d_swerve_v4.yaml --out $O >> $E/evidence/exp_swmid.out 2>&1
for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null; done; python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1
echo "$(date '+%F %H:%M') exp_swmid: $(awk -F, 'NR>1{print $6":"$9}' $O/runs.csv | tr '\n' ' ')" >> $L
echo "$(date '+%F %H:%M') exp_swmid: done" >> $L
