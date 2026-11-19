#!/bin/bash
# p6route2d: the 2D Nav2 baseline (bounded 2D params, as exp_2drep) on the Phase 6 routes urc_mix_routes,
# 3 routes x3, so the 100 m route result has a 2D comparison. The stack is the main install, only the
# benchmark runner comes from ~/ws_v3/install (route support). Ground-truth odometry. No smoke gate,
# a 2D run that reaches no waypoint is a valid result.
# Args: deadline (epoch), pid to wait for.
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log; B=$HOME/.ros/rover_benchmark
log() { echo "$(date '+%F %H:%M') exp_p6route2d: $*" >> $L; }
clean() { for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null; done
  python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1; }
tally() { awk -F, 'NR>1{print $4":"$7}' $1/runs.csv 2>/dev/null | sort | uniq -c | tr -s ' \n' ' '; }
while kill -0 $2 2>/dev/null; do [ "$(date +%s)" -ge "$1" ] && { log "deadline while waiting"; exit 1; }; sleep 60; done
cd /home/ram/ROS2_Autonomous_2026_27; source /opt/ros/jazzy/setup.bash; source install/setup.bash
source $HOME/ws_v3/install/setup.bash
[ -f "$(ros2 pkg prefix rover_nav2)/share/rover_nav2/benchmark/goals/urc_mix_routes.yaml" ] || { log "v3 overlay not active (no urc_mix_routes), stopping"; exit 1; }
clean
source $E/tools/simenv.sh; unset ROVER_COUPLING_K ROVER_COUPLING_D ROVER_COUPLING_MAX
t=3d_gt_p6route2d_d250; O=$B/$(date +%F)_$t
[ -e $O ] && { log "out dir $O exists, a rerun would skip its done runs, stopping"; exit 1; }
log "start (stack install of $(git rev-parse --short HEAD), runner from ws_v3 $(git -C $HOME/ws_v3 rev-parse --short HEAD)), out $O"
A="--config 2d --params-file /home/ram/ROS2_Autonomous_2026_27/src/rover_nav2/config/nav2_params_sim_2d_bounded.yaml --launch-args odom_source:=ground_truth suspension:=plugin"
$E/tools/guard.sh 36000 ros2 run rover_nav2 benchmark --worlds urc_mix_routes --runs 3 --tag $t $A --out $O >> $E/evidence/exp_p6route2d.out 2>&1 || log "$t routes exited $?"; clean
log "$t routes: $(tally $O), $(grep -o 'waypoints [0-9]*/[0-9]*' $O/runs.csv | sort | uniq -c | tr -s ' \n' ' ')"
[ "$(awk -F, 'NR>1' $O/runs.csv 2>/dev/null | grep -vc startup_failed)" = 0 ] && log "all startup_failed"
log "done"
