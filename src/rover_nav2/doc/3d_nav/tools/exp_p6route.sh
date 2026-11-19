#!/bin/bash
# Phase 6 gate item 1: multi-waypoint runs of at least 100 m complete with zero unsafe entries.
# Goal set urc_mix_routes (3 loops, 101 to 108 m straight-line, waypoints clear of craters, rocks,
# ramp lanes and the sidehill). The navigation stack is the installed v2 (main install). Only the
# benchmark runner comes from the v3 worktree overlay (~/ws_v3/install: route support and the
# timing-plan planner id fix). Ground-truth odometry, as every run until the EKF is approved.
# A smoke run of one route first, the rest only if it navigated.
# Args: deadline (epoch), pid to wait for.
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log; B=$HOME/.ros/rover_benchmark
log() { echo "$(date '+%F %H:%M') exp_p6route: $*" >> $L; }
clean() { for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null; done
  python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1; }
tally() { awk -F, 'NR>1{print $4":"$7}' $1/runs.csv 2>/dev/null | sort | uniq -c | tr -s ' \n' ' '; }
while kill -0 $2 2>/dev/null; do [ "$(date +%s)" -ge "$1" ] && { log "deadline while waiting"; exit 1; }; sleep 60; done
cd /home/ram/ROS2_Autonomous_2026_27; source /opt/ros/jazzy/setup.bash; source install/setup.bash
source $HOME/ws_v3/install/setup.bash
[ -f "$(ros2 pkg prefix rover_nav2)/share/rover_nav2/benchmark/goals/urc_mix_routes.yaml" ] || { log "v3 overlay not active (no urc_mix_routes), stopping"; exit 1; }
clean
source $E/tools/simenv.sh; unset ROVER_COUPLING_K ROVER_COUPLING_D ROVER_COUPLING_MAX
t=3d_gt_p6route_d250; O=$B/$(date +%F)_$t
[ -e $O ] && { log "out dir $O exists, a rerun would skip its done runs, stopping"; exit 1; }
log "start (stack install of $(git rev-parse --short HEAD), runner from ws_v3 $(git -C $HOME/ws_v3 rev-parse --short HEAD)), out $O"
A="--config 3d --launch-args odom_source:=ground_truth suspension:=plugin governor_v_nominal:=0.5 governor_u_free:=0.3"
# Smoke: one route once. A route takes up to 1800 s sim, about 75 min wall at RTF 0.4.
$E/tools/guard.sh 6000 ros2 run rover_nav2 benchmark --worlds urc_mix_routes --goals loop_east --runs 1 --tag $t $A --out $O >> $E/evidence/exp_p6route.out 2>&1 || log "$t smoke exited $?"; clean
log "$t smoke: $(tally $O) $(grep -o 'waypoints [0-9]*/[0-9]*' $O/runs.csv | tail -1)"
grep -q 'waypoints [1-9]' $O/runs.csv 2>/dev/null || { log "smoke reached no waypoint, stopping for a look"; log "done"; exit 1; }
[ "$(date +%s)" -ge "$1" ] && { log "deadline before the full set"; log "done"; exit 1; }
# The benchmark resumes the out dir, so loop_east r1 (the smoke) is not run again.
$E/tools/guard.sh 36000 ros2 run rover_nav2 benchmark --worlds urc_mix_routes --runs 3 --tag $t $A --out $O >> $E/evidence/exp_p6route.out 2>&1 || log "$t routes exited $?"; clean
log "$t routes: $(tally $O), $(grep -o 'waypoints [0-9]*/[0-9]*' $O/runs.csv | sort | uniq -c | tr -s ' \n' ' ')"
log "done"
