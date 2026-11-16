#!/bin/bash
# Swerve v7 = v6 + heading plan (HeadingSmoother, path orientations followed). Targeted: the three
# goals v6 fails every time (slopes lane25, sidehill, crater far_left, 0/5 each) x3, plus
# regression goals urc_mix mid_20m and marsyard ahead_8m x3 (v6 3/3 and 3/3).
# Builds rover_traversability into the main install first, while no benchmark runs.
# Args: deadline (epoch), pid to wait for.
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log; B=$HOME/.ros/rover_benchmark; X=$E/exp
log() { echo "$(date '+%F %H:%M') exp_sw7: $*" >> $L; }
clean() { for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null; done
  python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1; }
tally() { awk -F, -v w=$2.sdf 'NR>1&&$3==w{print $4":"$6}' $1/runs.csv 2>/dev/null | sort | uniq -c | tr -s ' \n' ' '; }
while kill -0 $2 2>/dev/null; do [ "$(date +%s)" -ge "$1" ] && { log "deadline while waiting"; exit 1; }; sleep 60; done
cd /home/ram/ROS2_Autonomous_2026_27; source /opt/ros/jazzy/setup.bash; source install/setup.bash
clean
log "build rover_traversability ($(git rev-parse --short HEAD))"
nice colcon build --packages-select rover_traversability --cmake-args -DCMAKE_BUILD_TYPE=Release > $E/evidence/exp_sw7_build.log 2>&1 || { log "build failed, see evidence/exp_sw7_build.log"; exit 1; }
source install/setup.bash
ROS_DOMAIN_ID=81 ./build/rover_traversability/src/smoother/test_heading_smoother > $E/evidence/exp_sw7_test.log 2>&1 || { log "heading smoother test failed"; exit 1; }
source $E/tools/simenv.sh; unset ROVER_COUPLING_K ROVER_COUPLING_D ROVER_COUPLING_MAX
log "start"
t=3d_gt_sw7_d250; O=$B/$(date +%F)_$t
A="--config 3d --launch-args odom_source:=ground_truth suspension:=plugin governor_v_nominal:=0.5 governor_u_free:=0.3 drive:=swerve nav_extra_overlay:=$X/nav2_3d_swerve_v7.yaml"
for spec in "slopes --goals lane25_top sidehill_uphill" "crater_field --goals far_left" "urc_mix --goals mid_20m" "marsyard --goals ahead_8m"; do
  [ "$(date +%s)" -ge "$1" ] && break
  $E/tools/guard.sh 9000 ros2 run rover_nav2 benchmark --worlds $spec --runs 3 --tag $t $A --out $O >> $E/evidence/exp_sw7.out 2>&1 || log "$t ${spec%% *} exited $?"; clean
  log "$t ${spec%% *}: $(tally $O ${spec%% *})"
  # Stop early if the heading plan breaks the stack: every run so far failed to start.
  [ "$(awk -F, 'NR>1' $O/runs.csv 2>/dev/null | grep -vc startup_failed)" = 0 ] && { log "all startup_failed, stopping"; break; }
done
log "done"
