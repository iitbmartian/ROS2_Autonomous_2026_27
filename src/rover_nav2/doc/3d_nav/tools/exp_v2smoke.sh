#!/bin/bash
# v2 release smoke test: the 3D stack on its defaults, no overlay, no mask_tip_margin or planner
# arguments, so the heading planner, the 0.05 margin and the heading-first tree come from the
# installed config. lane25_top, marsyard behind_left (re-approach) and bars past_bars_6m, x2.
# Args: deadline (epoch), pid to wait for.
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log; B=$HOME/.ros/rover_benchmark
log() { echo "$(date '+%F %H:%M') exp_v2smoke: $*" >> $L; }
clean() { for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null; done
  python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1; }
tally() { awk -F, -v w=$2.sdf 'NR>1&&$3==w{print $4":"$7}' $1/runs.csv 2>/dev/null | sort | uniq -c | tr -s ' \n' ' '; }
while kill -0 $2 2>/dev/null; do [ "$(date +%s)" -ge "$1" ] && { log "deadline while waiting"; exit 1; }; sleep 60; done
cd /home/ram/ROS2_Autonomous_2026_27; source /opt/ros/jazzy/setup.bash; source install/setup.bash
clean
log "build ($(git rev-parse --short HEAD))"
nice colcon build --packages-select rover_traversability rover_terrain_mapping rover_bringup rover_nav2 --cmake-args -DCMAKE_BUILD_TYPE=Release > $E/evidence/exp_v2smoke_build.log 2>&1 || { log "build failed"; exit 1; }
source install/setup.bash
source $E/tools/simenv.sh; unset ROVER_COUPLING_K ROVER_COUPLING_D ROVER_COUPLING_MAX
log "start"
t=3d_gt_v2smoke_d250; O=$B/$(date +%F)_$t
A="--config 3d --launch-args odom_source:=ground_truth suspension:=plugin governor_v_nominal:=0.5 governor_u_free:=0.3"
first=1
for spec in "slopes --goals lane25_top" "marsyard --goals behind_left" "bars --goals past_bars_6m"; do
  [ "$(date +%s)" -ge "$1" ] && break
  $E/tools/guard.sh 3600 ros2 run rover_nav2 benchmark --worlds $spec --runs 2 --tag $t $A --out $O >> $E/evidence/exp_v2smoke.out 2>&1 || log "$t ${spec%% *} exited $?"; clean
  log "$t ${spec%% *}: $(tally $O ${spec%% *})"
  if [ $first = 1 ]; then first=0
    f=$(ls -tr $O/logs/*.log 2>/dev/null | head -1)
    grep -q "mask_tip_margin 0.050" "$f" && log "default mask_tip_margin 0.05: ok" || log "default mask_tip_margin NOT 0.05"
    grep -q "HeadingAwarePlanner:" "$f" && log "HeadingAwarePlanner loaded by default: ok" || log "HeadingAwarePlanner NOT loaded"
  fi
done
log "done"
