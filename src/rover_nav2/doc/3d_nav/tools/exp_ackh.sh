#!/bin/bash
# Ackermann heading-aware planning (v2): v1.1 plus the HeadingAwarePlanner (Smac fallback).
# Targeted: slopes, all 5 goals x3 (lane25 enters the ramp turning, v1.1 0/3 to 1/3), then
# regression crater_field and marsyard x3. Compared with ckpt, ck4, ck5 (same goals, v1.1).
# Builds rover_traversability into the main install first, while no benchmark runs, and stops if
# the planner did not load. Args: deadline (epoch), pid to wait for.
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log; B=$HOME/.ros/rover_benchmark; X=$E/exp
log() { echo "$(date '+%F %H:%M') exp_ackh: $*" >> $L; }
clean() { for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null; done
  python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1; }
tally() { awk -F, -v w=$2.sdf 'NR>1&&$3==w{print $4":"$6}' $1/runs.csv 2>/dev/null | sort | uniq -c | tr -s ' \n' ' '; }
while kill -0 $2 2>/dev/null; do [ "$(date +%s)" -ge "$1" ] && { log "deadline while waiting"; exit 1; }; sleep 60; done
cd /home/ram/ROS2_Autonomous_2026_27; source /opt/ros/jazzy/setup.bash; source install/setup.bash
clean
log "build rover_traversability rover_nav2 ($(git rev-parse --short HEAD))"
nice colcon build --packages-select rover_traversability rover_nav2 --cmake-args -DCMAKE_BUILD_TYPE=Release > $E/evidence/exp_ackh_build.log 2>&1 || { log "build failed, see evidence/exp_ackh_build.log"; exit 1; }
source install/setup.bash
./build/rover_traversability/src/core/test_heading_aware_search > $E/evidence/exp_ackh_test.log 2>&1 || { log "search test failed"; exit 1; }
source $E/tools/simenv.sh; unset ROVER_COUPLING_K ROVER_COUPLING_D ROVER_COUPLING_MAX
log "start"
t=3d_gt_ackh_d250; O=$B/$(date +%F)_$t
A="--config 3d --launch-args odom_source:=ground_truth suspension:=plugin governor_v_nominal:=0.5 governor_u_free:=0.3 global_fine_step:=true nav_extra_overlay:=$X/nav2_3d_ack_heading.yaml"
first=1
for w in slopes crater_field marsyard; do
  [ "$(date +%s)" -ge "$1" ] && { log "deadline before $w"; break; }
  $E/tools/guard.sh 9000 ros2 run rover_nav2 benchmark --worlds $w --runs 3 --tag $t $A --out $O >> $E/evidence/exp_ackh.out 2>&1 || log "$t $w exited $?"; clean
  log "$t $w: $(tally $O $w)"
  if [ $first = 1 ]; then first=0
    f=$(ls -tr $O/logs/*.log 2>/dev/null | head -1)
    grep -q "HeadingAwarePlanner:" "$f" || { log "planner not loaded in $f, stopping"; break; }
    log "planner fallbacks to Smac in first run: $(grep -c 'HeadingAwarePlanner: no path' "$f")"
  fi
  [ "$(awk -F, 'NR>1' $O/runs.csv 2>/dev/null | grep -vc startup_failed)" = 0 ] && { log "all startup_failed, stopping"; break; }
done
log "done"
