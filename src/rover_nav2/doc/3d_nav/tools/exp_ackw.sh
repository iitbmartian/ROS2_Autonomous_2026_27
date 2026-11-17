#!/bin/bash
# Safe-heading mask with the safety monitor's tip margin (mask_tip_margin 0.05 rad). The lane25
# bag (run_ackhlane_burn8_1116_2337) showed the mask allowing headings 32 to 41 deg off the fall
# line on the 25 deg ramp: roll under the 16.1 deg limit but past the 13.2 deg abort line.
# VARIANT ackm: heading planner + margin. VARIANT ckm: v1.1 + margin (mask feeds the costmap).
# VARIANT ackf: ackm after the empty depth cloud fix (built from the current HEAD).
# Slopes x3, crater_field and marsyard x3 regression. Args: deadline (epoch), pid to wait for.
VARIANT=${VARIANT:-ackm}
# WORLDS and SUF (tag suffix) let one variant cover the other worlds of the full check.
WORLDS=${WORLDS:-slopes crater_field marsyard}; SUF=${SUF:-}
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log; B=$HOME/.ros/rover_benchmark; X=$E/exp
log() { echo "$(date '+%F %H:%M') exp_$VARIANT$SUF: $*" >> $L; }
clean() { for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null; done
  python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1; }
tally() { awk -F, -v w=$2.sdf 'NR>1&&$3==w{print $4":"$7}' $1/runs.csv 2>/dev/null | sort | uniq -c | tr -s ' \n' ' '; }
while kill -0 $2 2>/dev/null; do [ "$(date +%s)" -ge "$1" ] && { log "deadline while waiting"; exit 1; }; sleep 60; done
cd /home/ram/ROS2_Autonomous_2026_27; source /opt/ros/jazzy/setup.bash; source install/setup.bash
clean
log "build ($(git rev-parse --short HEAD))"
nice colcon build --packages-select rover_traversability rover_terrain_mapping rover_bringup rover_nav2 --cmake-args -DCMAKE_BUILD_TYPE=Release > $E/evidence/exp_${VARIANT}${SUF}_build.log 2>&1 || { log "build failed"; exit 1; }
source install/setup.bash
./build/rover_traversability/src/core/test_yaw_evaluation > $E/evidence/exp_${VARIANT}${SUF}_test.log 2>&1 || { log "yaw test failed"; exit 1; }
source $E/tools/simenv.sh; unset ROVER_COUPLING_K ROVER_COUPLING_D ROVER_COUPLING_MAX
log "start"
t=3d_gt_${VARIANT}${SUF}_d250; O=$B/$(date +%F)_$t
if [ $VARIANT = ackm ] || [ $VARIANT = ackf ]; then ov=nav2_3d_ack_heading.yaml; pid="--plan-planner-id Heading"; else ov=nav2_3d_reapproach.yaml; pid=""; fi
A="--config 3d $pid --launch-args odom_source:=ground_truth suspension:=plugin governor_v_nominal:=0.5 governor_u_free:=0.3 global_fine_step:=true mask_tip_margin:=0.05 nav_extra_overlay:=$X/$ov"
first=1
for w in $WORLDS; do
  [ "$(date +%s)" -ge "$1" ] && { log "deadline before $w"; break; }
  $E/tools/guard.sh 9000 ros2 run rover_nav2 benchmark --worlds $w --runs 3 --tag $t $A --out $O >> $E/evidence/exp_$VARIANT$SUF.out 2>&1 || log "$t $w exited $?"; clean
  log "$t $w successes: $(tally $O $w)"
  if [ $first = 1 ]; then first=0
    f=$(ls -tr $O/logs/*.log 2>/dev/null | head -1)
    grep -q "mask_tip_margin 0.050" "$f" || { log "mask_tip_margin not 0.05 in $f, stopping"; break; }
  fi
  [ "$(awk -F, 'NR>1' $O/runs.csv 2>/dev/null | grep -vc startup_failed)" = 0 ] && { log "all startup_failed, stopping"; break; }
done
log "done"
