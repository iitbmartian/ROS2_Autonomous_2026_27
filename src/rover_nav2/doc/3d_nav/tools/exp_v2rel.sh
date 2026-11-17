#!/bin/bash
# v2 release run: the full 6-world benchmark on the installed v2 defaults (no overlay, no planner
# or margin arguments), 3 runs per goal. No build step, so it runs exactly the release install.
# It is also the v3 drift baseline. Args: deadline (epoch), pid to wait for.
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log; B=$HOME/.ros/rover_benchmark
log() { echo "$(date '+%F %H:%M') exp_v2rel: $*" >> $L; }
clean() { for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null; done
  python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1; }
tally() { awk -F, -v w=$2.sdf 'NR>1&&$3==w{print $4":"$7}' $1/runs.csv 2>/dev/null | sort | uniq -c | tr -s ' \n' ' '; }
while kill -0 $2 2>/dev/null; do [ "$(date +%s)" -ge "$1" ] && { log "deadline while waiting"; exit 1; }; sleep 60; done
cd /home/ram/ROS2_Autonomous_2026_27; source /opt/ros/jazzy/setup.bash; source install/setup.bash
clean
source $E/tools/simenv.sh; unset ROVER_COUPLING_K ROVER_COUPLING_D ROVER_COUPLING_MAX
log "start (install of $(git rev-parse --short HEAD))"
t=3d_gt_v2rel_d250; O=$B/$(date +%F)_$t
A="--config 3d --launch-args odom_source:=ground_truth suspension:=plugin governor_v_nominal:=0.5 governor_u_free:=0.3"
first=1
for w in slopes crater_field marsyard rock_field urc_mix bars; do
  [ "$(date +%s)" -ge "$1" ] && { log "deadline before $w"; break; }
  $E/tools/guard.sh 9000 ros2 run rover_nav2 benchmark --worlds $w --runs 3 --tag $t $A --out $O >> $E/evidence/exp_v2rel.out 2>&1 || log "$t $w exited $?"; clean
  log "$t $w successes: $(tally $O $w)"
  if [ $first = 1 ]; then first=0
    f=$(ls -tr $O/logs/*.log 2>/dev/null | head -1)
    grep -q "mask_tip_margin 0.050" "$f" && grep -q "HeadingAwarePlanner:" "$f" || { log "v2 defaults not loaded in $f, stopping"; break; }
  fi
  [ "$(awk -F, 'NR>1' $O/runs.csv 2>/dev/null | grep -vc startup_failed)" = 0 ] && { log "all startup_failed, stopping"; break; }
done
log "done"
