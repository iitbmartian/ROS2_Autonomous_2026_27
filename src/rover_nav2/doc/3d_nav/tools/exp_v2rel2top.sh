#!/bin/bash
# v2rel2 top-up: slopes past_lanes x3 into the existing 2026-11-18 v2rel2 out dir. Those runs were
# lost when a mis-chained exp_v2weak ran its clean() at 22:05 and killed the v2rel2 stack. The
# benchmark resumes an existing out dir, so only the missing runs execute. No build.
# Args: deadline (epoch), pid to wait for.
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log; B=$HOME/.ros/rover_benchmark
log() { echo "$(date '+%F %H:%M') exp_v2rel2top: $*" >> $L; }
clean() { for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null; done
  python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1; }
tally() { awk -F, -v w=$2.sdf 'NR>1&&$3==w{print $4":"$7}' $1/runs.csv 2>/dev/null | sort | uniq -c | tr -s ' \n' ' '; }
while kill -0 $2 2>/dev/null; do [ "$(date +%s)" -ge "$1" ] && { log "deadline while waiting"; exit 1; }; sleep 60; done
cd /home/ram/ROS2_Autonomous_2026_27; source /opt/ros/jazzy/setup.bash; source install/setup.bash
clean
source $E/tools/simenv.sh; unset ROVER_COUPLING_K ROVER_COUPLING_D ROVER_COUPLING_MAX
t=3d_gt_v2rel2_d250; O=$B/2026-11-18_$t
[ -d $O ] || { log "out dir $O missing, stopping"; exit 1; }
log "start (install of $(git rev-parse --short HEAD))"
A="--config 3d --launch-args odom_source:=ground_truth suspension:=plugin governor_v_nominal:=0.5 governor_u_free:=0.3"
$E/tools/guard.sh 3600 ros2 run rover_nav2 benchmark --worlds slopes --goals past_lanes --runs 3 --tag $t $A --out $O >> $E/evidence/exp_v2rel2.out 2>&1 || log "$t slopes exited $?"; clean
log "$t slopes successes: $(tally $O slopes)"
log "done"
