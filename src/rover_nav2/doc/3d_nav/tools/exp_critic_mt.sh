#!/bin/bash
# After exp_final_susp: the TerrainCritic split over 4 threads (trajectory_cost threads, critic
# param "threads", default 4). Every TerrainCritic config missed the 20 Hz control rate 200 to 630
# times per run (plain MPPI: 1), the critic alone took 18 to 34 ms of the 50 ms period on one core.
# Waits for exp_final_susp (pid arg 2) to exit, installs rover_traversability, then reruns the worlds
# where critic v2 failed (slopes, rock_field, crater_field x3) and swerve v4 (marsyard, slopes x2).
# Arg 1: deadline (epoch). Arg 2: pid to wait for.
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log; B=$HOME/.ros/rover_benchmark; X=$E/exp
log() { echo "$(date '+%F %H:%M') exp_critic_mt: $*" >> $L; }
clean() { for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null; done
  python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1; }
tally() { awk -F, -v w=$2.sdf 'NR>1&&$3==w{print $4":"$6}' $1/runs.csv 2>/dev/null | sort | uniq -c | tr -s ' \n' ' '; }
misses() { echo "$(cat $1/logs/*.log 2>/dev/null | grep -c 'missed its desired rate') misses / $(ls $1/logs 2>/dev/null | wc -l) runs"; }
D=$1; late() { [ "$(date +%s)" -ge "$D" ]; }
while kill -0 $2 2>/dev/null; do late && { log "deadline while waiting for $2"; exit 1; }; sleep 60; done
log "start, waited for $2"
source $E/tools/simenv.sh; cd /home/ram/ROS2_Autonomous_2026_27
nice -n 5 colcon build --packages-select rover_traversability > $E/evidence/exp_critic_mt_build.out 2>&1 || { log "build failed"; exit 1; }
source install/setup.bash
log "installed rover_traversability ($(git rev-parse --short HEAD))"
G3="odom_source:=ground_truth suspension:=plugin governor_v_nominal:=0.5 governor_u_free:=0.3"
run() { local tag=$1 out=$2; shift 2; local w=(); while [ "$1" != "--" ]; do w+=($1); shift; done; shift
  for x in "${w[@]}"; do late && { log "deadline before $tag $x"; return 1; }
    $E/tools/guard.sh 9000 ros2 run rover_nav2 benchmark --worlds $x "$@" --tag $tag --out $out >> $E/evidence/exp_critic_mt.out 2>&1 || log "$tag $x exited $?"; clean
    log "$tag $x: $(tally $out $x)"; done; log "$tag: $(misses $out)"; }
O=$B/2026-11-10_3d_gt_ack_critic_v2_mt
run 3d_gt_ack_critic_v2_mt $O slopes rock_field crater_field -- --runs 3 --config 3d --launch-args $G3 nav_extra_overlay:=$X/nav2_3d_ack_terrain_critic_v2.yaml
O=$B/2026-11-10_3d_gt_swerve_v4_mt
run 3d_gt_swerve_v4_mt $O marsyard slopes -- --runs 2 --config 3d --launch-args $G3 drive:=swerve nav_extra_overlay:=$X/nav2_3d_swerve_v4.yaml
log "done"
