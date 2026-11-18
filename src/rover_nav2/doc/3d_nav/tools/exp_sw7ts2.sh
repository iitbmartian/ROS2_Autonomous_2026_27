#!/bin/bash
# Swerve v7 + mask_tip_margin 0.05: the HeadingSmoother picks body headings from the safe-heading
# mask, which allowed headings past the tip monitor abort line. Same targeted goals as exp_sw7
# sw7t: sw7f plus the start-heading anchor in the HeadingSmoother (plan keeps the body yaw the rover holds, no 180 deg flips between replans that the rotation shim chased by spinning in place, bag run_v7lane_burn8_1116_2056). Builds rover_traversability first.
# sw7t: sw7a without the TerrainCritic in MPPI (overlay nav2_3d_swerve_v7t.yaml). Run sw7a dithered at the lane25 ramp foot. No build.
# sw7ts2: the slopes part of sw7t again. The sw7t benchmark hung on a futex after lane25 r1 (18 Nov 10:08) and was stopped, so its slopes runs were lost.
# Args: deadline (epoch), pid to wait for.
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log; B=$HOME/.ros/rover_benchmark; X=$E/exp
log() { echo "$(date '+%F %H:%M') exp_sw7ts2: $*" >> $L; }
clean() { for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null; done
  python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1; }
tally() { awk -F, -v w=$2.sdf 'NR>1&&$3==w{print $4":"$6}' $1/runs.csv 2>/dev/null | sort | uniq -c | tr -s ' \n' ' '; }
while kill -0 $2 2>/dev/null; do [ "$(date +%s)" -ge "$1" ] && { log "deadline while waiting"; exit 1; }; sleep 60; done
cd /home/ram/ROS2_Autonomous_2026_27; source /opt/ros/jazzy/setup.bash; source install/setup.bash
clean
source $E/tools/simenv.sh; unset ROVER_COUPLING_K ROVER_COUPLING_D ROVER_COUPLING_MAX
log "start"
t=3d_gt_sw7ts2_d250; O=$B/$(date +%F)_$t
[ -e $O ] && { log "out dir $O exists, a rerun would skip its done runs, stopping"; exit 1; }
A="--config 3d --launch-args odom_source:=ground_truth suspension:=plugin governor_v_nominal:=0.5 governor_u_free:=0.3 drive:=swerve mask_tip_margin:=0.05 nav_extra_overlay:=$X/nav2_3d_swerve_v7t.yaml"
for spec in "slopes --goals lane25_top sidehill_uphill"; do
  [ "$(date +%s)" -ge "$1" ] && break
  $E/tools/guard.sh 9000 ros2 run rover_nav2 benchmark --worlds $spec --runs 3 --tag $t $A --out $O >> $E/evidence/exp_sw7ts2.out 2>&1 || log "$t ${spec%% *} exited $?"; clean
  log "$t ${spec%% *}: $(tally $O ${spec%% *})"
  grep -q "anchor on" $(ls -tr $O/logs/*.log | head -1) || { log "HeadingSmoother anchor not on, stopping"; break; }
  grep -q "Critic loaded : mppi::critics::TerrainCritic" $(ls -tr $O/logs/*.log | head -1) && { log "TerrainCritic still loaded, stopping"; break; }
  # Stop early if the heading plan breaks the stack: every run so far failed to start.
  [ "$(awk -F, 'NR>1' $O/runs.csv 2>/dev/null | grep -vc startup_failed)" = 0 ] && { log "all startup_failed, stopping"; break; }
done
log "done"
