#!/bin/bash
# Swerve v7 on all 6 worlds x3, the v2 swerve candidate run, compared with v6 (sw6full).
# Runs only if exp_sw7's regression goals (urc_mix mid_20m, marsyard ahead_8m) held 4 of 6.
# Args: deadline (epoch), pid to wait for.
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log; B=$HOME/.ros/rover_benchmark; X=$E/exp
log() { echo "$(date '+%F %H:%M') exp_sw7full: $*" >> $L; }
clean() { for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null; done
  python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1; }
tally() { awk -F, -v w=$2.sdf 'NR>1&&$3==w{print $4":"$6}' $1/runs.csv 2>/dev/null | sort | uniq -c | tr -s ' \n' ' '; }
while kill -0 $2 2>/dev/null; do [ "$(date +%s)" -ge "$1" ] && { log "deadline while waiting"; exit 1; }; sleep 60; done
cd /home/ram/ROS2_Autonomous_2026_27; source /opt/ros/jazzy/setup.bash; source install/setup.bash
clean
R=$(ls -td $B/*_3d_gt_sw7_d250 2>/dev/null | head -1)
ok=$(awk -F, 'NR>1&&($3=="urc_mix.sdf"||$3=="marsyard.sdf")&&$7==1' $R/runs.csv 2>/dev/null | wc -l)
[ "$ok" -ge 4 ] || { log "skipped, exp_sw7 regression goals $ok/6"; exit 0; }
source $E/tools/simenv.sh; unset ROVER_COUPLING_K ROVER_COUPLING_D ROVER_COUPLING_MAX
log "start"
t=3d_gt_sw7full_d250; O=$B/$(date +%F)_$t
A="--config 3d --launch-args odom_source:=ground_truth suspension:=plugin governor_v_nominal:=0.5 governor_u_free:=0.3 drive:=swerve nav_extra_overlay:=$X/nav2_3d_swerve_v7.yaml"
for spec in marsyard rock_field urc_mix slopes crater_field bars; do
  [ "$(date +%s)" -ge "$1" ] && break
  $E/tools/guard.sh 9000 ros2 run rover_nav2 benchmark --worlds $spec --runs 3 --tag $t $A --out $O >> $E/evidence/exp_sw7full.out 2>&1 || log "$t ${spec%% *} exited $?"; clean
  log "$t ${spec%% *}: $(tally $O ${spec%% *})"
done
python3 $E/tools/phase3_gate.py $O/runs.csv $B/2026-11-13_2d_base_d250/runs.csv > $E/evidence/phase4_gate_sw7full_d250.txt 2>&1
log "gate: $(grep -c FAIL $E/evidence/phase4_gate_sw7full_d250.txt) FAIL items"
log "done"
