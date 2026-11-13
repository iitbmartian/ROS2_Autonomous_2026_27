#!/bin/bash
# Plain 3D at D 250 with the global features taking the step of the 0.05 m local map
# (global_fine_step:=true, fine_step.hpp), so rocks under 0.2 m stay in the global costmap.
# rock_field, urc_mix, marsyard, slopes x3, gate against 2D D 250. Args: deadline, pid to wait for.
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log; B=$HOME/.ros/rover_benchmark; X=$E/exp
log() { echo "$(date '+%F %H:%M') exp_fs: $*" >> $L; }
clean() { for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null; done
  python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1; }
tally() { awk -F, -v w=$2.sdf 'NR>1&&$3==w{print $4":"$6}' $1/runs.csv 2>/dev/null | sort | uniq -c | tr -s ' \n' ' '; }
while kill -0 $2 2>/dev/null; do [ "$(date +%s)" -ge "$1" ] && { log "deadline while waiting"; exit 1; }; sleep 60; done
cd /home/ram/ROS2_Autonomous_2026_27; source /opt/ros/jazzy/setup.bash
source $E/tools/simenv.sh; unset ROVER_COUPLING_K ROVER_COUPLING_D ROVER_COUPLING_MAX
grep -q "ROVER_COUPLING_D 250" install/rover_gazebo/share/rover_gazebo/urdf/rover_gazebo.xacro || { log "installed xacro is not D 250"; exit 1; }
log "start ($(git rev-parse --short HEAD)), D 250 default, global_fine_step"
G3="odom_source:=ground_truth suspension:=plugin governor_v_nominal:=0.5 governor_u_free:=0.3"
W="rock_field urc_mix marsyard slopes"
t=3d_gt_fs_d250; A="--config 3d --launch-args $G3 global_fine_step:=true"
O=$B/2026-11-13_$t
  for w in $W; do [ "$(date +%s)" -ge "$1" ] && { log "deadline before $t $w"; break; }
    $E/tools/guard.sh 9000 ros2 run rover_nav2 benchmark --worlds $w --runs 3 --tag $t $A --out $O >> $E/evidence/exp_fs.out 2>&1 || log "$t $w exited $?"; clean
    log "$t $w: $(tally $O $w)"
    [ $w = rock_field ] && log "fine step active: $(grep -l 'fine step from' $O/logs/*.log 2>/dev/null | wc -l) logs"
  done
python3 $E/tools/phase3_gate.py $B/2026-11-13_3d_gt_fs_d250/runs.csv $B/2026-11-13_2d_base_d250/runs.csv > $E/evidence/phase3_gate_fs_d250.txt 2>&1
log "gate: $(grep -c FAIL $E/evidence/phase3_gate_fs_d250.txt) FAIL items"
log "done"
