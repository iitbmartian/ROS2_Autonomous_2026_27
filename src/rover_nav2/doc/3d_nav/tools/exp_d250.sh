#!/bin/bash
# Final numbers on the new default coupling (K 30000, D 250, MAX 3000, ram/suspension 0d5e518):
# install rover_gazebo once idle, then 2D bounded and plain 3D with the lethal-free tree on all
# 6 worlds x3 with no coupling override, then the gate. Drift-check against the D 200 runs before
# any comparison is reported. Args: deadline (epoch), pid to wait for.
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log; B=$HOME/.ros/rover_benchmark; X=$E/exp
log() { echo "$(date '+%F %H:%M') exp_d250: $*" >> $L; }
clean() { for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null; done
  python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1; }
tally() { awk -F, -v w=$2.sdf 'NR>1&&$3==w{print $4":"$6}' $1/runs.csv 2>/dev/null | sort | uniq -c | tr -s ' \n' ' '; }
while kill -0 $2 2>/dev/null; do [ "$(date +%s)" -ge "$1" ] && { log "deadline while waiting"; exit 1; }; sleep 60; done
cd /home/ram/ROS2_Autonomous_2026_27; source /opt/ros/jazzy/setup.bash
colcon build --packages-select rover_gazebo > $E/evidence/exp_d250_build.out 2>&1 || { log "rover_gazebo build failed"; exit 1; }
source $E/tools/simenv.sh; unset ROVER_COUPLING_K ROVER_COUPLING_D ROVER_COUPLING_MAX
grep -q "ROVER_COUPLING_D 250" install/rover_gazebo/share/rover_gazebo/urdf/rover_gazebo.xacro || { log "installed xacro is not D 250"; exit 1; }
log "start ($(git rev-parse --short HEAD)), rover_gazebo installed with D 250"
G3="odom_source:=ground_truth suspension:=plugin governor_v_nominal:=0.5 governor_u_free:=0.3"
W="marsyard rock_field urc_mix slopes crater_field bars"
for st in 3d 2d; do
  if [ $st = 3d ]; then t=3d_gt_lethalfree_d250; A="--config 3d --launch-args $G3 nav_extra_overlay:=$X/nav2_3d_lethalfree.yaml"
  else t=2d_base_d250; A="--config 2d --params-file /home/ram/ROS2_Autonomous_2026_27/src/rover_nav2/config/nav2_params_sim_2d_bounded.yaml --launch-args odom_source:=ground_truth suspension:=plugin"; fi
  O=$B/2026-11-13_$t
  for w in $W; do [ "$(date +%s)" -ge "$1" ] && { log "deadline before $t $w"; break 2; }
    $E/tools/guard.sh 9000 ros2 run rover_nav2 benchmark --worlds $w --runs 3 --tag $t $A --out $O >> $E/evidence/exp_d250.out 2>&1 || log "$t $w exited $?"; clean
    log "$t $w: $(tally $O $w)"
  done
done
python3 $E/tools/phase3_gate.py $B/2026-11-13_3d_gt_lethalfree_d250/runs.csv $B/2026-11-13_2d_base_d250/runs.csv > $E/evidence/phase3_gate_lethalfree_d250.txt 2>&1
log "gate: $(grep -c FAIL $E/evidence/phase3_gate_lethalfree_d250.txt) FAIL items"
log "done"
