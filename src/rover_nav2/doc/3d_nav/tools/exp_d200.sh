#!/bin/bash
# Coupling damping fix (PROGRESS 2026-11-10 16:10): D 500 makes the body shake on flat ground, D 200
# does not at K 30000. 1) Loaded swerve v4 spin bag at K 30000 D 200, the run that showed the loop
# slip: must keep coupling residuals under 5 deg and not tip-stop, else stop here. 2) The three-way
# rerun at D 200: smoke, critic v2 (threaded), plain 3D, 2D bounded, all 6 worlds x3, then the gate.
# Args: deadline (epoch), pid to wait for.
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log; B=$HOME/.ros/rover_benchmark; X=$E/exp
log() { echo "$(date '+%F %H:%M') exp_d200: $*" >> $L; }
clean() { for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null; done
  python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1; }
tally() { awk -F, -v w=$2.sdf 'NR>1&&$3==w{print $4":"$6}' $1/runs.csv 2>/dev/null | sort | uniq -c | tr -s ' \n' ' '; }
while kill -0 $2 2>/dev/null; do [ "$(date +%s)" -ge "$1" ] && { log "deadline while waiting"; exit 1; }; sleep 60; done
source $E/tools/simenv.sh; cd /home/ram/ROS2_Autonomous_2026_27
export ROVER_COUPLING_K=30000 ROVER_COUPLING_D=200 ROVER_COUPLING_MAX=3000
D=$1; late() { [ "$(date +%s)" -ge "$D" ]; }
log "start ($(git rev-parse --short HEAD)), K $ROVER_COUPLING_K D $ROVER_COUPLING_D MAX $ROVER_COUPLING_MAX"
BURN=8 $E/tools/guard.sh 1800 $E/tools/bag_run.sh swerve_v4_d200 marsyard.sdf 6.0 5.0 0.7 swerve $X/nav2_3d_swerve_v4.yaml; clean
bag=$(ls -td $E/bags/run_swerve_v4_d200_burn8_* | grep -v '\.' | head -1)
r=$(python3 $E/tools/rocker_residuals.py $bag 2>&1 | head -2 | tr '\n' ' ')
g=$(tr '\n' ' ' < $bag.goal.out)
log "spin check: $r | goal: $g"
emax=$(echo "$r" | grep -oE 'e[123] [0-9.]+' | awk '{if($2>m)m=$2} END{print m+0}')
if echo "$g" | grep -q "tip risk" || [ -z "$emax" ] || awk -v m="$emax" 'BEGIN{exit !(m>=5)}'; then
  log "spin check failed (max residual $emax deg or tip stop), D 200 not adopted, stop"; exit 1; fi
log "spin check passed (max residual $emax deg)"
G3="odom_source:=ground_truth suspension:=plugin governor_v_nominal:=0.5 governor_u_free:=0.3"
run() { # tag out worlds... -- extra args
  local tag=$1 out=$2; shift 2; local w=(); while [ "$1" != "--" ]; do w+=($1); shift; done; shift
  for x in "${w[@]}"; do late && { log "deadline before $tag $x"; return 1; }
    $E/tools/guard.sh 9000 ros2 run rover_nav2 benchmark --worlds $x --runs 3 --tag $tag "$@" --out $out >> $E/evidence/exp_d200.out 2>&1 || log "$tag $x exited $?"; clean
    log "$tag $x: $(tally $out $x)"; done; }
W="slopes marsyard rock_field crater_field urc_mix bars"
run 3d_gt_ack_critic_v2_d200 $B/2026-11-11_3d_gt_ack_critic_v2_d200 $W -- --config 3d --launch-args $G3 nav_extra_overlay:=$X/nav2_3d_ack_terrain_critic_v2.yaml
run 3d_gt_gate_d200 $B/2026-11-11_3d_gt_gate_d200 $W -- --config 3d --launch-args $G3
run 2d_base_d200 $B/2026-11-11_2d_base_d200 $W -- --config 2d --params-file /home/ram/ROS2_Autonomous_2026_27/src/rover_nav2/config/nav2_params_sim_2d_bounded.yaml --launch-args odom_source:=ground_truth suspension:=plugin
for t in ack_critic_v2 gate; do
  python3 $E/tools/phase3_gate.py $B/2026-11-11_3d_gt_${t}_d200/runs.csv $B/2026-11-11_2d_base_d200/runs.csv \
    $HOME/rover_3d_nav_results/analysis/phase3_gate_${t}_d200.csv > $E/evidence/phase3_gate_${t}_d200.txt 2>&1; done
log "done"
