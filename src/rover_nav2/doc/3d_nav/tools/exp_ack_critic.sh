#!/bin/bash
# Phase 4b first step: TerrainCritic on the Ackermann 3D controller (exp/nav2_3d_ack_terrain_critic.yaml),
# plugin suspension, u_free 0.3, ground truth. slopes and marsyard x3, then rock_field, crater_field
# and urc_mix x3, against 2026-11-09_3d_gt_gate_clean. Arg 1: PID to wait for. Arg 2: deadline (epoch).
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log; B=$HOME/.ros/rover_benchmark
log() { echo "$(date '+%F %H:%M') exp_ack_critic: $*" >> $L; }
clean() { for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null; done
  python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1; }
while kill -0 $1 2>/dev/null; do [ "$(date +%s)" -ge "$2" ] && { log "deadline while waiting"; exit 1; }; sleep 20; done
source $E/tools/simenv.sh; cd /home/ram/ROS2_Autonomous_2026_27
log "start ($(git rev-parse --short HEAD))"
O=$B/2026-11-09_3d_gt_ack_critic
A="--config 3d --tag 3d_gt_ack_critic --launch-args odom_source:=ground_truth suspension:=plugin governor_v_nominal:=0.5 governor_u_free:=0.3 nav_extra_overlay:=$E/exp/nav2_3d_ack_terrain_critic.yaml"
for w in slopes marsyard rock_field crater_field urc_mix; do
  [ "$(date +%s)" -ge "$2" ] && { log "deadline before $w"; break; }
  $E/tools/guard.sh 9000 ros2 run rover_nav2 benchmark --worlds $w --runs 3 $A --out $O >> $E/evidence/exp_ack_critic.out 2>&1 || log "$w exited $?"; clean
  log "$w: $(awk -F, -v w=$w.sdf '$3==w{print $4":"$6}' $O/runs.csv | sort | uniq -c | tr -s ' \n' ' ')"
done
python3 $E/tools/phase3_gate.py $O/runs.csv $B/2026-11-09_2d_base_susp/runs.csv $HOME/rover_3d_nav_results/analysis/phase3_gate_ack_critic_vs_2d_susp.csv > $E/evidence/phase3_gate_ack_critic.txt 2>&1
log "done"
