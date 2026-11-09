#!/bin/bash
# Everything on the final rocker coupling (K 30000, D 500, clamp 3000, 7f76021), in order of what
# decides v1.1: critic v2 (Ackermann TerrainCritic u_free 0.5, u_tip_lethal 0.80), the plain 3D
# gate, the 2D baseline, swerve v4 x2, then the re-approach tree. Smoke first: Ackermann slopes
# lane15 and marsyard ahead_8m x1 must succeed with roll under 10 deg, else stop (coupling unstable).
# Arg 1: deadline (epoch). All 3D runs: plugin suspension, ground truth, v_nominal 0.5, u_free 0.3.
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log; B=$HOME/.ros/rover_benchmark; X=$E/exp
log() { echo "$(date '+%F %H:%M') exp_final_susp: $*" >> $L; }
clean() { for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null; done
  python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1; }
tally() { awk -F, -v w=$2.sdf 'NR>1&&$3==w{print $4":"$6}' $1/runs.csv 2>/dev/null | sort | uniq -c | tr -s ' \n' ' '; }
source $E/tools/simenv.sh; cd /home/ram/ROS2_Autonomous_2026_27
D=$1; late() { [ "$(date +%s)" -ge "$D" ]; }
G3="odom_source:=ground_truth suspension:=plugin governor_v_nominal:=0.5 governor_u_free:=0.3"
run() { # tag out worlds... -- extra launch args
  local tag=$1 out=$2; shift 2; local w=(); while [ "$1" != "--" ]; do w+=($1); shift; done; shift
  for x in "${w[@]}"; do late && { log "deadline before $tag $x"; return 1; }
    $E/tools/guard.sh 9000 ros2 run rover_nav2 benchmark --worlds $x --runs 3 --tag $tag "$@" --out $out >> $E/evidence/exp_final_susp.out 2>&1 || log "$tag $x exited $?"; clean
    log "$tag $x: $(tally $out $x)"; done; }
log "start ($(git rev-parse --short HEAD))"
S=$B/2026-11-09_final_smoke
for g in "slopes lane15_top" "marsyard ahead_8m"; do set -- $g
  $E/tools/guard.sh 1800 ros2 run rover_nav2 benchmark --worlds $1 --goals $2 --runs 1 --config 3d --tag final_smoke --launch-args $G3 --out $S >> $E/evidence/exp_final_susp.out 2>&1; clean; done
bad=$(awk -F, 'NR>1 && ($7!=1 || $12>10 || $12<-10)' $S/runs.csv | wc -l); log "smoke: $(awk -F, 'NR>1{printf "%s %s roll %s; ",$4,$6,$12}' $S/runs.csv)"
[ "$bad" -ne 0 ] && { log "smoke failed, stop"; exit 1; }
O=$B/2026-11-10_3d_gt_ack_critic_v2
run 3d_gt_ack_critic_v2 $O slopes marsyard rock_field crater_field urc_mix bars -- --config 3d --launch-args $G3 nav_extra_overlay:=$X/nav2_3d_ack_terrain_critic_v2.yaml
O=$B/2026-11-10_3d_gt_gate_final
run 3d_gt_gate_final $O slopes marsyard rock_field crater_field urc_mix bars -- --config 3d --launch-args $G3
O=$B/2026-11-10_2d_base_final
run 2d_base_final $O slopes marsyard rock_field crater_field urc_mix bars -- --config 2d --params-file /home/ram/ROS2_Autonomous_2026_27/src/rover_nav2/config/nav2_params_sim_2d_bounded.yaml --launch-args odom_source:=ground_truth suspension:=plugin
late || { O=$B/2026-11-10_3d_gt_swerve_v4_final
  $E/tools/guard.sh 14400 ros2 run rover_nav2 benchmark --worlds marsyard slopes rock_field urc_mix --runs 2 --tag 3d_gt_swerve_v4_final --config 3d \
    --launch-args $G3 drive:=swerve nav_extra_overlay:=$X/nav2_3d_swerve_v4.yaml --out $O >> $E/evidence/exp_final_susp.out 2>&1 || log "swerve exited $?"; clean
  log "swerve v4: success $(awk -F, 'NR>1&&$7==1' $O/runs.csv | wc -l)/$(awk -F, 'NR>1' $O/runs.csv | wc -l), $(awk -F, 'NR>1{print $6}' $O/runs.csv | sort | uniq -c | tr -s ' \n' ' ')"; }
O=$B/2026-11-10_3d_gt_reapproach
run 3d_gt_reapproach $O marsyard urc_mix rock_field -- --config 3d --launch-args $G3 nav_extra_overlay:=$X/nav2_3d_reapproach.yaml
for t in ack_critic_v2 gate_final; do python3 $E/tools/phase3_gate.py $B/2026-11-10_3d_gt_$t/runs.csv $B/2026-11-10_2d_base_final/runs.csv $HOME/rover_3d_nav_results/analysis/phase3_gate_${t}_vs_2d_final.csv > $E/evidence/phase3_gate_${t}_final.txt 2>&1; done
log "done"
