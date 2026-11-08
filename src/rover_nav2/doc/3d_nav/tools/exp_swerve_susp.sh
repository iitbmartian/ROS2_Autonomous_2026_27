#!/bin/bash
# Swerve v2 on the plugin suspension (ram/suspension merged into ram/3d_nav): smoke on one goal,
# then all 14 goals x1 on marsyard, slopes, rock_field, urc_mix, against swerve v2 on the torque node
# (2026-11-07_3d_gt_swerve_v2: 6/14, 4 tip stops). Then Ackermann u_free 0.3 on the gate goals
# that had tip stops, to see whether they were artifacts. Starts now.
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log; B=$HOME/.ros/rover_benchmark
log() { echo "$(date '+%F %H:%M') exp_swerve_susp: $*" >> $L; }
clean() { for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null; done
  python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1; }
source $E/tools/simenv.sh; cd /home/ram/ROS2_Autonomous_2026_27
log "start ($(git rev-parse --short HEAD))"
SW="--config 3d --launch-args odom_source:=ground_truth drive:=swerve suspension:=plugin governor_v_nominal:=0.5 governor_u_free:=0.3 nav_extra_overlay:=$E/exp/nav2_3d_swerve_v2.yaml"
O=$B/2026-11-08_3d_gt_swerve_susp
$E/tools/guard.sh 1800 ros2 run rover_nav2 benchmark --worlds marsyard --goals ahead_8m --runs 1 --tag 3d_gt_swerve_susp_smoke $SW \
  --out ${O}_smoke >> $E/evidence/exp_swerve_susp.out 2>&1; clean
if ! grep -q ",succeeded,1," ${O}_smoke/runs.csv 2>/dev/null; then
  log "smoke failed: $(tail -n +2 ${O}_smoke/runs.csv 2>/dev/null | cut -d, -f6,8,9,20)"; exit 1; fi
log "smoke ok, full swerve run"
$E/tools/guard.sh 12600 ros2 run rover_nav2 benchmark --worlds marsyard slopes rock_field urc_mix --runs 1 --tag 3d_gt_swerve_susp $SW \
  --out $O >> $E/evidence/exp_swerve_susp.out 2>&1 || log "swerve exited $?"; clean
log "swerve: $(cut -d, -f1,6,9 $O/runs.csv | tail -n +2 | tr '\n' ' ')"
A=$B/2026-11-08_3d_gt_free03_susp
for spec in "urc_mix far_south 3" "rock_field far_right 3"; do set -- $spec
  $E/tools/guard.sh 7200 ros2 run rover_nav2 benchmark --worlds $1 --goals $2 --runs $3 --config 3d --tag 3d_gt_free03_susp \
    --launch-args odom_source:=ground_truth suspension:=plugin governor_v_nominal:=0.5 governor_u_free:=0.3 \
    --out $A >> $E/evidence/exp_swerve_susp.out 2>&1 || log "$1 exited $?"; clean
done
log "done: $(cut -d, -f1,6,9 $A/runs.csv | tail -n +2 | tr '\n' ' ')"
