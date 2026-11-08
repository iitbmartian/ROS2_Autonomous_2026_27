#!/bin/bash
# After exp_swerve_susp: rebuild rover_traversability and rover_nav2 (approach-from-behind BT),
# smoke the far_south fix, then the night: 3D gate rerun and 2D baseline rerun, both on the plugin
# suspension, then swerve x2 with what time is left. No stage starts with under 1 h before $2.
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log; B=$HOME/.ros/rover_benchmark; R=$HOME/rover_3d_nav_results
DL=$1; LAST=$2
log() { echo "$(date '+%F %H:%M') exp_tonight: $*" >> $L; }
clean() { for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null; done
  python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1; }
left() { echo $(( LAST - $(date +%s) )); }
n0=$(wc -l < $L)
until tail -n +$((n0+1)) $L | grep -q "exp_swerve_susp: done\|exp_swerve_susp: smoke failed\|exp_swerve_susp: deadline"; do
  [ "$(date +%s)" -ge "$DL" ] && { log "deadline before exp_swerve_susp finished, nothing run"; exit 1; }; sleep 30; done
cd /home/ram/ROS2_Autonomous_2026_27
if ! bash -c 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && colcon build --packages-select rover_traversability rover_nav2 --cmake-args -DCMAKE_BUILD_TYPE=Release' > $E/evidence/exp_tonight_build.log 2>&1; then
  log "build failed, see evidence/exp_tonight_build.log"; exit 1; fi
source $E/tools/simenv.sh
log "start ($(git rev-parse --short HEAD))"
AK="--config 3d --launch-args odom_source:=ground_truth suspension:=plugin governor_v_nominal:=0.5 governor_u_free:=0.3"
S=$B/2026-11-08_3d_gt_approach_bt
for spec in "urc_mix far_south 2" "marsyard behind_left 2"; do set -- $spec
  $E/tools/guard.sh 5400 ros2 run rover_nav2 benchmark --worlds $1 --goals $2 --runs $3 --tag 3d_gt_approach_bt $AK --out $S >> $E/evidence/exp_tonight.out 2>&1 || log "$1 exited $?"; clean
done
log "approach smoke: $(cut -d, -f1,6,8,9 $S/runs.csv | tail -n +2 | tr '\n' ' ')"
if [ "$(awk -F, 'NR>1 && $6=="startup_failed"' $S/runs.csv | wc -l)" -ge 2 ]; then log "approach BT stack does not start, night runs skipped"; exit 1; fi
if [ $(left) -gt 3600 ]; then
  G=$B/2026-11-08_3d_gt_gate_susp
  $E/tools/guard.sh $(left) ros2 run rover_nav2 benchmark --worlds marsyard crater_field slopes urc_mix rock_field bars --runs 3 --tag 3d_gt_gate_susp $AK --out $G >> $E/evidence/exp_tonight.out 2>&1 || log "3D gate exited $?"; clean
  python3 $E/tools/phase3_gate.py $G/runs.csv $B/2026-11-07_2d_base/runs.csv $R/analysis/phase3_gate_susp_vs_2d_base.csv > $E/evidence/phase3_gate_susp.txt 2>&1
  log "3D gate on plugin suspension: $(cut -d, -f6 $G/runs.csv | tail -n +2 | sort | uniq -c | tr '\n' ' ')"
fi
if [ $(left) -gt 3600 ]; then
  D=$B/2026-11-09_2d_base_susp
  $E/tools/guard.sh $(left) ros2 run rover_nav2 benchmark --worlds marsyard crater_field slopes urc_mix rock_field bars --runs 3 --config 2d \
    --params-file /home/ram/ROS2_Autonomous_2026_27/src/rover_nav2/config/nav2_params_sim_2d_bounded.yaml --tag 2d_base_susp \
    --launch-args odom_source:=ground_truth suspension:=plugin --out $D >> $E/evidence/exp_tonight.out 2>&1 || log "2D exited $?"; clean
  [ -f $B/2026-11-08_3d_gt_gate_susp/runs.csv ] && python3 $E/tools/phase3_gate.py $B/2026-11-08_3d_gt_gate_susp/runs.csv $D/runs.csv $R/analysis/phase3_gate_susp_vs_2d_susp.csv > $E/evidence/phase3_gate_susp_vs_2d_susp.txt 2>&1
  log "2D on plugin suspension: $(cut -d, -f6 $D/runs.csv | tail -n +2 | sort | uniq -c | tr '\n' ' ')"
fi
if [ $(left) -gt 3600 ]; then
  W=$B/2026-11-09_3d_gt_swerve_susp_x2
  $E/tools/guard.sh $(left) ros2 run rover_nav2 benchmark --worlds marsyard slopes rock_field urc_mix --runs 2 --config 3d --tag 3d_gt_swerve_susp_x2 \
    --launch-args odom_source:=ground_truth drive:=swerve suspension:=plugin governor_v_nominal:=0.5 governor_u_free:=0.3 \
    nav_extra_overlay:=$E/exp/nav2_3d_swerve_v2.yaml --out $W >> $E/evidence/exp_tonight.out 2>&1 || log "swerve exited $?"; clean
fi
log "done"
