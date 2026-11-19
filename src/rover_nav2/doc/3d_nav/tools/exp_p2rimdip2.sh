#!/bin/bash
# p2rimdip2: the crater test with every ram/3d_nav-p26 fix and the tip guard. The driver backs 2.5 m after a touch
# (1.5 m in p2rimdip): the dip detector flags the touched rim, and 1.5 m left the rover in its lethal buffer.
# Phase 2 crater item: every crater wider than the track flagged negative_obstacle before the
# footprint reaches it, on crater_field. The 7 Nov run was vacuous (the 3D stack drove around all
# 24, "0 of 0"), so rim_approach.py drives the footprint onto each rim blind after a Nav2 standoff,
# while terrain_eval scores flag time against reach time. v2 install, ground-truth odometry.
# Args: deadline (epoch), pid to wait for.
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log; OUT=$E/evidence/phase2
log() { echo "$(date '+%F %H:%M') exp_p2rimdip2: $*" >> $L; }
clean() { for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null; done
  python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1; }
while kill -0 $2 2>/dev/null; do [ "$(date +%s)" -ge "$1" ] && { log "deadline while waiting"; exit 1; }; sleep 60; done
cd /home/ram/ROS2_Autonomous_2026_27; source /opt/ros/jazzy/setup.bash; source install/setup.bash; source $HOME/ws_p26/install/setup.bash
[ "$(ros2 pkg prefix rover_terrain_mapping)" = "$HOME/ws_p26/install/rover_terrain_mapping" ] || { log "p26 overlay not active, stopping"; exit 1; }
clean
source $E/tools/simenv.sh; unset ROVER_COUPLING_K ROVER_COUPLING_D ROVER_COUPLING_MAX
[ "$(date +%s)" -ge "$1" ] && { log "deadline before start"; exit 1; }
mkdir -p $OUT; S=$(date +%m%d_%H%M); CSV=$OUT/eval_crater_field_rimdip2_$S.csv; DRV=$OUT/rim_approachdip2_$S.log
log "start (install of $(git rev-parse --short HEAD), p26 overlay $(git -C $HOME/ws_p26 rev-parse --short HEAD)), out $CSV"
N=p2rimdip2_$S
trap '$E/tools/sim_down.sh $N > /dev/null 2>&1; clean' EXIT
$E/tools/sim_up.sh phase2 $N world:=crater_field.sdf odom_source:=ground_truth suspension:=plugin \
  governor_v_nominal:=0.5 governor_u_free:=0.3 nav_extra_overlay:=$HOME/.ros/rover_3d_nav/exp/nav2_3d_tipguard.yaml >> $DRV 2>&1 || { log "stack did not come up, see $E/evidence/phase2/$N.log"; exit 1; }
sleep 10
grep -rqs "observed dips on" $E/evidence/phase2/$N.log $HOME/.ros/log/latest/ 2>/dev/null || log "warning: no observed dips on line found yet"
CR=$(ros2 pkg prefix rover_gazebo)/share/rover_gazebo/models/TerrainCraterField/craters.yaml
ros2 run rover_nav2 terrain_eval --world crater_field --out $CSV --duration 100000 --craters $CR > ${CSV%.csv}.out 2>&1 &
EVAL=$!
$E/tools/guard.sh 7200 python3 $E/tools/rim_approach.py $CR $DRV --reverse 2.5 || log "rim_approach exited $?"
# ros2 run does not forward SIGINT, so signal terrain_eval itself, then force after 30 s.
pkill -INT -f "lib/rover_nav2/terrain_eval --world crater_field"
for i in $(seq 30); do kill -0 $EVAL 2>/dev/null || break; sleep 1; done
pkill -KILL -f "lib/rover_nav2/terrain_eval --world crater_field"; wait $EVAL 2>/dev/null
log "$(tail -1 $DRV)"
log "$(grep -E 'RMSE|reached craters flagged first' ${CSV%.csv}.txt | tr '\n' ' ')"
log "done"
