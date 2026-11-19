#!/bin/bash
# Bars stall diagnosis (v3 item). bars4 showed the global mask blocks only a ~1 m patch around the
# 12 cm bar, and the planner's paths go around it, yet FollowPath fails 22 times at (1.94, 0.74).
# Need the controller's view at the stall: the local feature map and the local costmap (sampled
# every 15 s by map_sampler.py, not bagged: about 5 MB/s), the global ones every 60 s, and a light
# bag with the plan geometry (/plan), the local plan, commands, BT log and TF.
# v2 install, past_bars_4m, ground truth, BURN=8 like bars4. Goal capped at 600 s wall.
# Args: deadline (epoch), pid to wait for.
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log
log() { echo "$(date '+%F %H:%M') exp_bag_bars5: $*" >> $L; }
while kill -0 $2 2>/dev/null; do [ "$(date +%s)" -ge "$1" ] && { log "deadline while waiting"; exit 1; }; sleep 60; done
[ "$(date +%s)" -ge "$1" ] && { log "deadline before start"; exit 1; }
cd /home/ram/ROS2_Autonomous_2026_27; source /opt/ros/jazzy/setup.bash; source install/setup.bash
source $E/tools/simenv.sh; unset ROVER_COUPLING_K ROVER_COUPLING_D ROVER_COUPLING_MAX
export GZ_PARTITION=rover_bag ROVER_BENCH_RUN=bag_bars5
OUT=$E/bags/run_v2bars5_burn8_$(date +%m%d_%H%M); SMP=$HOME/rover_3d_nav_results/bars5_probe
[ -e $SMP ] && { log "$SMP exists, stopping"; exit 1; }
mkdir -p $E/bags $SMP/local $SMP/global
log "start, bag $OUT, samples $SMP"
cleanup() {
  for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null; done
  python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1; }
trap cleanup EXIT
setsid ros2 launch rover_bringup pipeline_3d.launch.py world:=bars.sdf gui:=false viz:=false \
  odom_source:=ground_truth drive:=ackermann suspension:=plugin governor_v_nominal:=0.5 governor_u_free:=0.3 \
  > $OUT.launch.log 2>&1 &
LP=$!
up=0; for i in $(seq 120); do ros2 lifecycle get /bt_navigator 2>/dev/null | grep -q active && { up=1; break; }; sleep 5; done
[ $up = 1 ] || { log "stack did not come up, see $OUT.launch.log"; exit 1; }
sleep 10
ros2 bag record -o $OUT /odom /cmd_vel_nav /cmd_vel /plan /local_plan /behavior_tree_log /tf /tf_static \
  /speed_limit > $OUT.bag.log 2>&1 &
BP=$!
python3 $E/tools/map_sampler.py $SMP/local --every 15 > $SMP/local.log 2>&1 &
S1=$!
python3 $E/tools/map_sampler.py $SMP/global --every 60 --features /terrain/global_features \
  --costmap /global_costmap/costmap > $SMP/global.log 2>&1 &
S2=$!
sleep 3
BP2=(); for i in $(seq 8); do timeout 900 python3 -c "while True: pass" & BP2+=($!); done
read gx gy gyaw < <(python3 -c "import yaml;g=[x for x in yaml.safe_load(open('src/rover_nav2/rover_nav2/benchmark/goals/bars.yaml'))['goals'] if x['id']=='past_bars_4m'][0];print(g['x'],g['y'],g['yaw'])")
timeout 700 python3 $E/tools/video/send_goal.py $gx $gy $gyaw 600 > $OUT.goal.out 2>&1
kill ${BP2[@]} 2>/dev/null
kill -INT $BP $S1 $S2; for i in $(seq 20); do kill -0 $BP 2>/dev/null || break; sleep 1; done; kill -TERM $BP $S1 $S2 2>/dev/null; wait $BP
kill -INT -$LP; sleep 10; kill -9 -$LP 2>/dev/null
log "done: $(tr '\n' ' ' < $OUT.goal.out | cut -c1-160), $(ls $SMP/local | wc -l) local and $(ls $SMP/global | wc -l) global samples, bag $(du -sh $OUT 2>/dev/null | cut -f1)"
