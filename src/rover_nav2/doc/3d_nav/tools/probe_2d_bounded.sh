#!/bin/bash
# Starts a 2d_bounded stack on a world, waits for Nav2, snapshots the costmaps, stops it.
source $HOME/.ros/rover_3d_nav/tools/simenv.sh
W=${1:-urc_mix}
PF=/home/ram/ROS2_Autonomous_2026_27/install/rover_nav2/share/rover_nav2/config/nav2_params_sim_2d_bounded.yaml
LOG=$HOME/.ros/rover_3d_nav/evidence/probe_2d_bounded_$W.log
setsid ros2 launch rover_bringup pipeline.launch.py world:=$W.sdf gui:=false viz:=false \
  nav_params_file:=$PF database_path:=/tmp/claude-1000/probe.db delete_db_on_start:=true > $LOG 2>&1 &
LP=$!
for i in $(seq 1 60); do grep -q "Managed nodes are active" $LOG && break; sleep 5; done
sleep 30
python3 $HOME/.ros/rover_3d_nav/tools/snapshot.py p2b_2d_bounded_$W $W
kill -INT -$LP; sleep 15; kill -9 -$LP 2>/dev/null
echo probe done
