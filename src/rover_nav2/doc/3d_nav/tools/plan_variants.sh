#!/bin/bash
# One 2d_bounded stack on a world, the planner probe under several Smac settings set live.
source $HOME/.ros/rover_3d_nav/tools/simenv.sh
W=${1:-urc_mix}
PF=/home/ram/ROS2_Autonomous_2026_27/install/rover_nav2/share/rover_nav2/config/nav2_params_sim_2d_bounded.yaml
LOG=$HOME/.ros/rover_3d_nav/evidence/plan_variants_$W.log
setsid ros2 launch rover_bringup pipeline.launch.py world:=$W.sdf gui:=false viz:=false \
  nav_params_file:=$PF database_path:=/tmp/claude-1000/probe.db delete_db_on_start:=true > $LOG 2>&1 &
LP=$!
for i in $(seq 1 60); do grep -q "Managed nodes are active" $LOG && break; sleep 5; done
sleep 20
P=$HOME/.ros/rover_3d_nav/tools/plan_probe.py
set_p() { ros2 param set /planner_server "$1" "$2" > /dev/null; }
echo "== default (2 s)"; python3 $P 20.0 42.0
echo "== max_planning_time 10"; set_p GridBased.max_planning_time 10.0; python3 $P 20.0 42.0
echo "== 10 s + cache_obstacle_heuristic"; set_p GridBased.cache_obstacle_heuristic true; python3 $P 20.0 42.0
echo "== 2 s + downsample x2 + cache"; set_p GridBased.max_planning_time 2.0; set_p GridBased.downsample_costmap true; set_p GridBased.downsampling_factor 2; python3 $P 20.0 42.0
echo "== 2 s + downsample x2, 36 angle bins"; set_p GridBased.angle_quantization_bins 36; python3 $P 20.0 42.0
kill -INT -$LP; sleep 15; kill -9 -$LP 2>/dev/null
echo variants done
