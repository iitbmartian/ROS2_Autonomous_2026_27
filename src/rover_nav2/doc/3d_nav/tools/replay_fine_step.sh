#!/bin/bash
# Offline replay of a bag with /terrain/global, /terrain/local_features and /tf through two global
# terrain_features_node instances, fine step off and on, then compares their step layers
# (replay_fine_step_cmp.py). No simulator. Own ROS domain so a running benchmark is untouched.
# Args: bag dir, out prefix (png and txt).
B=$1; OUT=$2
source /opt/ros/jazzy/setup.bash; source /home/ram/ROS2_Autonomous_2026_27/install/setup.bash
export ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1
T=$HOME/.ros/rover_3d_nav/tools
common="-p use_sim_time:=true -p input_topic:=/terrain/global -p sensor_frame:=lidar3d_link"
ros2 run rover_terrain_mapping terrain_features_node --ros-args -r __node:=feat_off $common \
  -p output_topic:=/replay/off -p fine_step_topic:="" > $OUT.off.log 2>&1 & P1=$!
ros2 run rover_terrain_mapping terrain_features_node --ros-args -r __node:=feat_on $common \
  -p output_topic:=/replay/on -p fine_step_topic:=/terrain/local_features > $OUT.on.log 2>&1 & P2=$!
python3 $T/replay_fine_step_cmp.py $OUT > $OUT.txt 2>&1 & P3=$!
sleep 4
ros2 bag play $B --clock --topics /terrain/global /terrain/local_features /tf /tf_static > $OUT.play.log 2>&1
sleep 3; kill -INT $P3; wait $P3; kill -INT $P1 $P2; sleep 2; kill -9 $P1 $P2 2>/dev/null
cat $OUT.txt
