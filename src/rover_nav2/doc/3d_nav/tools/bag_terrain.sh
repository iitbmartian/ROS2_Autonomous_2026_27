#!/bin/bash
# After the v5 ledge runner exits: one rock_field ahead_10m run with the terrain grids bagged, to
# find what puts 1 m lethal discs on flat ground. Then 3D v5 on bars to complete v5.
E=$HOME/.ros/rover_3d_nav; B=$HOME/.ros/rover_benchmark; L=$HOME/rover_3d_nav_results/batch.log
while pgrep -f "rover_nav2/benchmark --worlds" > /dev/null; do sleep 20; done
sleep 20
OUT=$E/bags/rock_terrain_$(date +%H%M)
source $E/tools/simenv.sh
export GZ_PARTITION=rover_bag
cd /home/ram/ROS2_Autonomous_2026_27
setsid ros2 launch rover_bringup pipeline_3d.launch.py world:=rock_field.sdf gui:=false viz:=false \
  x:=0.0 y:=0.0 z:=0.02 yaw:=0.0 odom_source:=ground_truth > $E/bags/rock_terrain_launch.log 2>&1 &
LP=$!
for i in $(seq 120); do ros2 lifecycle get /bt_navigator 2>/dev/null | grep -q active && break; sleep 5; done
ros2 bag record -s mcap -o $OUT /terrain/local /terrain/local_features /terrain/global \
  /terrain/global_features /terrain/hazard_points /plan /global_costmap/costmap \
  /local_costmap/costmap /odom /cmd_vel_nav /cmd_vel /tf /tf_static > $E/bags/rock_terrain_bag.log 2>&1 &
BP=$!
sleep 3
timeout 900 python3 $E/tools/video/send_goal.py 10.0 0.0 0.0 400 > $E/bags/rock_terrain_goal.out 2>&1
kill -INT $BP; sleep 5; kill -INT $BP 2>/dev/null; wait $BP
kill -INT -$LP; sleep 10; kill -9 -$LP 2>/dev/null
echo "$(date '+%F %H:%M') terrain bag $OUT: $(tail -1 $E/bags/rock_terrain_goal.out)" >> $L
export GZ_PARTITION=rover3d
source $E/tools/run_fn.sh
run 3d 3d_gt_v5 2026-11-05_3d_gt_v5 1 bars
python3 $E/tools/fig_benchmark.py check_3d_gt_v5 $B/2026-11-05_3d_gt_v5 3d_gt_v5=3d >> $L 2>&1
echo "$(date '+%F %H:%M') v5 complete" >> $L
