#!/bin/bash
# Bag of Ackermann with the HeadingAwarePlanner on slopes lane25_top. In exp_ackh r2 it climbed
# aligned (roll 2.3, pitch 24.8) and then aborted on "Costmap timed out" near the top, with
# terrain_mapping logging an empty source frame. Records the plan, global costmap, governor.
# Args: pid to wait for.
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log
while kill -0 $1 2>/dev/null; do sleep 30; done
source $E/tools/simenv.sh; cd /home/ram/ROS2_Autonomous_2026_27; unset ROVER_COUPLING_K ROVER_COUPLING_D ROVER_COUPLING_MAX
export EXTRA_TOPICS="/terrain_speed_governor/diagnostics /global_costmap/costmap /plan /plan_smoothed"
export EXTRA_LAUNCH="global_fine_step:=true"
read gx gy gyaw < <(python3 -c "import yaml;g=[x for x in yaml.safe_load(open('src/rover_nav2/rover_nav2/benchmark/goals/slopes.yaml'))['goals'] if x['id']=='lane25_top'][0];print(g['x'],g['y'],g['yaw'])")
BURN=8 $E/tools/guard.sh 1500 $E/tools/bag_run.sh ackhlane slopes.sdf $gx $gy $gyaw ackermann $E/exp/nav2_3d_ack_heading.yaml
for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null; done
python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1
b=$(ls -td $E/bags/run_ackhlane_* | grep -v '[.][a-z]*$' | head -1)
echo "$(date '+%F %H:%M') exp_bag_ackl: $(tr '\n' ' ' < $b.goal.out 2>/dev/null), bag $b" >> $L
echo "$(date '+%F %H:%M') exp_bag_ackl: done" >> $L
