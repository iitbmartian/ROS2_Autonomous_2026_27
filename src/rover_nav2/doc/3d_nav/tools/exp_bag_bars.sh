#!/bin/bash
# Bag of the v2 candidate (heading planner + mask_tip_margin 0.05 + crest fix) on bars past_bars_6m.
# In exp_ackfr it stalled 3/3 at the first bar (4.1 m short, "Failed to make progress"), v1.1 crosses
# in about 20 s. Records the plans, the local and global costmaps and the command chain. Args: pid to wait for.
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log
while kill -0 $1 2>/dev/null; do sleep 30; done
source $E/tools/simenv.sh; cd /home/ram/ROS2_Autonomous_2026_27; source install/setup.bash; unset ROVER_COUPLING_K ROVER_COUPLING_D ROVER_COUPLING_MAX
echo "$(date '+%F %H:%M') exp_bag_bars: start" >> $L
export EXTRA_TOPICS="/global_costmap/costmap /local_costmap/costmap /plan_smoothed /terrain_speed_governor/diagnostics"
export EXTRA_LAUNCH="global_fine_step:=true mask_tip_margin:=0.05"
read gx gy gyaw < <(python3 -c "import yaml;g=[x for x in yaml.safe_load(open('src/rover_nav2/rover_nav2/benchmark/goals/bars.yaml'))['goals'] if x['id']=='past_bars_6m'][0];print(g['x'],g['y'],g['yaw'])")
BURN=8 $E/tools/guard.sh 1500 $E/tools/bag_run.sh v2bars bars.sdf $gx $gy $gyaw ackermann $E/exp/nav2_3d_ack_heading.yaml
for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null; done
python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1
echo "$(date '+%F %H:%M') exp_bag_bars: done" >> $L
