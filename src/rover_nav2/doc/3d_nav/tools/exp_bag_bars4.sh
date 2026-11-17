#!/bin/bash
# Bag of v2 (heading planner + mask_tip_margin 0.05) on bars past_bars_4m with the global feature map
# (elevation, variance, step, safe_yaw_mask). The planner aborts 0.7 m short on "no safe heading",
# but on the ground-truth heightmap the mask allows the bar (probe: margin 0.315 > 0.22). Args: pid to wait for.
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log
while kill -0 $1 2>/dev/null; do sleep 30; done
source $E/tools/simenv.sh; cd /home/ram/ROS2_Autonomous_2026_27; source install/setup.bash; unset ROVER_COUPLING_K ROVER_COUPLING_D ROVER_COUPLING_MAX
echo "$(date '+%F %H:%M') exp_bag_bars4: start" >> $L
export EXTRA_TOPICS="/terrain/global_features /global_costmap/costmap /plan_smoothed"
export EXTRA_LAUNCH="global_fine_step:=true mask_tip_margin:=0.05"
read gx gy gyaw < <(python3 -c "import yaml;g=[x for x in yaml.safe_load(open('src/rover_nav2/rover_nav2/benchmark/goals/bars.yaml'))['goals'] if x['id']=='past_bars_4m'][0];print(g['x'],g['y'],g['yaw'])")
BURN=8 $E/tools/guard.sh 1500 $E/tools/bag_run.sh v2bars4 bars.sdf $gx $gy $gyaw ackermann $E/exp/nav2_3d_ack_heading.yaml
for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null; done
python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1
echo "$(date '+%F %H:%M') exp_bag_bars4: done" >> $L
