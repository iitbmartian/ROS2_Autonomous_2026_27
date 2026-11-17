#!/bin/bash
# Bag of swerve v7 + mask_tip_margin 0.05 + start-heading anchor on slopes lane25_top (sw7a 0/3, ends 3.2 to 4.0 m short, 0 recoveries). Is the spin gone, what stops it now? Global costmap
# heading-planned path and governor diagnostics. Args: pid to wait for.
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log
while kill -0 $1 2>/dev/null; do sleep 30; done
source $E/tools/simenv.sh; cd /home/ram/ROS2_Autonomous_2026_27; unset ROVER_COUPLING_K ROVER_COUPLING_D ROVER_COUPLING_MAX
export EXTRA_TOPICS="/terrain_speed_governor/diagnostics /global_costmap/costmap /plan /plan_smoothed"
export EXTRA_LAUNCH="mask_tip_margin:=0.05"
echo "$(date "+%F %H:%M") exp_bag_v7al: start" >> $L
read gx gy gyaw < <(python3 -c "import yaml;g=[x for x in yaml.safe_load(open('src/rover_nav2/rover_nav2/benchmark/goals/slopes.yaml'))['goals'] if x['id']=='lane25_top'][0];print(g['x'],g['y'],g['yaw'])")
BURN=8 $E/tools/guard.sh 1500 $E/tools/bag_run.sh v7alane slopes.sdf $gx $gy $gyaw swerve $E/exp/nav2_3d_swerve_v7.yaml
for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null; done
python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1
b=$(ls -td $E/bags/run_v7alane_* | grep -v '[.][a-z]*$' | head -1)
echo "$(date '+%F %H:%M') exp_bag_v7al: $(tr '\n' ' ' < $b.goal.out 2>/dev/null), bag $b" >> $L
echo "$(date '+%F %H:%M') exp_bag_v7al: done" >> $L
