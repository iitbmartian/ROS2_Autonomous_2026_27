#!/bin/bash
# One slopes lane25_top run with global_fine_step and the terrain topics recorded, for an offline
# replay of the global features node (fine step on and off) to see what the fine step adds on the
# lanes. lane25 is 0/6 with the fix (safety cancels at about 20 s). Args: pid to wait for.
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log
while kill -0 $1 2>/dev/null; do sleep 20; done
source $E/tools/simenv.sh; cd /home/ram/ROS2_Autonomous_2026_27; unset ROVER_COUPLING_K ROVER_COUPLING_D ROVER_COUPLING_MAX
read gx gy gyaw < <(python3 -c "import yaml;g=[x for x in yaml.safe_load(open('src/rover_nav2/rover_nav2/benchmark/goals/slopes.yaml'))['goals'] if x['id']=='lane25_top'][0];print(g['x'],g['y'],g['yaw'])")
export EXTRA_LAUNCH="global_fine_step:=true"
export EXTRA_TOPICS="/terrain/global /terrain/global_features /terrain/local_features /global_costmap/costmap /tf /terrain_speed_governor/diagnostics"
BURN=8 $E/tools/guard.sh 900 $E/tools/bag_run.sh slope25 slopes.sdf $gx $gy $gyaw ackermann
for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null; done
python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1
b=$(ls -td $E/bags/run_slope25_* | grep -v '[.][a-z]*$' | head -1)
echo "$(date '+%F %H:%M') exp_bag_slope: $(tr '\n' ' ' < $b.goal.out 2>/dev/null), bag $b $(du -sh $b | cut -f1)" >> $L
