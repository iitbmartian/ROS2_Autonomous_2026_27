#!/bin/bash
# Bags with global_fine_step:=true of the two weak spots of exp_fs: urc_mix far_south (13 "Failed
# to make progress" before it arrives, with costmaps) and slopes sidehill_uphill (109 to 246 s
# against 38 s plain, with the governor diagnostics). Args: pid to wait for.
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log
while kill -0 $1 2>/dev/null; do sleep 30; done
source $E/tools/simenv.sh; cd /home/ram/ROS2_Autonomous_2026_27; unset ROVER_COUPLING_K ROVER_COUPLING_D ROVER_COUPLING_MAX
export EXTRA_LAUNCH="global_fine_step:=true"
for g in "fsfarsouth urc_mix.sdf 30.0 -20.0 -0.6" "fssidehill slopes.sdf"; do set -- $g
  if [ $1 = fssidehill ]; then
    read gx gy gyaw < <(python3 -c "import yaml;g=[x for x in yaml.safe_load(open('src/rover_nav2/rover_nav2/benchmark/goals/slopes.yaml'))['goals'] if x['id']=='sidehill_uphill'][0];print(g['x'],g['y'],g['yaw'])")
    set -- $1 $2 $gx $gy $gyaw
    export EXTRA_TOPICS="/terrain_speed_governor/diagnostics"
  else export EXTRA_TOPICS="/global_costmap/costmap /terrain_speed_governor/diagnostics"; fi
  BURN=8 $E/tools/guard.sh 1800 $E/tools/bag_run.sh $1 $2 $3 $4 $5 ackermann
  for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null; done
  python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1
  b=$(ls -td $E/bags/run_$1_* | grep -v '[.][a-z]*$' | head -1)
  echo "$(date '+%F %H:%M') exp_bag_fs: $1 $(tr '\n' ' ' < $b.goal.out 2>/dev/null), bag $b" >> $L
done
echo "$(date '+%F %H:%M') exp_bag_fs: done" >> $L
