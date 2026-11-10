#!/bin/bash
# The D 200 spin check really failed (e1 6.6, e2 5.2 deg, the chain read a crashed script as 0).
# D 500 kept it at 1.8 deg but shakes the body. Loaded swerve v4 spin bags at D 300 and D 250 find
# the smallest damping that keeps residuals under 5 deg. Args: deadline (epoch), pid to wait for.
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log; X=$E/exp
log() { echo "$(date '+%F %H:%M') exp_spin_d300: $*" >> $L; }
clean() { for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null; done
  python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1; }
while kill -0 $2 2>/dev/null; do [ "$(date +%s)" -ge "$1" ] && { log "deadline while waiting"; exit 1; }; sleep 60; done
source $E/tools/simenv.sh; cd /home/ram/ROS2_Autonomous_2026_27
export ROVER_COUPLING_K=30000 ROVER_COUPLING_MAX=3000
for d in 300 250; do [ "$(date +%s)" -ge "$1" ] && { log "deadline"; exit 1; }
  export ROVER_COUPLING_D=$d
  BURN=8 $E/tools/guard.sh 1800 $E/tools/bag_run.sh swerve_v4_d$d marsyard.sdf 6.0 5.0 0.7 swerve $X/nav2_3d_swerve_v4.yaml; clean
  bag=$(ls -td $E/bags/run_swerve_v4_d${d}_burn8_* | grep -v '\.' | head -1)
  # The bag can still be closing, retry until the script prints its result line.
  for k in 1 2 3 4 5 6; do r=$(python3 $E/tools/rocker_residuals.py $bag 2>&1 | head -1)
    echo "$r" | grep -q "max residual" && break; sleep 20; done
  log "D $d: $r | goal: $(tr '\n' ' ' < $bag.goal.out)"
done
log "done"
