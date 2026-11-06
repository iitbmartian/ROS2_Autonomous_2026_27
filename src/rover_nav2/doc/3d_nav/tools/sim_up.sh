#!/bin/bash
# Start pipeline_3d detached in the isolated sim domain, log to evidence/<phase>/<name>.log, and
# wait until Nav2 is active. Every process carries ROVER_SIM_RUN=<name> for sim_down.sh. A start
# that stalls (Nav2's lifecycle manager waiting forever on a service DDS never discovers, seen in
# sim) is retried once with a fresh stack.
# Usage: sim_up.sh PHASE NAME [launch args...]
E=$HOME/.ros/rover_3d_nav
PHASE=$1; NAME=$2; shift 2
mkdir -p $E/evidence/$PHASE
LOG=$E/evidence/$PHASE/$NAME.log
source $E/tools/simenv.sh
export ROVER_SIM_RUN=$NAME
cd /home/ram/ROS2_Autonomous_2026_27
for attempt in 1 2; do
  setsid nohup ros2 launch rover_bringup pipeline_3d.launch.py gui:=false viz:=false "$@" > $LOG 2>&1 < /dev/null &
  for i in $(seq 1 240); do
    if grep -q "Managed nodes are active" $LOG; then echo "up after ${i}s (attempt $attempt): $LOG"; exit 0; fi
    if grep -q "Failed to bring up" $LOG; then break; fi
    sleep 1
  done
  echo "attempt $attempt did not come up within 240 s, see $LOG"
  cp $LOG $LOG.attempt$attempt
  $E/tools/sim_down.sh "$NAME" > /dev/null
done
exit 1
