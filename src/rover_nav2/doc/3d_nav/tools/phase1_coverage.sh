#!/bin/bash
# Phase 1 gate item 1: near-field ground coverage, roof lidar against the inverted L2, rover at
# rest on flat.sdf. Usage: phase1_coverage.sh
E=$HOME/.ros/rover_3d_nav
source $E/tools/simenv.sh
OUT=$E/evidence/phase1
mkdir -p $OUT
for CFG in "roof lidar_model:=default" "l2_inverted lidar_model:=l2 lidar_inverted:=true"; do
  set -- $CFG; NAME=$1; shift
  $E/tools/sim_down.sh "p1_$NAME" > /dev/null
  $E/tools/sim_up.sh phase1 "p1_$NAME" world:=flat.sdf "$@" || { $E/tools/sim_down.sh "p1_$NAME"; continue; }
  sleep 15
  echo "== $NAME ==" | tee $OUT/coverage_$NAME.txt
  ros2 run rover_nav2 coverage_check --duration 40 2>&1 | grep -vE "WARN|TF_OLD" | tee -a $OUT/coverage_$NAME.txt
  timeout 10 ros2 topic hz /lidar/points 2>&1 | grep "average rate" | tail -1 | tee -a $OUT/coverage_$NAME.txt
  $E/tools/sim_down.sh "p1_$NAME"
done
