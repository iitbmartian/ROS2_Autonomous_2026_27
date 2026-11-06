#!/bin/bash
# Parallel benchmark on one machine: N lanes, each a runner with its own ROVER_BENCH_INSTANCE (domain
# 42 + i, own Gazebo partition) and own out dir, worlds dealt round-robin. Afterwards merge_runs.py
# joins the lanes into OUT. Usage:
#   farm_run.sh N CAP_S OUT "world1 world2 ..." <benchmark args, e.g. --runs 3 --config 3d --tag T --launch-args ...>
# Each lane is capped at CAP_S by guard.sh.
E=$HOME/.ros/rover_3d_nav; B=$HOME/.ros/rover_benchmark
N=$1; CAP=$2; OUT=$3; WORLDS=($4); shift 4
pids=()
for ((i = 0; i < N; i++)); do
  lane=()
  for ((k = i; k < ${#WORLDS[@]}; k += N)); do lane+=("${WORLDS[k]}"); done
  [ ${#lane[@]} -eq 0 ] && continue
  (
    export ROVER_BENCH_INSTANCE=$((i + 1))
    source $E/tools/simenv.sh
    $E/tools/guard.sh $CAP ros2 run rover_nav2 benchmark --worlds "${lane[@]}" "$@" --out $B/${OUT}_lane$i \
      > $B/${OUT}_lane$i.out 2>&1
    echo "lane $i (${lane[*]}) exited $?" >> $B/${OUT}_farm.log
  ) &
  pids+=($!)
  sleep 20   # stagger starts, a dozen stacks starting at once is the worst case for discovery
done
wait "${pids[@]}"
python3 $E/tools/merge_runs.py $B/$OUT $B/${OUT}_lane*/ >> $B/${OUT}_farm.log 2>&1
