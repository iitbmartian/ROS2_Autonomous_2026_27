#!/bin/bash
# run_remote.sh user@host cap_s out_dir config tag runs "worlds" [extra runner args]: start a
# guarded benchmark on the host, detached, one runner per world. fetch_remote.sh copies results.
H=$1; CAP=$2; OUT=$3; CFG=$4; TAG=$5; RUNS=$6; WORLDS=$7; shift 7; EXTRA="$*"
ssh $H "setsid nohup bash -c '
  source ~/.ros/rover_3d_nav/tools/simenv.sh
  for w in $WORLDS; do
    ~/.ros/rover_3d_nav/tools/guard.sh $CAP ros2 run rover_nav2 benchmark --worlds \$w --runs $RUNS \
      --config $CFG --tag $TAG --launch-args odom_source:=ground_truth --out ~/.ros/rover_benchmark/$OUT $EXTRA
    for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN \$p/environ 2>/dev/null && kill -9 \${p#/proc/}; done
  done; echo done > ~/.ros/rover_benchmark/$OUT.done' > ~/.ros/rover_3d_nav/evidence/$OUT.out 2>&1 < /dev/null &"
echo "started $TAG on $H: $WORLDS"
