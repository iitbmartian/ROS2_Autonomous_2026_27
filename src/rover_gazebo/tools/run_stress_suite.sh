#!/bin/bash
# Stress-test the controls: run each stress suite in its own headless sim, then re-run a
# few showcase scenarios with a full Gazebo state log + rosbag + plot, for watching later.
#
#   ros2 run rover_gazebo run_stress_suite.sh                    # every suite + showcases
#   ros2 run rover_gazebo run_stress_suite.sh slopes rocks       # just these suites
#   RECORD= ros2 run rover_gazebo run_stress_suite.sh abuse      # no recordings
#   RECORD="rocks:explicit_pid_rocks_dense" ros2 run rover_gazebo run_stress_suite.sh none
#
# Suites (see rover_gazebo/stress_suites.py): abuse slopes side_slope rocks washboard
# friction rough. "none" runs no suites, only the recordings.
#
# Results land in $OUT_DIR (default ./stress_runs/<timestamp>): <suite>.json/.out/_sim.log
# per suite, and recordings/<suite>__<scenario>/{gzlog,bag,plot.png,result.out} per
# showcase. doc/STRESS_TESTING.md explains how to play them back. Exits non-zero if any
# suite had a failed check.
#
# Every sim runs in its own ROS domain and Gazebo partition, localhost only: a second
# sim anywhere on the LAN on the default domain otherwise spawns its rover into this one
# and corrupts the run (seen while writing this).

OUT_DIR=${OUT_DIR:-$PWD/stress_runs/$(date +%Y%m%d_%H%M%S)}
ALL_SUITES=(abuse slopes side_slope rocks washboard friction rough)
SUITES=("$@")
[ ${#SUITES[@]} -eq 0 ] && SUITES=("${ALL_SUITES[@]}")
[ "${SUITES[0]}" = "none" ] && SUITES=()
# Three on purpose: enough to see the controls working, small on disk (no camera or
# lidar topics are recorded; see BAG_TOPICS).
# One working well, two showing open findings (doc/STRESS_TESTING.md).
RECORD=${RECORD-"rough:explicit_pid_rough_y+0 slopes:explicit_pid_hold_on_20 \
abuse:abuse_explicit_aim_clamp_then_ackermann"}

declare -A WORLD=([abuse]=flat [slopes]=slopes [side_slope]=side_slope [rocks]=rocks
                  [washboard]=washboard [friction]=low_friction [rough]=rough_terrain)

export ROS_DOMAIN_ID=${STRESS_DOMAIN_ID:-73}
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_PARTITION=rover_stress_$$
export GZ_IP=127.0.0.1

# /joint_states and /tf come in at 1 kHz and were ~90% of a 50 MB bag; bag_throttle.py
# relays them at 100 Hz as /rec/joint_states and /rec/tf (remap them back on playback).
BAG_TOPICS=(/clock /cmd_vel /rover/steer_mode /rover/steer_aim /odom /imu/data
            /rec/joint_states /rec/tf /tf_static /robot_description
            /steer_controller/commands /wheel_controller/commands
            /explicit_pid_control/steer_target /explicit_pid_control/wheel_target
            /steer_pid_controller/commands /wheel_pid_controller/commands)

mkdir -p "$OUT_DIR"
echo "results -> $OUT_DIR (ROS_DOMAIN_ID=$ROS_DOMAIN_ID, GZ_PARTITION=$GZ_PARTITION)"
status=0
launch_pid=""

start_sim() {   # world log [record_path]
  # record_path only when recording: ros2 launch rejects an empty "name:=".
  setsid ros2 launch rover_gazebo rover_sim.launch.py gui:=false world:="$1.sdf" \
    ${3:+record_path:="$3"} >"$2" 2>&1 &
  launch_pid=$!
  # Ready once the PID controllers (spawned last, --inactive) are loaded.
  for _ in $(seq 1 120); do
    if ros2 service call /controller_manager/list_controllers \
        controller_manager_msgs/srv/ListControllers 2>/dev/null \
        | grep -q "wheel_pid_controller"; then return 0; fi
    sleep 1
  done
  return 1
}

stop_group() {   # pid -- SIGINT the whole process group, then make sure
  [ -z "$1" ] && return
  kill -INT -- -"$1" 2>/dev/null
  for _ in $(seq 1 20); do kill -0 -- -"$1" 2>/dev/null || break; sleep 1; done
  kill -9 -- -"$1" 2>/dev/null
  sleep 2
}

trap 'stop_group "$bag_pid"; stop_group "$throttle_pid"; stop_group "$launch_pid"; exit 130' INT TERM

# -- 1. the suites -------------------------------------------------------------
for suite in "${SUITES[@]}"; do
  world=${WORLD[$suite]}
  [ -z "$world" ] && { echo "unknown suite $suite"; status=1; continue; }
  echo "=== $suite ($world.sdf) ==="
  if start_sim "$world" "$OUT_DIR/${suite}_sim.log"; then
    ros2 run rover_gazebo stability_test.py --suite "$suite" --world "$world" \
      --out "$OUT_DIR/$suite.json" | tee "$OUT_DIR/$suite.out"
    [ "${PIPESTATUS[0]}" -ne 0 ] && status=1
  else
    echo "sim for $world never became ready; see $OUT_DIR/${suite}_sim.log"; status=1
  fi
  stop_group "$launch_pid"; launch_pid=""
done

# -- 2. recorded showcases -----------------------------------------------------
for item in $RECORD; do
  suite=${item%%:*}; scenario=${item#*:}; world=${WORLD[$suite]}
  dir="$OUT_DIR/recordings/${suite}__${scenario}"
  mkdir -p "$dir"
  echo "=== recording $suite / $scenario ==="
  if ! start_sim "$world" "$dir/sim.log" "$dir/gzlog"; then
    echo "sim never became ready; see $dir/sim.log"; stop_group "$launch_pid"; continue
  fi
  setsid ros2 run rover_gazebo bag_throttle.py >"$dir/throttle.log" 2>&1 &
  throttle_pid=$!
  # A background job in a non-interactive bash starts with SIGINT ignored, and unlike
  # ros2 launch the recorder never un-ignores it, so it would sit through stop_group's
  # SIGINT, get SIGKILLed and leave an unfinalized, unreadable mcap. Re-arm it first.
  setsid python3 -c 'import os, signal, sys; signal.signal(signal.SIGINT, signal.SIG_DFL); \
os.execvp("ros2", ["ros2"] + sys.argv[1:])' bag record -s mcap \
    --storage-preset-profile zstd_fast -o "$dir/bag" "${BAG_TOPICS[@]}" \
    >"$dir/bag.log" 2>&1 &
  bag_pid=$!
  sleep 3
  ros2 run rover_gazebo stability_test.py --suite "$suite" --world "$world" \
    --only "$scenario" --out "$dir/result.json" >"$dir/result.out" 2>&1
  tail -n 4 "$dir/result.out"
  stop_group "$bag_pid"; bag_pid=""
  stop_group "$throttle_pid"; throttle_pid=""
  stop_group "$launch_pid"; launch_pid=""
  ros2 run rover_gazebo plot_run.py "$dir/bag" "$dir/plot.png" --title "$suite / $scenario" \
    || echo "plot failed for $dir"
done

echo "done -> $OUT_DIR"
exit $status
