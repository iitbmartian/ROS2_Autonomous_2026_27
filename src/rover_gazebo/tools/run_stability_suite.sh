#!/bin/bash
# Launch a headless sim per world, run stability_test.py's suite against it, shut it down.
#
#   ros2 run rover_gazebo run_stability_suite.sh            # flat, ledge, bars
#   ros2 run rover_gazebo run_stability_suite.sh flat.sdf   # just one world
#
# Needs this workspace (and nothing else providing rover_gazebo) sourced. Results land in
# $OUT_DIR (default /tmp/rover_stability), one JSON + sim log per world. Exits non-zero
# if any world's suite failed.

OUT_DIR=${OUT_DIR:-/tmp/rover_stability}
WORLDS=("$@")
[ ${#WORLDS[@]} -eq 0 ] && WORLDS=(flat.sdf ledge.sdf bars.sdf)
mkdir -p "$OUT_DIR"
status=0

for world in "${WORLDS[@]}"; do
  suite=obstacle
  [ "$world" = "flat.sdf" ] && suite=flat
  log="$OUT_DIR/${world%.sdf}_sim.log"
  echo "=== $world ($suite suite) ==="
  # Own process group, so shutdown can take every node with it: killing only the
  # launch process orphans its nodes, and an orphaned rocker_coupling_node keeps
  # publishing torque into the next world's sim.
  setsid ros2 launch rover_bringup rover_sim.launch.py gui:=false world:="$world" >"$log" 2>&1 &
  launch_pid=$!

  # Wait for the PID controllers to be loaded (spawned last, --inactive).
  ready=0
  for _ in $(seq 1 90); do
    if ros2 service call /controller_manager/list_controllers \
        controller_manager_msgs/srv/ListControllers 2>/dev/null \
        | grep -q "wheel_pid_controller"; then ready=1; break; fi
    sleep 1
  done
  if [ $ready -eq 1 ]; then
    ros2 run rover_gazebo stability_test.py --suite "$suite" \
      --out "$OUT_DIR/${world%.sdf}.json" || status=1
  else
    echo "sim for $world never became ready; see $log"; status=1
  fi

  kill -INT -- -$launch_pid 2>/dev/null
  for _ in $(seq 1 15); do kill -0 -- -$launch_pid 2>/dev/null || break; sleep 1; done
  kill -9 -- -$launch_pid 2>/dev/null
  sleep 2
done

exit $status
