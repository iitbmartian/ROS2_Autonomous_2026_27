#!/bin/bash
# Governor speed experiment, standard 3D overlay, 1 run per goal on marsyard, crater_field,
# rock_field, slopes: A) v_nominal 0.5 (the MPPI vx_max, as 2D drives) instead of 0.25,
# B) v_nominal 0.5 with u_free 0.3 (full speed on easy ground). Installs the u_free governor
# first, while no stack runs. Starts after exp_mppi_track ends or gives up at $1.
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log; B=$HOME/.ros/rover_benchmark
log() { echo "$(date '+%F %H:%M') exp_vnom: $*" >> $L; }
until grep -q "exp_mppi_track: done\|exp_mppi_track: deadline\|exp_mppi_track: exited" $L; do
  [ "$(date +%s)" -ge "$1" ] && { log "deadline before exp_mppi_track finished, nothing run"; exit 1; }
  sleep 60
done
source $E/tools/simenv.sh; cd /home/ram/ROS2_Autonomous_2026_27
colcon build --packages-select rover_traversability > $E/evidence/exp_vnom_build.out 2>&1 \
  || { log "build failed, stopping"; exit 1; }
log "start, governor with u_free installed ($(git rev-parse --short HEAD))"
for spec in "3d_gt_vnom05 0.0" "3d_gt_vnom05_free03 0.3"; do
  set -- $spec
  $E/tools/guard.sh 9000 ros2 run rover_nav2 benchmark --worlds marsyard crater_field rock_field slopes --runs 1 \
    --config 3d --tag $1 --launch-args odom_source:=ground_truth governor_v_nominal:=0.5 governor_u_free:=$2 \
    --out $B/2026-11-06_$1 >> $E/evidence/exp_vnom.out 2>&1 || log "$1 exited $?"
  for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null; done
  python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1  # kill -9 can leave broken SHM ports
  log "$1 done: $(cut -d, -f1,6,9,11 $B/2026-11-06_$1/runs.csv | tail -n +2 | tr '\n' ' ')"
done
log "done"
