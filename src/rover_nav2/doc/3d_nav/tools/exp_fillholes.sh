#!/bin/bash
# After the swerve probe: rebuild the main install (rover_terrain_mapping, rover_bringup) with the
# small-hole fill, then 3D Ackermann (confirm config) with fill_holes:=true on marsyard,
# crater_field, rock_field, urc_mix, 2 runs, against the confirm run. Start deadline $1.
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log; B=$HOME/.ros/rover_benchmark; O=$B/2026-11-08_3d_gt_fillholes
log() { echo "$(date '+%F %H:%M') exp_fillholes: $*" >> $L; }
n0=$(wc -l < $L)
until tail -n +$((n0+1)) $L | grep -q "swerve_probe: done\|swerve_probe: deadline"; do
  [ "$(date +%s)" -ge "$1" ] && { log "deadline before swerve_probe finished, nothing run"; exit 1; }; sleep 30; done
cd /home/ram/ROS2_Autonomous_2026_27
if ! bash -c 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && colcon build --packages-select rover_terrain_mapping rover_bringup --cmake-args -DCMAKE_BUILD_TYPE=Release' > $E/evidence/exp_fillholes_build.log 2>&1; then
  log "build failed, see evidence/exp_fillholes_build.log"; exit 1; fi
source $E/tools/simenv.sh
log "start ($(git rev-parse --short HEAD))"
$E/tools/guard.sh 10800 ros2 run rover_nav2 benchmark --worlds marsyard crater_field rock_field urc_mix --runs 2 --config 3d --tag 3d_gt_fillholes \
  --launch-args odom_source:=ground_truth governor_v_nominal:=0.5 governor_u_free:=0.0 fill_holes:=true \
  --out $O >> $E/evidence/exp_fillholes.out 2>&1 || log "exited $?"
for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null; done
python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1
log "done: $(cut -d, -f1,6,9,11 $O/runs.csv | tail -n +2 | tr '\n' ' ')"
