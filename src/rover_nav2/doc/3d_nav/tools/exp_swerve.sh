#!/bin/bash
# First Phase 4 look: swerve drive (MPPI Omni, TerrainCritic) on marsyard, slopes, rock_field,
# urc_mix, 1 run per goal, with the faster governor of exp_vnom A (v_nominal 0.5, u_free 0).
# Compared with exp_vnom A (same governor, Ackermann). Starts after exp_confirm ends or gives up at $1.
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log; B=$HOME/.ros/rover_benchmark
log() { echo "$(date '+%F %H:%M') exp_swerve: $*" >> $L; }
n0=$(wc -l < $L)  # only log lines written after this start count, older done lines are stale
until tail -n +$((n0+1)) $L | grep -q "exp_confirm: done\|exp_confirm: exited"; do
  [ "$(date +%s)" -ge "$1" ] && { log "deadline before p2_crater_gt finished, nothing run"; exit 1; }
  sleep 60
done
source $E/tools/simenv.sh; cd /home/ram/ROS2_Autonomous_2026_27
log "start ($(git rev-parse --short HEAD))"
$E/tools/guard.sh 10800 ros2 run rover_nav2 benchmark --worlds marsyard slopes rock_field urc_mix --runs 1 \
  --config 3d --tag 3d_gt_swerve --launch-args odom_source:=ground_truth drive:=swerve \
  governor_v_nominal:=0.5 governor_u_free:=0.0 \
  --out $B/2026-11-07_3d_gt_swerve >> $E/evidence/exp_swerve.out 2>&1 || log "exited $?"
for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null; done
python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1  # kill -9 can leave broken SHM ports
log "done: $(cut -d, -f1,6,9,11 $B/2026-11-07_3d_gt_swerve/runs.csv | tail -n +2 | tr '\n' ' ')"
