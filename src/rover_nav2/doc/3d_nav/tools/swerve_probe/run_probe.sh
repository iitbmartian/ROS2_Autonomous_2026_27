#!/bin/bash
# Swerve suspension probe: flat world, coupling (default) vs mimic + Bullet-Featherstone.
# Starts after "exp_morning: done" (or now with arg 'now'), start deadline $1 epoch.
E=$HOME/.ros/rover_3d_nav; P=$E/tools/swerve_probe; L=$HOME/rover_3d_nav_results/batch.log; O=$E/evidence/swerve_probe
log() { echo "$(date '+%F %H:%M') swerve_probe: $*" >> $L; }
if [ "$2" != now ]; then
  n0=$(wc -l < $L)
  until tail -n +$((n0+1)) $L | grep -q "exp_morning: done"; do
    [ "$(date +%s)" -ge "$1" ] && { log "deadline before exp_morning finished, nothing run"; exit 1; }; sleep 30; done
fi
source $E/tools/simenv.sh; export ROS_DOMAIN_ID=78 GZ_PARTITION=swerve_probe; mkdir -p $O
cd /home/ram/ROS2_Autonomous_2026_27; log "start ($(git rev-parse --short HEAD))"
for cfg in "coupling" "mimic suspension:=mimic physics_engine:=gz-physics-bullet-featherstone-plugin"; do
  set -- $cfg; tag=$1; shift
  setsid ros2 launch rover_bringup rover_sim.launch.py world:=flat.sdf gui:=false "$@" > $O/launch_$tag.log 2>&1 & LP=$!
  for i in $(seq 60); do ros2 topic list 2>/dev/null | grep -q '^/joint_states$' && break; sleep 2; done
  sleep 15   # controllers active and the rocker rest pose captured
  timeout 900 python3 $P/probe.py $O/probe_$tag.csv > $O/probe_$tag.out 2>&1
  kill -INT -$LP; sleep 10; kill -9 -$LP 2>/dev/null
  python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1
done
python3 $P/analyse.py $O/probe_coupling.csv $O/probe_mimic.csv > $O/summary.txt 2>&1
log "done: $(grep -c . $O/summary.txt) lines in $O/summary.txt"
