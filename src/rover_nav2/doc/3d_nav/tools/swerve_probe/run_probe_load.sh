#!/bin/bash
# Swerve probe under CPU load: default coupled suspension on the flat world, with busy-loop
# burners pulling the real-time factor down to benchmark levels (0.3 to 0.45). Tests whether the
# rocker coupling, closed over a ROS topic, lets the body roll when the machine is loaded.
# Starts after "exp_fillholes: done", start deadline $1 epoch. Arg 2: burner count (default 10).
E=$HOME/.ros/rover_3d_nav; P=$E/tools/swerve_probe; L=$HOME/rover_3d_nav_results/batch.log; O=$E/evidence/swerve_probe
log() { echo "$(date '+%F %H:%M') swerve_probe_load: $*" >> $L; }
NB=${2:-10}
n0=$(wc -l < $L)
until tail -n +$((n0+1)) $L | grep -q "exp_fillholes: done\|exp_fillholes: exited\|exp_fillholes: build failed"; do
  [ "$(date +%s)" -ge "$1" ] && { log "deadline before exp_fillholes finished, nothing run"; exit 1; }; sleep 30; done
source $E/tools/simenv.sh; export ROS_DOMAIN_ID=78 GZ_PARTITION=swerve_probe
cd /home/ram/ROS2_Autonomous_2026_27; log "start, $NB burners ($(git rev-parse --short HEAD))"
setsid ros2 launch rover_bringup rover_sim.launch.py world:=flat.sdf gui:=false > $O/launch_load.log 2>&1 & LP=$!
for i in $(seq 60); do ros2 topic list 2>/dev/null | grep -q '^/joint_states$' && break; sleep 2; done
sleep 15
BP=(); for i in $(seq $NB); do timeout 1500 python3 -c "while True: pass" & BP+=($!); done
timeout 1200 python3 $P/probe.py $O/probe_load.csv > $O/probe_load.out 2>&1
kill ${BP[@]} 2>/dev/null
kill -INT -$LP; sleep 10; kill -9 -$LP 2>/dev/null
python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1
python3 $P/analyse.py $O/probe_load.csv > $O/summary_load.txt 2>&1
python3 - $O/probe_load.csv >> $O/summary_load.txt <<'PY'
import csv, sys
R = list(csv.DictReader(open(sys.argv[1])))
print(f"RTF {(float(R[-1]['t']) - float(R[0]['t'])) / (float(R[-1]['wall']) - float(R[0]['wall'])):.2f}")
PY
log "done: $(tail -1 $O/summary_load.txt), max roll $(grep -o 'roll *[0-9.]*' $O/summary_load.txt | awk '{print $2}' | sort -n | tail -1) deg"
