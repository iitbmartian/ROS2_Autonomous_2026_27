#!/bin/bash
# suspension:=plugin (RockerCouplingSystem, ram/suspension worktree) on the flat probe, light load
# and 10 burners, against the torque-node results already in summary.txt and summary_load.txt.
# The probe drives in swerve mode, which main lacks, so it overlays only rover_gazebo and
# rover_bringup from the worktree on the 3d_nav install (swerve kinematics comes from there, see
# note in the log). Starts after "exp_gate_free03: done", deadline $1.
E=$HOME/.ros/rover_3d_nav; P=$E/tools/swerve_probe; L=$HOME/rover_3d_nav_results/batch.log; O=$E/evidence/swerve_probe
log() { echo "$(date '+%F %H:%M') swerve_probe_plugin: $*" >> $L; }
n0=$(wc -l < $L)
until tail -n +$((n0+1)) $L | grep -q "exp_gate_free03: done\|exp_gate_free03: exited"; do
  [ "$(date +%s)" -ge "$1" ] && { log "deadline before exp_gate_free03 finished, nothing run"; exit 1; }; sleep 30; done
source $E/tools/simenv.sh
source /home/ram/ws_susp_test/install/setup.bash   # 3d_nav plus ram/suspension, swerve kinematics included
export ROS_DOMAIN_ID=78 GZ_PARTITION=swerve_probe
log "start, plugin from ram/test-3dnav-suspension $(git -C /home/ram/ws_susp_test rev-parse --short HEAD)"
for nb in 0 10; do
  setsid ros2 launch rover_bringup rover_sim.launch.py world:=flat.sdf gui:=false suspension:=plugin > $O/launch_plugin_$nb.log 2>&1 & LP=$!
  for i in $(seq 60); do ros2 topic list 2>/dev/null | grep -q '^/joint_states$' && break; sleep 2; done
  sleep 15
  BP=(); for i in $(seq $nb); do timeout 1500 python3 -c "while True: pass" & BP+=($!); done
  timeout 1200 python3 $P/probe.py $O/probe_plugin_$nb.csv > $O/probe_plugin_$nb.out 2>&1
  [ ${#BP[@]} -gt 0 ] && kill ${BP[@]} 2>/dev/null
  kill -INT -$LP; sleep 10; kill -9 -$LP 2>/dev/null
  python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1
done
for nb in 0 10; do
  python3 $P/analyse.py $O/probe_plugin_$nb.csv
  python3 -c "
import csv,sys; R=list(csv.DictReader(open(sys.argv[1]))); print(f\"RTF {(float(R[-1]['t'])-float(R[0]['t']))/(float(R[-1]['wall'])-float(R[0]['wall'])):.2f}\")" $O/probe_plugin_$nb.csv
done > $O/summary_plugin.txt 2>&1
grep -q RockerCouplingSystem $O/launch_plugin_0.log && pl=loaded || pl="NOT loaded"
log "done: plugin $pl, $(grep RTF $O/summary_plugin.txt | tr '\n' ' '), max roll $(grep -o 'roll *[0-9.]*' $O/summary_plugin.txt | awk '{print $2}' | sort -n | tail -1) pitch $(grep -o 'pitch *[0-9.]*' $O/summary_plugin.txt | awk '{print $2}' | sort -n | tail -1)"
