#!/bin/bash
# Speed parity (v2): v1.1 Ackermann with a looser speed governor, u_free 0.4 then 0.5 (default 0.3),
# on marsyard, rock_field, urc_mix x3. Compared with ckpt/ck3 at u_free 0.3 for time and safety.
# Args: deadline (epoch), pid to wait for.
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log; B=$HOME/.ros/rover_benchmark; X=$E/exp
log() { echo "$(date '+%F %H:%M') exp_gov: $*" >> $L; }
clean() { for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null; done
  python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1; }
tally() { awk -F, -v w=$2.sdf 'NR>1&&$3==w{print $4":"$6}' $1/runs.csv 2>/dev/null | sort | uniq -c | tr -s ' \n' ' '; }
while kill -0 $2 2>/dev/null; do [ "$(date +%s)" -ge "$1" ] && { log "deadline while waiting"; exit 1; }; sleep 60; done
cd /home/ram/ROS2_Autonomous_2026_27; source /opt/ros/jazzy/setup.bash
source $E/tools/simenv.sh; unset ROVER_COUPLING_K ROVER_COUPLING_D ROVER_COUPLING_MAX
log "start ($(git rev-parse --short HEAD))"
for uf in 0.4 0.5; do
  t=3d_gt_gov${uf/./}_d250; O=$B/$(date +%F)_$t
  A="--config 3d --launch-args odom_source:=ground_truth suspension:=plugin governor_v_nominal:=0.5 governor_u_free:=$uf global_fine_step:=true nav_extra_overlay:=$X/nav2_3d_reapproach.yaml"
  for w in marsyard rock_field urc_mix; do [ "$(date +%s)" -ge "$1" ] && { log "deadline before $t $w"; break 2; }
    $E/tools/guard.sh 9000 ros2 run rover_nav2 benchmark --worlds $w --runs 3 --tag $t $A --out $O >> $E/evidence/exp_gov.out 2>&1 || log "$t $w exited $?"; clean
    log "$t $w: $(tally $O $w)"
  done
done
log "done"
