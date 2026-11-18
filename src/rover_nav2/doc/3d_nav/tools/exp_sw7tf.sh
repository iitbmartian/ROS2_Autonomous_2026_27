#!/bin/bash
# sw7tf: swerve v7 without the MPPI TerrainCritic (overlay nav2_3d_swerve_v7t.yaml), the best swerve
# variant so far (sw7t 5/10 targeted, sw7u 0/8, sw7c 0/6 on slopes), on the full 6 worlds x3. The v3
# rule is that swerve ships only at Ackermann level (v2rel 49/57) with 0 unsafe. No build.
# Args: deadline (epoch), pid to wait for.
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log; B=$HOME/.ros/rover_benchmark; X=$E/exp
log() { echo "$(date '+%F %H:%M') exp_sw7tf: $*" >> $L; }
clean() { for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null; done
  python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1; }
tally() { awk -F, -v w=$2.sdf 'NR>1&&$3==w{print $4":"$7}' $1/runs.csv 2>/dev/null | sort | uniq -c | tr -s ' \n' ' '; }
while kill -0 $2 2>/dev/null; do [ "$(date +%s)" -ge "$1" ] && { log "deadline while waiting"; exit 1; }; sleep 60; done
cd /home/ram/ROS2_Autonomous_2026_27; source /opt/ros/jazzy/setup.bash; source install/setup.bash
clean
source $E/tools/simenv.sh; unset ROVER_COUPLING_K ROVER_COUPLING_D ROVER_COUPLING_MAX
log "start (install of $(git rev-parse --short HEAD))"
t=3d_gt_sw7tf_d250; O=$B/$(date +%F)_$t
[ -e $O ] && { log "out dir $O exists, a rerun would skip its done runs, stopping"; exit 1; }
A="--config 3d --launch-args odom_source:=ground_truth suspension:=plugin governor_v_nominal:=0.5 governor_u_free:=0.3 drive:=swerve mask_tip_margin:=0.05 nav_extra_overlay:=$X/nav2_3d_swerve_v7t.yaml"
first=1
for w in slopes crater_field marsyard rock_field urc_mix bars; do
  [ "$(date +%s)" -ge "$1" ] && { log "deadline before $w"; break; }
  $E/tools/guard.sh 9000 ros2 run rover_nav2 benchmark --worlds $w --runs 3 --tag $t $A --out $O >> $E/evidence/exp_sw7tf.out 2>&1 || log "$t $w exited $?"; clean
  log "$t $w successes: $(tally $O $w)"
  if [ $first = 1 ]; then first=0
    f=$(ls -tr $O/logs/*.log 2>/dev/null | head -1)
    grep -q "anchor on" "$f" || { log "HeadingSmoother anchor not on, stopping"; break; }
    grep -q "Critic loaded : mppi::critics::TerrainCritic" "$f" && { log "TerrainCritic still loaded, stopping"; break; }
  fi
  [ "$(awk -F, 'NR>1' $O/runs.csv 2>/dev/null | grep -vc startup_failed)" = 0 ] && { log "all startup_failed, stopping"; break; }
done
log "done"
