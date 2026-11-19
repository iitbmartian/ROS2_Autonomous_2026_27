#!/bin/bash
# p26weak: every ram/3d_nav-p26 fix (observed dips, START_OCCUPIED escape, from ~/ws_p26/install) plus
# the Ackermann tip guard overlay, on the v2 weak goals x5 (same set as v2weak 28/30, pooled v2 11-run
# base). Tests the escape on the Start occupied goals (rock ahead_10m, marsyard behind_left) and the
# guard on the tip-stop goals (rock far_right, urc far_south). No build. Args: deadline (epoch), pid.
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log; B=$HOME/.ros/rover_benchmark
log() { echo "$(date '+%F %H:%M') exp_p26weak: $*" >> $L; }
clean() { for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null; done
  python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1; }
tally() { awk -F, -v w=$2.sdf 'NR>1&&$3==w{print $4":"$7}' $1/runs.csv 2>/dev/null | sort | uniq -c | tr -s ' \n' ' '; }
while kill -0 $2 2>/dev/null; do [ "$(date +%s)" -ge "$1" ] && { log "deadline while waiting"; exit 1; }; sleep 60; done
cd /home/ram/ROS2_Autonomous_2026_27; source /opt/ros/jazzy/setup.bash; source install/setup.bash
source $HOME/ws_p26/install/setup.bash
[ "$(ros2 pkg prefix rover_terrain_mapping)" = "$HOME/ws_p26/install/rover_terrain_mapping" ] && [ "$(ros2 pkg prefix rover_nav2)" = "$HOME/ws_p26/install/rover_nav2" ] || { log "p26 overlay not active, stopping"; exit 1; }
clean
source $E/tools/simenv.sh; unset ROVER_COUPLING_K ROVER_COUPLING_D ROVER_COUPLING_MAX
t=3d_gt_p26weak_d250; O=$B/$(date +%F)_$t
[ -e $O ] && { log "out dir $O exists, a rerun would skip its done runs, stopping"; exit 1; }
log "start (stack install of $(git rev-parse --short HEAD), p26 overlay $(git -C $HOME/ws_p26 rev-parse --short HEAD)), out $O"
A="--config 3d --launch-args odom_source:=ground_truth suspension:=plugin governor_v_nominal:=0.5 governor_u_free:=0.3 nav_extra_overlay:=$HOME/.ros/rover_3d_nav/exp/nav2_3d_tipguard.yaml"
first=1
for spec in "slopes --goals sidehill_uphill" "marsyard --goals behind_left" "rock_field --goals far_right ahead_10m" \
            "urc_mix --goals far_south" "bars --goals past_bars_6m"; do
  [ "$(date +%s)" -ge "$1" ] && { log "deadline before ${spec%% *}"; break; }
  $E/tools/guard.sh 9000 ros2 run rover_nav2 benchmark --worlds $spec --runs 5 --tag $t $A --out $O >> $E/evidence/exp_p26weak.out 2>&1 || log "$t ${spec%% *} exited $?"; clean
  log "$t ${spec%% *} successes: $(tally $O ${spec%% *})"
  if [ $first = 1 ]; then first=0
    f=$(ls -tr $O/logs/*.log 2>/dev/null | head -1)
    grep -q "observed dips on" "$f" && grep -q "Critic loaded : mppi::critics::TerrainCritic" "$f" && grep -q "HeadingAwarePlanner:" "$f" || { log "p26 stack not loaded in $f, stopping"; break; }
  fi
  [ "$(awk -F, 'NR>1' $O/runs.csv 2>/dev/null | grep -vc startup_failed)" = 0 ] && { log "all startup_failed, stopping"; break; }
done
log "done"
