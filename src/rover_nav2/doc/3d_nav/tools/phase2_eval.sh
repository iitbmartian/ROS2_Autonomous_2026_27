#!/bin/bash
# Phase 2 gate runs: per world, the 3D stack drives goals while terrain_eval scores the map
# against ground truth. Usage: phase2_eval.sh WORLD [GOAL_ID ...] (default: the first goal).
E=$HOME/.ros/rover_3d_nav
WORLD=$1; shift
GOALS="$*"
source $E/tools/simenv.sh
OUT=$E/evidence/phase2
mkdir -p $OUT
GS=$(ros2 pkg prefix rover_nav2)/share/rover_nav2/benchmark/goals/$WORLD.yaml
[ -z "$GOALS" ] && GOALS=$(python3 -c "import yaml;print(yaml.safe_load(open('$GS'))['goals'][0]['id'])")
CRATERS=""
if [ "$WORLD" = crater_field ]; then
  CRATERS="--craters $(ros2 pkg prefix rover_gazebo)/share/rover_gazebo/models/TerrainCraterField/craters.yaml"
fi
$E/tools/sim_down.sh "p2_$WORLD" > /dev/null
$E/tools/sim_up.sh phase2 "p2_$WORLD" world:=$WORLD.sdf $P2_EXTRA || { $E/tools/sim_down.sh "p2_$WORLD"; exit 1; }
sleep 10    # let the first maps arrive
ros2 run rover_nav2 terrain_eval --world $WORLD --out $OUT/eval_$WORLD.csv --duration 100000 $CRATERS > $OUT/eval_$WORLD.out 2>&1 &
EVAL=$!
for G in $GOALS; do
  read X Y YAW TO < <(python3 -c "
import yaml
gs = yaml.safe_load(open('$GS'))
g = [g for g in gs['goals'] if g['id'] == '$G'][0]
print(g['x'], g['y'], g.get('yaw', 0.0), gs['timeout_s'])")
  python3 $E/tools/send_goal.py $X $Y $YAW $((TO * 3)) 2>&1 | sed "s/^/$(date +%H:%M) $G /" | tee -a $OUT/goals_$WORLD.log
done
timeout 120 python3 $E/tools/snapshot.py "p2_${WORLD}_end" $WORLD > $OUT/snapshot_$WORLD.txt 2>&1
# ros2 run does not forward SIGINT, so signal terrain_eval itself, then force after 30 s.
pkill -INT -f "lib/rover_nav2/terrain_eval --world $WORLD"
for i in $(seq 30); do kill -0 $EVAL 2>/dev/null || break; sleep 1; done
pkill -KILL -f "lib/rover_nav2/terrain_eval --world $WORLD"; kill $EVAL 2>/dev/null; wait $EVAL
cat $OUT/eval_$WORLD.txt
$E/tools/sim_down.sh "p2_$WORLD"
