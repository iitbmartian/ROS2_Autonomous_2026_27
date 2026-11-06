#!/bin/bash
# Discovery trial: default Fast DDS against the discovery server (ROVER_DDS_DS=1). Per stack
# start: does a fresh process discover bt_navigator, does a goal get accepted, does the rover
# move at least 1 m (Gazebo pose) within 90 s. Results in evidence/dds/trial.csv.
E=$HOME/.ros/rover_3d_nav
OUT=$E/evidence/dds; mkdir -p $OUT
N=${1:-4}
echo "mode,trial,up,discovered,accepted,moved_m" > $OUT/trial.csv
for MODE in 0 1; do
  for T in $(seq $N); do
    export ROVER_DDS_DS=$MODE
    NAME=dds_m${MODE}_t$T
    $E/tools/sim_up.sh dds $NAME world:=marsyard.sdf > $OUT/$NAME.up 2>&1 && UP=1 || UP=0
    DISC=0; ACC=0; MOVED=0
    if [ $UP = 1 ]; then
      sleep 10
      source $E/tools/simenv.sh
      timeout 40 python3 $E/tools/disc_probe.py 30 | grep -q ok && DISC=1
      P0=$(timeout 20 python3 $E/tools/gt_pose.py)
      timeout 120 python3 $E/tools/send_goal.py 8.0 0.0 0.0 90 > $OUT/$NAME.goal 2>&1
      grep -q accepted $OUT/$NAME.goal && ACC=1
      P1=$(timeout 20 python3 $E/tools/gt_pose.py)
      MOVED=$(python3 -c "
a='$P0'.split(); b='$P1'.split()
print(f'{((float(a[0])-float(b[0]))**2+(float(a[1])-float(b[1]))**2)**0.5:.2f}' if len(a)==2 and len(b)==2 else 'nan')")
    fi
    echo "$MODE,$T,$UP,$DISC,$ACC,$MOVED" >> $OUT/trial.csv
    $E/tools/sim_down.sh $NAME > /dev/null
  done
done
pkill -f "fastdds discovery -i 0 -l 127.0.0.1 -p 11888"
cat $OUT/trial.csv
