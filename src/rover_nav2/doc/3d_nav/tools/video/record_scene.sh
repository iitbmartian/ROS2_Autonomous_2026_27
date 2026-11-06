#!/bin/bash
# One video scene: 3D stack with Gazebo GUI and RViz, ground-truth odometry, one goal, both windows
# captured by id (no terminals). Args: goalset goal_id label
E=$HOME/.ros/rover_3d_nav; V=$E/tools/video; OUT=$HOME/rover_3d_nav_results/video
GS=$1; GID=$2; LABEL=$3; WALL=$4   # optional wall timeout s
mkdir -p $OUT/raw
source $E/tools/simenv.sh
export DISPLAY=:1 GZ_PARTITION=rover_video
cd /home/ram/ROS2_Autonomous_2026_27
read SX SY SZ SYAW GX GY GYAW TO < <(python3 - $GS $GID <<'PY'
import sys, yaml
d = yaml.safe_load(open(f'src/rover_nav2/rover_nav2/benchmark/goals/{sys.argv[1]}.yaml'))
s = d['spawn']; g = [g for g in d['goals'] if g['id'] == sys.argv[2]][0]
print(s['x'], s['y'], s['z'], s['yaw'], g['x'], g['y'], g.get('yaw', 0.0), g.get('timeout_s', d.get('timeout_s', 240)))
PY
)
W=$(python3 -c "import yaml;print(yaml.safe_load(open('src/rover_nav2/rover_nav2/benchmark/goals/$GS.yaml'))['world'])")
LOG=$OUT/raw/${GS}_${GID}.log
setsid ros2 launch rover_bringup pipeline_3d.launch.py world:=$W gui:=true viz:=false \
  x:=$SX y:=$SY z:=$SZ yaw:=$SYAW odom_source:=ground_truth > $LOG 2>&1 &
LP=$!
rviz2 -d $V/terrain_video.rviz --ros-args -p use_sim_time:=true >> $LOG 2>&1 &
RP=$!
# Nav2 is up once bt_navigator is active.
for i in $(seq 120); do
  ros2 lifecycle get /bt_navigator 2>/dev/null | grep -q active && break; sleep 5
done
GW=$(python3 $V/windows.py "Gazebo" 0 40 960 1040)
RW=$(python3 $V/windows.py "RViz" 960 40 960 1040)
gz service -s /gui/follow --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean --timeout 3000 --req 'data: "rover"' > /dev/null
gz service -s /gui/follow/offset --reqtype gz.msgs.Vector3d --reptype gz.msgs.Boolean --timeout 3000 --req 'x: -7, y: -4, z: 5' > /dev/null
# Raise both again right before recording.
python3 $V/windows.py "Gazebo" 0 40 960 1040 > /dev/null; python3 $V/windows.py "RViz" 960 40 960 1040 > /dev/null
sleep 2
ffmpeg -loglevel error -y -f x11grab -draw_mouse 0 -framerate 15 -video_size 960x1040 -i :1+0,40 -c:v libx264 -preset veryfast -pix_fmt yuv420p $OUT/raw/${GS}_${GID}_gz.mp4 < /dev/null &
F1=$!
ffmpeg -loglevel error -y -f x11grab -draw_mouse 0 -framerate 15 -video_size 960x1040 -i :1+960,40 -c:v libx264 -preset veryfast -pix_fmt yuv420p $OUT/raw/${GS}_${GID}_rviz.mp4 < /dev/null &
F2=$!
sleep 3
RES=$(python3 $V/send_goal.py $GX $GY $GYAW ${WALL:-$(python3 -c "print(int($TO*2.5))")} 2>&1 | tail -1)
sleep 4
kill -INT $F1 $F2; wait $F1 $F2 2>/dev/null
echo "$GS $GID $RES" >> $OUT/scenes.txt
kill -INT $RP; kill -INT -$LP; sleep 10; kill -9 -$LP 2>/dev/null
# Side by side, 4x the wall clock (the sim runs near 0.45 real time), a caption bar on top.
ffmpeg -loglevel error -y -i $OUT/raw/${GS}_${GID}_gz.mp4 -i $OUT/raw/${GS}_${GID}_rviz.mp4 -filter_complex \
 "[0:v]scale=-2:720,setpts=0.25*PTS[a];[1:v]scale=-2:720,setpts=0.25*PTS[b];[a][b]hstack=inputs=2,pad=iw:ih+60:0:60:black,drawtext=text='$LABEL   (3D nav, ground-truth odometry, about 2x real time)':fontcolor=white:fontsize=28:x=20:y=16[v]" \
 -map "[v]" -r 30 -c:v libx264 -crf 22 -pix_fmt yuv420p $OUT/${GS}_${GID}.mp4
echo "$OUT/${GS}_${GID}.mp4 $RES"
