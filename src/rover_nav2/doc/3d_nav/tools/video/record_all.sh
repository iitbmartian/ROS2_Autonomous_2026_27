#!/bin/bash
# Ram 2026-11-04: video of the 3D stack, after the ground-truth batch. Then one stitched video.
OUT=$HOME/rover_3d_nav_results/video; V=$HOME/.ros/rover_3d_nav/tools/video
L=$HOME/rover_3d_nav_results/batch.log
until grep -q "gt fix done" $L; do sleep 60; done
echo "$(date '+%F %H:%M') video start" >> $L
mkdir -p $OUT; : > $OUT/scenes.txt
$V/record_scene.sh marsyard ahead_8m "Marsyard, 8 m between rocks"
$V/record_scene.sh crater_field far_left "Crater field, 18 m around craters"
$V/record_scene.sh slopes past_lanes "Slopes, 20 m past the ramp lanes"
$V/record_scene.sh slopes lane35_top "Slopes, 35 deg lane, expected refusal" 240
$V/record_scene.sh rock_field ahead_10m "Rock field, 10 m through rocks" 500
$V/record_scene.sh urc_mix far_east "URC mix, 42 m mixed terrain" 900
ls $OUT/*.mp4 | grep -v all_scenes | sed "s/^/file '/; s/$/'/" > $OUT/list.txt
ffmpeg -loglevel error -y -f concat -safe 0 -i $OUT/list.txt -c copy $OUT/3d_nav_all_scenes.mp4
echo "$(date '+%F %H:%M') video done" >> $L
