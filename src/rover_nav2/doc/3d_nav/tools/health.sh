#!/bin/bash
# One-shot health report for the unattended chain, read by the hourly check.
L=$HOME/rover_3d_nav_results/batch.log; E=$HOME/.ros/rover_3d_nav/evidence; B=$HOME/.ros/rover_benchmark
echo "== $(date '+%F %H:%M:%S'), cpu $(( $(cat /sys/class/hwmon/hwmon5/temp1_input)/1000 ))C, $(df -h ~ | awk 'END{print $5" disk"}')"
echo "== chain scripts"; ps -eo pid,etimes,args | awk '$4 ~ /(exp_|p2_|backfill|hang_watch|batch_)[a-z0-9_]*\.sh$/ {print $1, int($2/60)"min", $4, $5}'
echo "== runners"; pgrep -af "lib/rover_nav2/benchmark --worlds" | cut -c1-160
n=0; for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && n=$((n+1)); done; echo "marked processes: $n, gz sim: $(ps -eo args | grep -c '[g]z sim ')"
echo "== batch.log"; tail -5 $L | cut -c1-250
f=$(ls -t $E/*.out 2>/dev/null | head -1); a=$(( $(date +%s) - $(stat -c %Y $f) ))
echo "== newest runner output $f, last write ${a} s ago"; grep -E "^\d|^[0-9]{2}:[0-9]{2}" $f | tail -4 | cut -c1-200
echo "== newest runs.csv"; c=$(ls -t $B/*/runs.csv | head -1); echo $c; cut -d, -f1,6,9,11 $c | tail -n +2 | awk -F, '{s[$2]++} END{for(k in s) printf "%s %d  ", k, s[k]; print ""}'; tail -3 $c | cut -d, -f1,6,8,9,11
python3 $HOME/.ros/rover_3d_nav/tools/shm_stale_ports.py 2>&1 | tail -1
