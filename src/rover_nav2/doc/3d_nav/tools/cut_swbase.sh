#!/bin/bash
# Stops exp_swbase (pid $1) right after it logs urc_mix, so exp_sw5 (waiting on that pid) starts.
# The remaining worlds run later in exp_swbase_rest.sh. Kills only the script, its guard and the
# ROVER_BENCH_RUN processes of the stage it may just have started.
E=$HOME/.ros/rover_3d_nav; L=$HOME/rover_3d_nav_results/batch.log
until grep -q "exp_swbase: 3d_gt_swbase_d250 urc_mix:" $L; do kill -0 $1 2>/dev/null || exit 0; sleep 10; done
kill $1; sleep 2
for p in $(pgrep -f "rover_nav2 benchmark --worlds slopes --runs 3 --tag 3d_gt_swbase"); do kill -- -$(ps -o pgid= -p $p | tr -d ' ') 2>/dev/null; kill $p 2>/dev/null; done
sleep 5
for p in /proc/[0-9]*; do grep -qa ROVER_BENCH_RUN $p/environ 2>/dev/null && kill -9 ${p#/proc/} 2>/dev/null; done
python3 $E/tools/shm_stale_ports.py --rm > /dev/null 2>&1
echo "$(date '+%F %H:%M') cut_swbase: exp_swbase stopped after urc_mix, exp_sw5 next, rest in exp_swbase_rest" >> $L
