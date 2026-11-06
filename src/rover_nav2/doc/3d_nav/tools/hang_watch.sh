#!/bin/bash
# Kills a benchmark runner that sits with no stack running for over 4 min (2026-11-06: deadlocked
# in its node teardown for 1.5 h between runs), or whose one stack lives over 75 min (2026-11-06:
# hung building its node after a failed start). Exits at the deadline (epoch seconds, $1).
L=$HOME/rover_3d_nav_results/batch.log; idle=0; last=""; age=0
while [ "$(date +%s)" -lt "$1" ]; do
  r=$(pgrep -f "lib/rover_nav2/benchmark --worlds" | head -1)
  m=""
  for p in /proc/[0-9]*; do
    m=$(tr '\0' '\n' < $p/environ 2>/dev/null | grep -m1 '^ROVER_BENCH_RUN=') && break
  done
  if [ -n "$r" ] && [ -z "$m" ]; then idle=$((idle+20)); else idle=0; fi
  if [ -n "$m" ] && [ "$m" = "$last" ]; then age=$((age+20)); else age=0; last=$m; fi
  if [ $idle -ge 240 ] || { [ -n "$r" ] && [ $age -ge 4500 ]; }; then
    echo "$(date '+%F %H:%M') hang_watch: runner $r stuck (idle ${idle} s, stack age ${age} s), killed" >> $L
    kill -9 $r; idle=0; age=0
  fi
  sleep 20
done
echo "$(date '+%F %H:%M') hang_watch: deadline" >> $L
