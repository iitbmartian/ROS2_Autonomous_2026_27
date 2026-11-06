#!/bin/bash
# guard.sh <seconds> <cmd...>: run cmd in its own process group with a hard wall-clock cap. INT at
# the cap, TERM 20 s later, KILL 20 s after that, so a node that ignores SIGINT cannot block a
# batch (2026-11-05: a bag recorder did, 16 h lost). Refuses to start below 10 GB free disk.
cap=$1; shift
free=$(df -BG --output=avail "$HOME" | tail -1 | tr -dc 0-9)
if [ "$free" -lt "${GUARD_MIN_GB:-10}" ]; then echo "guard: only ${free} GB free, not starting: $*" >&2; exit 90; fi
setsid "$@" & pid=$!
( sleep "$cap"; kill -INT -$pid 2>/dev/null; sleep 20; kill -TERM -$pid 2>/dev/null; sleep 20
  kill -KILL -$pid 2>/dev/null; echo "guard: cap ${cap}s hit: $*" >&2 ) & wd=$!
wait $pid; rc=$?
kill $wd 2>/dev/null; wait $wd 2>/dev/null
exit $rc
