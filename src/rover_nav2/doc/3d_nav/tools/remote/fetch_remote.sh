#!/bin/bash
# fetch_remote.sh user@host out_dir: copy a remote benchmark folder here (db maps left behind).
rsync -a --exclude db $1:.ros/rover_benchmark/$2/ $HOME/.ros/rover_benchmark/$2_$(echo $1 | tr -c 'a-zA-Z0-9\n' _)/
ssh $1 "test -f ~/.ros/rover_benchmark/$2.done" && echo "$1 $2 finished" || echo "$1 $2 still running"
