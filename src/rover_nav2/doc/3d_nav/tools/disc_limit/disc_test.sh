#!/bin/bash
# 40 idle nodes on a spare domain, then a 41st counts which it sees. Arg: extra env setup.
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=77 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
eval "$1"
S=$(dirname $0)
for i in $(seq -w 0 39); do python3 $S/idle_node.py idle$i > /dev/null 2>&1 & sleep 0.15; done
sleep 8
python3 $S/count_nodes.py 2>&1 | grep -E "of 40|rror" | head -3
kill -9 $(jobs -p) 2>/dev/null; wait 2>/dev/null; true
