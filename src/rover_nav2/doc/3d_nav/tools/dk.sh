#!/bin/bash
# Run a command in the rover-dev container. The repo's src/ is mounted into a separate
# container workspace (~/.ros/rover_3d_nav/cws) so container builds never mix with the host's.
WS=/home/ram/ROS2_Autonomous_2026_27
CWS=$HOME/.ros/rover_3d_nav/cws
exec docker run --rm --network host --ipc host -u $(id -u):$(id -g) -e HOME=/tmp/h \
  -v $CWS:/cws -v $WS/src:/cws/src -w /cws \
  ghcr.io/iitbmartian/rover-dev@sha256:4ba62ec8083613f5f2d601611c2c24190cf69524a5b53e091ffa1d3cf9de31c8 \
  bash -c "mkdir -p /tmp/h; source /opt/ros/jazzy/setup.bash; [ -f install/setup.bash ] && source install/setup.bash; $*"
