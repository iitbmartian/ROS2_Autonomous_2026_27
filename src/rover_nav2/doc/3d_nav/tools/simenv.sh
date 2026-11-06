# Isolated ROS/Gazebo session for the 3D nav sim runs, so they never collide with other sessions.
source /opt/ros/jazzy/setup.bash
source /home/ram/ROS2_Autonomous_2026_27/install/setup.bash
export ROS_DOMAIN_ID=42
export GZ_PARTITION=rover3d
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
# LOCALHOST discovery reaches participant ids 0..31 only (loopback has no multicast), a 3D stack
# with tools passes that. The profile widens the peer range (rover_nav2 config).
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/ram/ROS2_Autonomous_2026_27/install/rover_nav2/share/rover_nav2/config/fastdds_sim_localhost.xml
export RCUTILS_LOGGING_BUFFERED_STREAM=0   # unbuffered node logs, so a log read mid-run is complete
# Gazebo transport on loopback. The 2026-11-02 loopback profile (fastdds_loopback.xml) that left
# late-joining nodes undiscovered most likely hit the same 32-participant limit.
export GZ_IP=127.0.0.1
# Opt-in Fast DDS discovery server (ROVER_DDS_DS=1): every participant registers with one local
# server instead of finding 50 peers one by one. Under test since 2026-11-03, after discovery
# misses stalled goals, the benchmark runner and once the velocity chain.
if [ "${ROVER_DDS_DS:-0}" = 1 ]; then
  export ROS_DISCOVERY_SERVER=127.0.0.1:11888
  if ! pgrep -f "fastdds discovery -i 0 -l 127.0.0.1 -p 11888" > /dev/null; then
    setsid nohup fastdds discovery -i 0 -l 127.0.0.1 -p 11888 > /dev/null 2>&1 < /dev/null &
    sleep 1
  fi
fi
