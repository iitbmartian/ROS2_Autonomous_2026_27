#!/bin/bash
# setup_remote.sh user@host: copy ram/3d_nav to the host as a git bundle (no GitHub push), build
# the workspace natively there (needs /opt/ros/jazzy), install the sim tools. Safe to rerun:
# later runs fetch the new commits and rebuild.
set -e
H=$1; WS=ROS2_Autonomous_2026_27; E=$HOME/.ros/rover_3d_nav
B=$(mktemp -d)/3d_nav.bundle
git -C $HOME/$WS bundle create $B ram/3d_nav
ssh $H 'command -v nproc >/dev/null && echo "$(hostname): $(nproc) threads, $(free -g | awk "/Mem/{print \$2}") GB, $(df -BG --output=avail ~ | tail -1) free"; ls /opt/ros 2>/dev/null; command -v docker >/dev/null && docker images --format "{{.Repository}}:{{.Tag}}" | head -3' 
ssh $H "test -d /opt/ros/jazzy" || { echo "no ROS Jazzy on $H, use the docker path"; exit 2; }
scp -q $B $H:/tmp/3d_nav.bundle
ssh $H bash -s <<REMOTE
set -e
if [ ! -d ~/$WS/.git ]; then git clone -q -b ram/3d_nav /tmp/3d_nav.bundle ~/$WS; else
  cd ~/$WS && git fetch -q /tmp/3d_nav.bundle ram/3d_nav && git checkout -q ram/3d_nav && git reset -q --hard FETCH_HEAD; fi
cd ~/$WS && git submodule update --init --recursive -q 2>/dev/null || true
touch src/rover_drivers/zed2i_ws/src/zed-ros2-wrapper/COLCON_IGNORE 2>/dev/null || true
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -y -q > /tmp/rosdep.out 2>&1 || echo "rosdep incomplete, see /tmp/rosdep.out"
colcon build --parallel-workers \$(( \$(nproc) / 2 )) > /tmp/colcon.out 2>&1 && echo "build ok \$(git log --oneline -1)" || { tail -20 /tmp/colcon.out; exit 3; }
REMOTE
ssh $H "mkdir -p ~/.ros/rover_3d_nav/tools ~/.ros/rover_3d_nav/evidence ~/.ros/rover_benchmark"
for f in guard.sh fastdds_peers.xml; do scp -q $E/tools/$f $H:.ros/rover_3d_nav/tools/; done
# simenv with the remote home instead of /home/ram
sed "s#/home/ram#\$HOME#g" $E/tools/simenv.sh | ssh $H "cat > ~/.ros/rover_3d_nav/tools/simenv.sh"
echo "setup done on $H"
