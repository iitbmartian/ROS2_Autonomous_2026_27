# docker

One environment for everyone: ROS 2 Jazzy, Gazebo, RTAB-Map, Nav2, and every
dependency declared in `src/**/package.xml`, optionally with CUDA + the ZED SDK.

**The image holds the environment, not the code.** The repo is mounted into the
container at `/ws`, so you edit on your host with any editor and build inside.

## First time

Install Docker Engine + the Compose plugin (Linux), or Docker Desktop with WSL2
(Windows). For NVIDIA machines, also install the NVIDIA Container Toolkit.

```bash
git clone https://github.com/iitbmartian/ROS2_Autonomous_2026_27.git
cd ROS2_Autonomous_2026_27
./docker/run.sh            # pulls the image, starts the container, opens a shell
cb                         # = colcon build, then sources install/setup.bash
ros2 launch rover_slam slam_sim.launch.py rviz:=true
```

In a second terminal: `./docker/exec.sh`, then `ros2 run rover_gazebo teleop_rover.py`.

| Command | What it does |
|---|---|
| `./docker/run.sh [service]` | start the container and open a shell |
| `./docker/exec.sh [service]` | another shell in the running container |
| `./docker/update.sh [service]` | pull the newest image and restart on it |
| `./docker/build.sh [cpu\|zed]` | build the image locally instead of pulling |
| `./docker/stop.sh` | stop the containers (code and `build/` stay in the repo) |

| Service | Image | Use when |
|---|---|---|
| `dev` (default) | `rover-dev:jazzy` | any Linux machine; simulation, SLAM, Nav2 |
| `dev-nvidia` | `rover-dev:jazzy` | NVIDIA laptop, want fast Gazebo, no ZED |
| `dev-zed` | `rover-dev:jazzy-zed` | NVIDIA GPU + ZED SDK + real sensors |

In the CPU image, `colcon build` skips the four ZED wrapper packages
automatically (see `colcon/defaults-no-zed.yaml`). `zed_msgs` still builds.

## When does the image need updating?

| You changed... | Image rebuild? | What to do |
|---|---|---|
| code, launch files, configs, URDF | **no** | `cb` inside the container |
| a dependency in any `package.xml` | yes | locally: `rosdep install --from-paths src --ignore-src -y` inside the container to keep working; after merge CI publishes a new image |
| added a new package (`package.xml`) | yes | picked up automatically, same as above |
| a non-rosdep dependency (pip, SDK) | yes | add it in `Dockerfile` (marked section) |
| `docker/` itself | yes | announce it after merge (shared area, see CONTRIBUTING.md) |

After a merge that rebuilds the image, everyone runs `./docker/update.sh`.
Anything you `apt`/`rosdep install` by hand inside a container is lost when it is
recreated, which is intended: the permanent fix is the `package.xml` change.

CI (`.github/workflows/docker.yml`) builds the CPU image and runs `colcon build`
on every PR touching `src/` or `docker/`, publishes `:jazzy` and `:jazzy-zed` to
GHCR on merges that change dependencies or `docker/`, and rebuilds weekly to pick
up ROS updates. Every published image is also tagged with its commit SHA, so a
bad update can be rolled back by pinning `image:` in `docker-compose.yml`.

## Notes

- **Always build inside the container.** Mixing host and container builds in the
  same `build/` folder gives confusing CMake cache errors. If that happens:
  `rm -rf build install log` and rebuild.
- **GUI:** `run.sh` allows your own user to use the X server
  (`xhost +SI:localuser:$USER`). On Wayland this works through XWayland.
- **WSL2:** remove the `devices:` block of `dev` in `docker-compose.yml`; WSLg
  provides the display. Hardware (USB) passthrough needs `usbipd`.
- **macOS:** not supported for GUI or hardware work; no host networking, no CUDA.
- **Hardware:** `dev-zed` mounts `/dev` with cgroup rules for USB, serial, and
  video devices, so sensors can be plugged in while the container runs. Stable
  device names (udev rules) are configured on the host.
- **Rover deployment:** `docker build -f docker/Dockerfile --target rover ...`
  bakes the code in. Finish this once `rover_bringup` exists and the rover
  computer is decided (Jetson needs its own L4T-based image).
