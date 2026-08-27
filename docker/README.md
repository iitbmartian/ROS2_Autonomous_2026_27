# docker

**Status:** placeholder — no Dockerfile yet

The whole autonomy stack is intended to run inside Docker, so that every member develops
against the same ROS2 distro, CUDA version, and driver SDKs (ZED SDK, sbg, lidar drivers)
regardless of what their host machine runs.

## What belongs here

- `Dockerfile` — base image, ROS2 install, driver SDKs, Python deps
- `docker-compose.yml` — device passthrough (`/dev/video*`, USB, network), X11/GUI forwarding
  for RViz and Gazebo, workspace bind-mount
- helper scripts — `build.sh`, `run.sh`, `exec.sh`

## Notes

Anything changed here affects everyone's environment. Announce changes to the team after merge
and note them in the PR description — see [CONTRIBUTING.md](../CONTRIBUTING.md).
