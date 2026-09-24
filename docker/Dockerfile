# syntax=docker/dockerfile:1.7
# =============================================================================
# ROS2_Autonomous_2026_27 - development / deployment image
#
# The image holds the ENVIRONMENT (ROS 2 Jazzy, Gazebo, RTAB-Map, Nav2, every
# rosdep dependency of every package in src/, optionally the ZED SDK + CUDA).
# Your code is NOT baked into the dev image: docker-compose bind-mounts the repo
# at /ws, and you run `colcon build` inside the container as usual. So:
#
#   * editing code                    -> no image rebuild, just colcon build
#   * adding a dep to any package.xml -> image rebuild (CI does it on merge)
#   * adding a new package.xml        -> image rebuild, picked up automatically
#
# Build context is the REPO ROOT, e.g. from the repo root:
#   docker build -f docker/Dockerfile -t rover-dev:jazzy .
# or simply ./docker/build.sh
#
# Two flavours come from this one file, selected by BASE_IMAGE:
#   CPU  (default) : osrf/ros:jazzy-desktop
#   ZED  (NVIDIA)  : stereolabs/zed:5.4-gl-devel-cuda12.8-ubuntu24.04
# =============================================================================

ARG BASE_IMAGE=osrf/ros:jazzy-desktop

# -----------------------------------------------------------------------------
# Stage 1: extract only package.xml (+ COLCON_IGNORE markers) from src/.
# The dependency layer below is keyed on these files alone, so editing a .py
# or .cpp file never invalidates the (large, slow) rosdep install layer.
# -----------------------------------------------------------------------------
FROM ubuntu:24.04 AS manifests
COPY src /tmp/src
RUN mkdir -p /manifests && cd /tmp/src && \
    find . \( -name package.xml -o -name COLCON_IGNORE -o -name AMENT_IGNORE \) \
      -exec cp --parents {} /manifests/ \;

# -----------------------------------------------------------------------------
# Stage 2: the environment
# -----------------------------------------------------------------------------
FROM ${BASE_IMAGE} AS env

ARG ROS_DISTRO=jazzy
ENV ROS_DISTRO=${ROS_DISTRO}
ARG DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8 LC_ALL=C.UTF-8
# GL/compute/video for the NVIDIA runtime (ignored on CPU hosts)
ENV NVIDIA_DRIVER_CAPABILITIES=all

# 2a. Install ROS 2 if the base image doesn't already have it (the ZED base
#     image is plain Ubuntu 24.04 + CUDA + ZED SDK).
RUN if [ ! -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then \
      apt-get update && \
      apt-get install -y --no-install-recommends curl ca-certificates software-properties-common && \
      add-apt-repository -y universe && \
      ROS_APT_SOURCE_VERSION=$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F '"tag_name"' | awk -F'"' '{print $4}') && \
      curl -fsSL -o /tmp/ros2-apt-source.deb \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" && \
      dpkg -i /tmp/ros2-apt-source.deb && rm /tmp/ros2-apt-source.deb && \
      apt-get update && \
      apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-desktop && \
      rm -rf /var/lib/apt/lists/* ; \
    fi

# 2b. Common tooling
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-rosdep python3-colcon-common-extensions python3-vcstool python3-pip \
      build-essential cmake git sudo gosu bash-completion less nano vim \
      usbutils iputils-ping net-tools \
    && rm -rf /var/lib/apt/lists/*

# ---- Extra non-rosdep dependencies ------------------------------------------
# If your package needs something rosdep can't install (a pip package with no
# rosdep key, a vendor SDK, ...), add it HERE, in its own RUN line, and mention
# it in your PR description (docker/ is a shared area, see CONTRIBUTING.md).
# RUN pip3 install --break-system-packages <package>==<version>
# -----------------------------------------------------------------------------

# 2c. All rosdep dependencies of every package in src/.
#
# SKIP_KEYS lists keys rosdep must not try to resolve. TEMPORARY: rover_ekf
# depends on rover_interfaces (no package.xml yet) and rover_gazebosim (a package
# from the old workspace that doesn't exist here). Once both are fixed, set this
# to "" so a missing dependency fails the build loudly.
ARG SKIP_KEYS="rover_interfaces rover_gazebosim"
COPY --from=manifests /manifests /tmp/manifests
# rosdep update: drop the macOS and ROS-1 "fuerte" sources we never use, and
# retry, because flaky networks often fail one of the GitHub downloads.
RUN apt-get update && \
    (rosdep init 2>/dev/null || true) && \
    sed -i -e '/osx-homebrew.yaml/d' -e '/gbpdistro/d' /etc/ros/rosdep/sources.list.d/20-default.list && \
    for i in 1 2 3 4 5; do \
      rosdep update --rosdistro ${ROS_DISTRO} && break; \
      [ "$i" = 5 ] && exit 1; \
      echo "rosdep update failed (attempt $i of 5), retrying in 10s..."; sleep 10; \
    done && \
    rosdep install --from-paths /tmp/manifests --ignore-src -y \
      --rosdistro ${ROS_DISTRO} \
      --skip-keys "${SKIP_KEYS}" && \
    rm -rf /var/lib/apt/lists/* /tmp/manifests

# Tools no package declares but everyone wants (`ros2 control list_controllers`, ...)
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-ros2controlcli \
    && rm -rf /var/lib/apt/lists/*

# 2d. Non-root user. UID/GID 1000 by default; the entrypoint remaps it at
#     runtime to HOST_UID/HOST_GID so files in the mounted repo stay yours.
ARG USERNAME=rover
ARG USER_UID=1000
ARG USER_GID=1000
RUN if id -u ubuntu >/dev/null 2>&1; then userdel -r ubuntu; fi && \
    groupadd --gid ${USER_GID} ${USERNAME} && \
    useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USERNAME} && \
    usermod -aG video,dialout,plugdev ${USERNAME} && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME} && \
    if [ -d /usr/local/zed ]; then chown -R ${USERNAME}:${USERNAME} /usr/local/zed; fi

# Give the user a copy of root's rosdep cache (no second download), so
# `rosdep install` works inside the container
RUN mkdir -p /home/${USERNAME}/.ros && \
    cp -r /root/.ros/rosdep /home/${USERNAME}/.ros/ && \
    chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}/.ros

# colcon defaults: `colcon build` alone does the right thing. Without the ZED SDK
# in the image, the four ZED wrapper packages are skipped automatically.
COPY docker/colcon /etc/colcon
RUN if [ -d /usr/local/zed ]; then ln -s /etc/colcon/defaults-zed.yaml /etc/colcon/defaults.yaml; \
    else ln -s /etc/colcon/defaults-no-zed.yaml /etc/colcon/defaults.yaml; fi
ENV COLCON_DEFAULTS_FILE=/etc/colcon/defaults.yaml

COPY docker/bashrc /etc/rover.bashrc
RUN echo "source /etc/rover.bashrc" >> /home/${USERNAME}/.bashrc
COPY docker/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod 0755 /usr/local/bin/entrypoint.sh

ENV ROVER_USER=${USERNAME}
WORKDIR /ws
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]

# -----------------------------------------------------------------------------
# Stage 3 (later): self-contained image for the rover. Code baked in and built.
#   docker build -f docker/Dockerfile --target rover \
#     --build-arg BASE_IMAGE=stereolabs/zed:5.4-gl-devel-cuda12.8-ubuntu24.04 .
# -----------------------------------------------------------------------------
FROM env AS rover
ARG COLCON_ARGS=""
COPY --chown=rover:rover src /ws/src
RUN gosu rover bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
      cd /ws && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release ${COLCON_ARGS}"
# Replace with the real bringup once rover_bringup exists, e.g.
# CMD ["ros2", "launch", "rover_bringup", "rover.launch.py"]
CMD ["bash"]
