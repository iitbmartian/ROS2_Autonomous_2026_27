#!/bin/bash
# Remaps the in-container user to the host user's UID/GID (so files created in
# the mounted repo belong to you, not root), sources ROS, then drops privileges.
set -e

USER_NAME="${ROVER_USER:-rover}"

# Source ROS (+ the workspace overlay if it has been built) so that commands
# like `docker compose run dev ros2 launch ...` work without an interactive shell.
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [ -f /ws/install/setup.bash ]; then
  # shellcheck disable=SC1091
  source /ws/install/setup.bash
fi

if [ "$(id -u)" = "0" ]; then
  HOST_UID="${HOST_UID:-$(id -u "$USER_NAME")}"
  HOST_GID="${HOST_GID:-$(id -g "$USER_NAME")}"

  if [ "$HOST_UID" = "0" ]; then
    # Host user is root (e.g. a system service on the rover): stay root.
    exec "$@"
  fi

  if [ "$HOST_GID" != "$(id -g "$USER_NAME")" ]; then
    if getent group "$HOST_GID" >/dev/null; then
      usermod -g "$HOST_GID" "$USER_NAME"
    else
      groupmod -g "$HOST_GID" "$USER_NAME"
    fi
  fi
  if [ "$HOST_UID" != "$(id -u "$USER_NAME")" ]; then
    usermod -u "$HOST_UID" "$USER_NAME"   # also re-owns /home/$USER_NAME
    if [ -d /usr/local/zed ]; then
      chown -R "$USER_NAME" /usr/local/zed/settings /usr/local/zed/resources 2>/dev/null || true
    fi
  fi

  # Give the user access to host device nodes (GPU render nodes, cameras,
  # serial ports). Their group IDs come from the host (e.g. "render") and
  # usually don't exist in the image, so create matching groups on the fly.
  for dev in /dev/dri/* /dev/video* /dev/ttyUSB* /dev/ttyACM*; do
    [ -e "$dev" ] || continue
    gid="$(stat -c %g "$dev")"
    [ "$gid" = "0" ] && continue
    grp="$(getent group "$gid" | cut -d: -f1)"
    if [ -z "$grp" ]; then
      grp="hostdev$gid"
      groupadd -g "$gid" "$grp"
    fi
    id -nG "$USER_NAME" | grep -qw "$grp" || usermod -aG "$grp" "$USER_NAME"
  done

  exec gosu "$USER_NAME" "$@"
fi

exec "$@"
