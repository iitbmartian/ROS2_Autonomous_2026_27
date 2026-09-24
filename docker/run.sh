#!/usr/bin/env bash
# Start the dev container (pulls or builds the image if missing) and open a shell.
#   ./docker/run.sh              dev         (any Linux machine)
#   ./docker/run.sh dev-nvidia   NVIDIA GPU, no ZED SDK
#   ./docker/run.sh dev-zed      NVIDIA GPU + ZED SDK + hardware access
set -euo pipefail
SERVICE="${1:-dev}"
cd "$(dirname "$0")"

export HOST_UID HOST_GID
HOST_UID="$(id -u)"
HOST_GID="$(id -g)"

# Let the container (same UID as you) draw windows for Gazebo / RViz.
if command -v xhost >/dev/null 2>&1 && [ -n "${DISPLAY:-}" ]; then
  xhost +SI:localuser:"$(id -un)" >/dev/null || true
else
  echo "note: no xhost/DISPLAY found, GUI apps (Gazebo, RViz) won't open." >&2
fi

docker compose up -d "$SERVICE"

# Wait until the entrypoint has finished setting up the user (UID remap, device
# groups). It ends by exec'ing `sleep`, so PID 1 becoming `sleep` means ready.
# A shell opened before that would miss the GPU/device groups.
for _ in $(seq 1 30); do
  if [ "$(docker compose exec -T "$SERVICE" cat /proc/1/comm 2>/dev/null)" = "sleep" ]; then
    break
  fi
  sleep 1
done

exec docker compose exec -u rover "$SERVICE" bash
