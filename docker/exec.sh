#!/usr/bin/env bash
# Open another shell in the running container (e.g. for teleop next to a launch).
#   ./docker/exec.sh [dev|dev-nvidia|dev-zed]
set -euo pipefail
SERVICE="${1:-dev}"
cd "$(dirname "$0")"
if [ -z "$(docker compose ps -q "$SERVICE")" ]; then
  echo "'$SERVICE' is not running. Start it with ./docker/run.sh $SERVICE" >&2
  exit 1
fi
exec docker compose exec -u rover "$SERVICE" bash
