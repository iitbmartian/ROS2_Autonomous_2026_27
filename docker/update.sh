#!/usr/bin/env bash
# Get the newest published image and restart your container on it.
# Run this after `git pull` whenever the team says the image changed
# (i.e. someone changed docker/ or a package.xml).
#   ./docker/update.sh [dev|dev-nvidia|dev-zed]
set -euo pipefail
SERVICE="${1:-dev}"
cd "$(dirname "$0")"
docker compose pull "$SERVICE"
docker compose up -d --force-recreate "$SERVICE"
echo "Updated. Open a shell with ./docker/exec.sh $SERVICE, then rebuild with 'cb'."
