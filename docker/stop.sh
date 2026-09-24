#!/usr/bin/env bash
# Stop and remove the dev containers (your code and build/ stay in the repo).
set -euo pipefail
cd "$(dirname "$0")"
docker compose down
