#!/usr/bin/env bash
# Build the dev image locally (normally you just pull it: ./docker/update.sh).
#   ./docker/build.sh          CPU image  -> ghcr.io/iitbmartian/rover-dev:jazzy
#   ./docker/build.sh zed      ZED image  -> ghcr.io/iitbmartian/rover-dev:jazzy-zed
set -euo pipefail
cd "$(dirname "$0")"
case "${1:-cpu}" in
  cpu) docker compose build dev ;;
  zed) docker compose build dev-zed ;;
  *)   echo "usage: $0 [cpu|zed]" >&2; exit 1 ;;
esac
