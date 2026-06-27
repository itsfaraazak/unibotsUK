#!/usr/bin/env bash
# Docker entrypoint — hand off to the shared launcher (reads runtime.env from
# the environment injected by docker-compose). Any args run instead (e.g. bash).
set -euo pipefail

if [ "$#" -gt 0 ]; then
  exec "$@"
fi

exec /opt/unibots/deploy/run.sh
