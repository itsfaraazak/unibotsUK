#!/usr/bin/env bash
# ============================================================================
# deploy/run.sh — launch the match stack from runtime.env.
# ============================================================================
# Shared by the Docker entrypoint and the systemd unit. Sources the ROS distro
# + the built workspace, then runs match.launch.py with the runtime args.
# ----------------------------------------------------------------------------
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$HERE/build.env"          # ROS_DISTRO

# runtime.env may be supplied by the environment (compose) or read from disk.
if [ -f "${RUNTIME_ENV:-$HERE/runtime.env}" ]; then
  # shellcheck disable=SC1090
  set -a; source "${RUNTIME_ENV:-$HERE/runtime.env}"; set +a
fi

WS="${WS:-$(cd "$HERE/.." && pwd)/unibots_ws}"

# ROS setup scripts reference unbound vars — disable nounset across the source.
set +u
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"
# shellcheck disable=SC1091
source "${WS}/install/setup.bash"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

exec ros2 launch unibots_bt match.launch.py \
  home_zone:="${HOME_ZONE:-north}" \
  controller:="${CONTROLLER:-mpc}" \
  hardware:="${HARDWARE:-true}" \
  camera:="${CAMERA:-true}" \
  localization:="${LOCALIZATION:-true}" \
  use_sim_time:="${USE_SIM_TIME:-false}"
