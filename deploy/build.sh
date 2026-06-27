#!/usr/bin/env bash
# ============================================================================
# deploy/build.sh — build the Unibots workspace with Pi 5 max optimization.
# ============================================================================
# Used identically by the Docker image build and the Ansible playbook, so the
# binary is the same however you deploy. Reads all tunables from build.env.
#
# Usage:
#   deploy/build.sh                  # build in ../unibots_ws
#   WS=/path/to/unibots_ws deploy/build.sh
# ----------------------------------------------------------------------------
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$HERE/build.env"

WS="${WS:-$(cd "$HERE/.." && pwd)/unibots_ws}"

# ---- Assemble the compiler / linker flags from build.env -----------------
CXXFLAGS="${OPT_LEVEL} -mcpu=${PI5_CPU} -mtune=${PI5_TUNE} ${EXTRA_CXX_FLAGS}"
LDFLAGS="-Wl,-O2 -Wl,--as-needed"
IPO_ARG=()

if [ "${ENABLE_LTO:-1}" = "1" ]; then
  CXXFLAGS="${CXXFLAGS} -flto=auto"
  LDFLAGS="${LDFLAGS} -flto=auto"
  IPO_ARG=(-DCMAKE_INTERPROCEDURAL_OPTIMIZATION=ON)
fi

if [ "${ENABLE_FAST_MATH:-0}" = "1" ]; then
  echo "WARNING: -ffast-math enabled — Kalman/EKF accuracy is NOT guaranteed."
  CXXFLAGS="${CXXFLAGS} -ffast-math"
fi

echo "==> ROS_DISTRO = ${ROS_DISTRO}"
echo "==> CXXFLAGS   = ${CXXFLAGS}"
echo "==> LDFLAGS    = ${LDFLAGS}"
echo "==> Workspace  = ${WS}"

# ROS setup scripts reference unbound vars — disable nounset across the source.
set +u
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u
cd "$WS"

# Packages that do not build / are not part of the competition runtime.
SKIP=(unibots_description unibots_sim unibots_behavior_tree)

COMMON_CMAKE_ARGS=(
  -DCMAKE_BUILD_TYPE=Release
  -DCMAKE_C_FLAGS="${CXXFLAGS}"
  -DCMAKE_CXX_FLAGS="${CXXFLAGS}"
  -DCMAKE_EXE_LINKER_FLAGS="${LDFLAGS}"
  -DCMAKE_SHARED_LINKER_FLAGS="${LDFLAGS}"
  "${IPO_ARG[@]}"
  --no-warn-unused-cli
)

# Extra cmake args, space-separated. Empty for a native Pi build. The emulated
# cross-build injects the Debian NumPy header hint here (see deploy/cross-build/).
if [ -n "${EXTRA_CMAKE_ARGS:-}" ]; then
  # shellcheck disable=SC2206
  COMMON_CMAKE_ARGS+=(${EXTRA_CMAKE_ARGS})
fi

# 1) messages first (everything else depends on them) — per CLAUDE.md.
colcon build --packages-select unibots_msgs \
  --cmake-args "${COMMON_CMAKE_ARGS[@]}"

# 2) the rest of the competition stack, fully optimized.
colcon build --packages-skip "${SKIP[@]}" \
  --cmake-args "${COMMON_CMAKE_ARGS[@]}"

echo "==> Build complete. Source: ${WS}/install/setup.bash"
