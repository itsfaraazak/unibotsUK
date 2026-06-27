#!/usr/bin/env bash
# ============================================================================
# deploy/docker/buildx.sh — CROSS-COMPILE the Pi 5 image IN ADVANCE on x86.
# ============================================================================
# Builds the aarch64 (Cortex-A76 optimized) competition image on a dev laptop
# using Docker buildx + QEMU emulation, then saves a loadable tarball you copy
# to the Pi 5. No compiling on the robot — match-day boot is just `docker load`
# + `compose up`.
#
#   ./deploy/docker/buildx.sh                 # build + save deploy/artifacts/unibots-pi5-arm64.tar
#   PUSH=1 IMAGE=ghcr.io/you/unibots:pi5 ./deploy/docker/buildx.sh   # push to a registry instead
#
# On the Pi 5 afterwards:
#   docker load -i unibots-pi5-arm64.tar
#   docker compose -f deploy/docker/docker-compose.yml --env-file deploy/runtime.env up -d
# ----------------------------------------------------------------------------
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"

IMAGE="${IMAGE:-unibots:pi5}"
PLATFORM="${PLATFORM:-linux/arm64}"
ARTIFACT_DIR="${ARTIFACT_DIR:-$ROOT/deploy/artifacts}"
ARTIFACT="${ARTIFACT:-$ARTIFACT_DIR/unibots-pi5-arm64.tar}"
PUSH="${PUSH:-0}"

command -v docker >/dev/null || { echo "docker not found"; exit 1; }

# 1) Register QEMU binfmt handlers so x86 can run/build arm64 containers.
echo "==> Registering QEMU binfmt (arm64 emulation)…"
docker run --privileged --rm tonistiigi/binfmt --install arm64 >/dev/null

# 2) Create a buildx builder once (idempotent).
if ! docker buildx inspect unibots-builder >/dev/null 2>&1; then
  echo "==> Creating buildx builder 'unibots-builder'…"
  docker buildx create --name unibots-builder --driver docker-container --use >/dev/null
else
  docker buildx use unibots-builder
fi
docker buildx inspect --bootstrap >/dev/null

# 3) Cross-build the aarch64 image (build.sh inside runs with -mcpu=cortex-a76).
echo "==> Cross-building ${IMAGE} for ${PLATFORM} (QEMU — this is slow)…"
if [ "$PUSH" = "1" ]; then
  docker buildx build --platform "$PLATFORM" \
    -f "$ROOT/deploy/docker/Dockerfile" -t "$IMAGE" --push "$ROOT"
  echo "==> Pushed ${IMAGE}"
else
  mkdir -p "$ARTIFACT_DIR"
  # --load only supports a single platform; emit a docker-format tar artifact.
  docker buildx build --platform "$PLATFORM" \
    -f "$ROOT/deploy/docker/Dockerfile" -t "$IMAGE" \
    --output "type=docker,dest=$ARTIFACT" "$ROOT"
  echo "==> Saved ${ARTIFACT}"
  echo "    Copy to the Pi 5, then:  docker load -i $(basename "$ARTIFACT")"
fi
