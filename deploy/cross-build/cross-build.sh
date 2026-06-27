#!/usr/bin/env bash
# ============================================================================
# cross-build.sh — produce deploy/artifacts/unibots-ws-arm64.tar.gz on an x86 host.
# ============================================================================
# Builds the Pi 5 (aarch64, Ubuntu 26.04, ROS lyrical) install/ tree WITHOUT Docker
# and WITHOUT installing anything on the Pi, using QEMU user-mode emulation inside a
# bubblewrap sandbox over an Ubuntu arm64 base rootfs.
#
# Host requirements (this is a NixOS host; adjust if yours differs):
#   - binfmt_misc registered for aarch64 with the F or P flag. On NixOS:
#         boot.binfmt.emulatedSystems = [ "aarch64-linux" ];
#     Verify: test -e /proc/sys/fs/binfmt_misc/aarch64-linux
#   - bwrap (bubblewrap) and curl/tar on PATH (nix-shell -p bubblewrap curl).
#   - /nix and /run/binfmt readable (the qemu interpreter lives in the nix store).
#
# Usage:
#   deploy/cross-build/cross-build.sh                 # bootstrap + build
#   FRESH=1 deploy/cross-build/cross-build.sh         # wipe the rootfs first
#   WORK=/path BASE_URL=... deploy/cross-build/cross-build.sh
# ----------------------------------------------------------------------------
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEPLOY="$(cd "$HERE/.." && pwd)"
REPO="$(cd "$DEPLOY/.." && pwd)"
source "$DEPLOY/build.env"                              # ROS_DISTRO

WORK="${WORK:-$HERE/.work}"
ROOTFS="$WORK/rootfs"
ARTIFACTS_OUT="$DEPLOY/artifacts"
CODENAME="${UBUNTU_CODENAME:-resolute}"                 # Ubuntu 26.04
BASE_URL="${BASE_URL:-https://cdimage.ubuntu.com/ubuntu-base/releases/26.04/release/ubuntu-base-26.04-base-arm64.tar.gz}"

# --- preflight ------------------------------------------------------------
[ -e /proc/sys/fs/binfmt_misc/aarch64-linux ] || {
  echo "ERROR: binfmt aarch64 not registered. On NixOS add:" >&2
  echo "  boot.binfmt.emulatedSystems = [ \"aarch64-linux\" ];  then nixos-rebuild" >&2
  exit 1
}
BWRAP="$(command -v bwrap)" || { echo "ERROR: bwrap not found (nix-shell -p bubblewrap)" >&2; exit 1; }

[ "${FRESH:-0}" = 1 ] && rm -rf "$ROOTFS"
mkdir -p "$ROOTFS" "$ARTIFACTS_OUT"

# --- 1) bootstrap the Ubuntu arm64 base rootfs (once) ---------------------
if [ ! -e "$ROOTFS/usr/bin/dpkg-deb" ]; then
  echo "==> fetching Ubuntu base rootfs ($CODENAME arm64)"
  base="$WORK/ubuntu-base.tar.gz"
  [ -f "$base" ] || curl -fSL "$BASE_URL" -o "$base"
  echo "==> extracting base into $ROOTFS"
  tar -C "$ROOTFS" -xzf "$base"
  # resolv.conf so apt can fetch from inside the sandbox
  cp -f /etc/resolv.conf "$ROOTFS/etc/resolv.conf" 2>/dev/null || true
fi

# --- 2) stage the repo (isolated copy — safe alongside other git work) ----
echo "==> staging workspace + deploy into rootfs"
mkdir -p "$ROOTFS/ws/src" "$ROOTFS/deploy" "$ROOTFS/artifacts"
rsync -a --delete "$REPO/unibots_ws/src/" "$ROOTFS/ws/src/"
rsync -a --delete --exclude artifacts/ --exclude cross-build/.work "$DEPLOY/" "$ROOTFS/deploy/"
rm -rf "$ROOTFS/ws/build" "$ROOTFS/ws/install" "$ROOTFS/ws/log"   # no stale CMakeCache (host-compiler leak)

# --- 3) build inside the sandbox -----------------------------------------
# --clearenv + minimal PATH so the rootfs aarch64 gcc/cmake/colcon are used, NOT a
# host x86 toolchain. /nix + /run/binfmt stay bound for the qemu interpreter.
echo "==> building (QEMU emulation; ~15-30 min)"
rm -f "$ROOTFS/artifacts/BUILD_OK"
"$BWRAP" \
  --clearenv \
  --setenv PATH /usr/sbin:/usr/bin:/sbin:/bin \
  --setenv HOME /root --setenv TMPDIR /tmp \
  --setenv LANG C.UTF-8 --setenv LC_ALL C.UTF-8 --setenv TERM xterm \
  --setenv DEBIAN_FRONTEND noninteractive \
  --unshare-user --uid 0 --gid 0 \
  --bind "$ROOTFS" / \
  --dev-bind /dev /dev --proc /proc --tmpfs /tmp \
  --ro-bind /nix /nix --ro-bind /run/binfmt /run/binfmt \
  --chdir / /bin/bash /deploy/cross-build/provision-and-build.sh

# --- 4) collect the artifact ---------------------------------------------
[ -e "$ROOTFS/artifacts/BUILD_OK" ] || { echo "ERROR: build did not complete" >&2; exit 1; }
cp -f "$ROOTFS/artifacts/unibots-ws-arm64.tar.gz" "$ARTIFACTS_OUT/"
echo "==> DONE: $ARTIFACTS_OUT/unibots-ws-arm64.tar.gz"
ls -la "$ARTIFACTS_OUT/unibots-ws-arm64.tar.gz"
