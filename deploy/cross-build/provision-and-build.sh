#!/bin/bash
# ============================================================================
# provision-and-build.sh — runs INSIDE the aarch64 sandbox (rootfs mounted as /).
# ============================================================================
# Adds the ROS apt repo, extracts the full dependency closure WITHOUT running any
# maintainer scripts (they crash under QEMU user emulation), then builds the
# workspace with the Pi 5 flags and tars the install/ tree.
#
# Invoked by cross-build.sh via bwrap; not meant to be run directly on a host.
# Expects bind mounts: /ws (workspace), /deploy (this deploy/ dir), /artifacts (out).
# ----------------------------------------------------------------------------
set -eo pipefail
export DEBIAN_FRONTEND=noninteractive
export PATH=/usr/sbin:/usr/bin:/sbin:/bin

source /deploy/build.env                       # ROS_DISTRO (+ the Ubuntu codename below)
APT="-o APT::Sandbox::User=root -o Acquire::Retries=3 --no-install-recommends -y"

# ROS repo. [trusted=yes] skips gpg — gnupg's postinst crashes under emulation, and
# we never run postinsts anyway (extraction only). UBUNTU_CODENAME pins the suite
# (Ubuntu 26.04 = "resolute"); ROS publishes ros2/ubuntu <codename> main.
echo "### add ROS repo (${UBUNTU_CODENAME:-resolute})"
echo "deb [trusted=yes] http://packages.ros.org/ros2/ubuntu ${UBUNTU_CODENAME:-resolute} main" \
  > /etc/apt/sources.list.d/ros2.list
apt-get $APT update

# Build the package set from deploy/packages.apt. ncnn is dropped — it is not an apt
# package on Ubuntu arm64 (build from source on the Pi); rosdep/vcstool are host tools.
PKGS=$(sed "s/@ROS_DISTRO@/${ROS_DISTRO}/g" /deploy/packages.apt \
        | sed 's/#.*//; s/[[:space:]]*$//' | awk 'NF' \
        | grep -vE 'rosdep|vcstool|libncnn-dev')

echo "### download dependency closure (no install, no scripts)"
apt-get $APT install --download-only $PKGS

echo "### extract every .deb via dpkg-deb -x (zero maintainer scripts)"
cd /var/cache/apt/archives
n=0; for d in *.deb; do dpkg-deb -x "$d" / && n=$((n+1)); done
echo "    extracted $n debs"
ldconfig || true

# EMULATION-ONLY: Debian's split python3-numpy does not import under QEMU, so
# FindPython3's NumPy probe fails. Feed the header dir directly. build.sh appends
# EXTRA_CMAKE_ARGS to its cmake args. (A native Pi build leaves this unset.)
NPY=/usr/lib/aarch64-linux-gnu/python3-numpy/numpy/_core/include
export EXTRA_CMAKE_ARGS="-DPython3_NumPy_INCLUDE_DIR=${NPY} -DPython3_NumPy_INCLUDE_DIRS=${NPY}"

echo "### build workspace (Pi 5 optimized) — msgs first, then the rest"
WS=/ws bash /deploy/build.sh

echo "### tar the install/ tree"
tar -C /ws -czf /artifacts/unibots-ws-arm64.tar.gz install
touch /artifacts/BUILD_OK
echo "### DONE — /artifacts/unibots-ws-arm64.tar.gz"
