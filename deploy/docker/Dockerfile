# ============================================================================
# deploy/docker/Dockerfile — Pi 5 (aarch64) competition image, Ubuntu 26.04.
# ============================================================================
# Reproducible: pin BASE_DIGEST to freeze the exact Ubuntu 26.04 snapshot.
# Build (on the Pi 5, or via buildx for arm64):
#   docker buildx build --platform linux/arm64 -f deploy/docker/Dockerfile \
#       -t unibots:latest --load .
# All optimization is driven by deploy/build.env (shared with Ansible).
# ----------------------------------------------------------------------------

ARG BASE_IMAGE=ubuntu:26.04
# Pin by digest for byte-reproducibility, e.g.:
#   ARG BASE_DIGEST=@sha256:....
ARG BASE_DIGEST=

# ─────────────────────────── Stage 1: builder ──────────────────────────────
FROM ${BASE_IMAGE}${BASE_DIGEST} AS builder

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# build.env is the single source of truth for ROS_DISTRO + Pi5 flags.
COPY deploy/build.env /deploy/build.env
COPY deploy/packages.apt /deploy/packages.apt

# Add the ROS 2 apt repository for Ubuntu 26.04, then install all deps.
RUN source /deploy/build.env \
 && apt-get update \
 && apt-get install -y --no-install-recommends \
        ca-certificates curl gnupg lsb-release software-properties-common \
 && add-apt-repository -y universe \
 && curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
        http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/ros2.list \
 && apt-get update \
 && sed "s/@ROS_DISTRO@/${ROS_DISTRO}/g" /deploy/packages.apt \
        | sed 's/#.*//; s/[[:space:]]*$//' | awk 'NF' \
        | xargs apt-get install -y --no-install-recommends \
 && rm -rf /var/lib/apt/lists/*

# Copy the source and build with the shared, optimized build script.
WORKDIR /opt/unibots
COPY deploy/ ./deploy/
COPY unibots_ws/src ./unibots_ws/src
RUN WS=/opt/unibots/unibots_ws bash /opt/unibots/deploy/build.sh

# ───────────────────────── Stage 2: runtime ────────────────────────────────
# Same base, runtime libs only (no -dev / compilers) → smaller, immutable image.
FROM ${BASE_IMAGE}${BASE_DIGEST} AS runtime

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

COPY deploy/build.env /deploy/build.env
COPY deploy/packages.apt /deploy/packages.apt

# Install only the ROS runtime + shared libs (strip build-only packages).
RUN source /deploy/build.env \
 && apt-get update \
 && apt-get install -y --no-install-recommends \
        ca-certificates curl gnupg lsb-release \
 && curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
        http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/ros2.list \
 && apt-get update \
 && sed "s/@ROS_DISTRO@/${ROS_DISTRO}/g" /deploy/packages.apt \
        | sed 's/#.*//; s/[[:space:]]*$//' | awk 'NF' \
        | grep -vE 'colcon|rosdep|vcstool|build-essential|^cmake$|ninja-build|^git$' \
        | xargs apt-get install -y --no-install-recommends \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/unibots
COPY deploy/ ./deploy/
# Bring over the built workspace (install + src for incremental match-day edits).
COPY --from=builder /opt/unibots/unibots_ws/install ./unibots_ws/install
COPY --from=builder /opt/unibots/unibots_ws/build   ./unibots_ws/build
COPY unibots_ws/src ./unibots_ws/src

ENTRYPOINT ["/opt/unibots/deploy/docker/entrypoint.sh"]
