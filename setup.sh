#!/usr/bin/env bash

add-apt-repository -y universe
apt update
apt install -y software-properties-common curl

# Install ROS2
apt update && apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
dpkg -i /tmp/ros2-apt-source.deb

apt update && apt install -y ros-dev-tools

apt install -y ros-lyrical-desktop

# Install gazebo
apt install -y ros-lyrical-ros-gz

# -----
# --- python ---
apt-get install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-pip \
  python3-argcomplete

pip3 install --break-system-packages \
  ultralytics \
  supervision \
  opencv-python \
  adafruit-circuitpython-bno055 \
  numpy \
  scipy \
  cvxpy \
  cvxpygen \
  osqp

pip3 install py_trees py_trees_ros

# --- Nav2 full stack ---
apt-get install -y \
  ros-lyrical-nav2-bringup \
  ros-lyrical-nav2-core \
  ros-lyrical-nav2-bt-navigator \
  ros-lyrical-nav2-mppi-controller \
  ros-lyrical-nav2-costmap-2d \
  ros-lyrical-nav2-navfn-planner \
  ros-lyrical-nav2-lifecycle-manager \
  ros-lyrical-nav2-recoveries

# --- Localisation ---
apt-get install -y \
  ros-lyrical-robot-localization \
  ros-lyrical-apriltag-ros

# --- ros2_control ---
apt-get install -y \
  ros-lyrical-ros2-control \
  ros-lyrical-ros2-controllers \
  ros-lyrical-gz-ros2-control

# --- Robot description ---
apt-get install -y \
  ros-lyrical-xacro \
  ros-lyrical-joint-state-publisher \
  ros-lyrical-joint-state-publisher-gui \
  ros-lyrical-robot-state-publisher

# --- Vision ---
apt-get install -y \
  ros-lyrical-cv-bridge \
  ros-lyrical-vision-msgs \
  ros-lyrical-image-transport \
  ros-lyrical-camera-calibration \
  ros-lyrical-camera-info-manager

# --- TF + tooling ---
apt-get install -y \
  ros-lyrical-tf2-tools \
  ros-lyrical-tf2-ros \
  ros-lyrical-tf2-geometry-msgs \
  ros-lyrical-teleop-twist-keyboard \
  ros-lyrical-twist-mux

# --- Gazebo Harmonic + bridge ---
apt-get install -y \
  ros-lyrical-ros-gz \
  ros-lyrical-ros-gz-bridge \
  ros-lyrical-ros-gz-sim

# --- Python packages ---
pip3 install --break-system-packages \
  ultralytics \
  supervision \
  opencv-python \
  adafruit-circuitpython-bno055 \
  numpy \
  scipy

# --- rosdep init ---
rosdep init || true   # 'true' so it doesn't fail if already initialised
rosdep update

# --- Auto-source ROS 2 in every shell inside the container ---
echo "source /opt/ros/lyrical/setup.bash" >> /etc/bash.bashrc
echo "source ~/unibots_ws/install/setup.bash 2>/dev/null || true" >> /etc/bash.bashrc

echo "=== Unibots environment ready ==="

