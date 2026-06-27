# Prebuilt aarch64 artifact — `unibots-ws-arm64.tar.gz`

Cross-compiled `install/` tree for the **Raspberry Pi 5 / Ubuntu 26.04 (aarch64)**,
ROS 2 `lyrical`. Every C++ node built with `-O3 -mcpu=cortex-a76+crypto
-mtune=cortex-a76 -flto`. **No Docker and no Nix on the Pi** — the Pi only needs the
ROS 2 `lyrical` apt packages installed; this tarball drops the compiled workspace on top.

Contents: `install/` for `unibots_msgs`, `unibots_bt`, `unibots_spatial_memory`,
`unibots_control`, `unibots_camera`, `unibots_localization`.

> **Not included: `unibots_perception`.** It needs `ncnn`, which is not an apt package
> on Ubuntu 26.04 arm64 — it must be built from source on the Pi (the Nix dev shell
> builds it too). Build perception separately on the Pi once ncnn is installed, or run
> the stack with `camera:=false`/perception disabled for a hardware/motion bring-up test.

## Use on the Pi

```bash
# 1) ROS 2 lyrical from apt (one time) — see deploy/packages.apt for the full list
sudo apt-get update
sudo apt-get install -y ros-lyrical-ros-base ros-lyrical-rosidl-default-runtime \
    python3-numpy libeigen3-dev   # + the rest of deploy/packages.apt

# 2) Drop the prebuilt workspace
mkdir -p ~/unibots_ws && tar -C ~/unibots_ws -xzf unibots-ws-arm64.tar.gz
#   → ~/unibots_ws/install/

# 3) Source + run
source /opt/ros/lyrical/setup.bash
source ~/unibots_ws/install/setup.bash
ros2 launch unibots_bt match.launch.py home_zone:=north hardware:=true camera:=false

# 4) Start the match (latched topic — durability flag mandatory)
ros2 topic pub --qos-durability transient_local /match/start std_msgs/msg/Bool '{data: true}' --once
```

## Provenance / reproduce

**Rebuild in one command:** `deploy/cross-build/cross-build.sh` (full method +
prerequisites in `deploy/cross-build/README.md`).

Built via QEMU user-mode emulation (binfmt aarch64) in a bubblewrap sandbox over an
Ubuntu 26.04 arm64 rootfs, using `deploy/build.sh` flags. Two emulation-only quirks
were handled (neither affects a native Pi build):
- sandbox launched `--clearenv` + minimal `PATH` so the rootfs aarch64 gcc is used, not
  a host toolchain; the workspace must be configured clean (no stale `CMakeCache.txt`
  pinning a host compiler).
- Debian's split `python3-numpy` is not importable under qemu, so `FindPython3`'s NumPy
  probe was fed the header dir directly
  (`-DPython3_NumPy_INCLUDE_DIR=/usr/lib/aarch64-linux-gnu/python3-numpy/numpy/_core/include`).

Native build on the Pi (`deploy/build.sh`, or the Ansible path) needs none of this and
also builds `unibots_perception` once `ncnn` is present.
