# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Environment Setup

```bash
nix develop          # enter nix shell (provides colcon, cmake, ncnn, opencv, etc.)
```

The `flake.nix` uses `nix-ros-overlay` with ROS distro `lyrical`. The unstable nixpkgs channel provides `nono` and `claude-code`.

## Build & Run

```bash
# Build workspace (msgs first, then everything else)
cd unibots_ws
colcon build --packages-select unibots_msgs
colcon build

# Source
source install/setup.bash

# Match day — single launch entry point
ros2 launch unibots_game match_day.launch.py home_zone:=north use_sim_time:=false

# Start the match after launch
ros2 topic pub /match/start std_msgs/msg/Bool '{data: true}' --once

# Monitor
ros2 topic echo /game/state
ros2 topic echo /spatial_memory/ball_map --no-arr
ros2 topic echo /spatial_memory/prediction_error
ros2 topic echo /game/target

# Debug visualiser (laptop only — saves frames to /tmp/unibots_debug/)
ros2 run unibots_camera debug_visualiser
```

## Tuning Live (no rebuild required)

```bash
# Perception
ros2 param set /perception_node conf_threshold 0.40
ros2 param set /perception_node input_size 256        # 256=~30fps RPi4
ros2 param set /perception_node input_size 320        # 320=~22fps but more accurate
ros2 param set /perception_node hfov_deg 62.5         # MUST calibrate per lens

# Spatial memory prediction
ros2 param set /spatial_memory_node prediction_mode none
ros2 param set /spatial_memory_node prediction_mode constant_velocity
ros2 param set /spatial_memory_node prediction_mode friction   # default

# BT game
ros2 param set /bt_game_node use_predicted_position true
ros2 param set /bt_game_node use_predicted_position false      # revert to current pos
ros2 param set /bt_game_node home_zone north
```

All tunable parameters are documented in `TUNING.md` with physical meaning and safe ranges.

## Architecture

```
camera_node (Python)          →  /unibots/camera/image_raw (sensor_msgs/Image)
                                        ↓
perception_node (C++)         →  /vision/balls (BallArray)
                                  /vision/obstacles (ObstacleArray)
                                        ↓
spatial_memory_node (C++)     →  /spatial_memory/ball_map (BallMap)
                                  /spatial_memory/prediction_error (Float32)
                                  /spatial_memory/debug_markers (MarkerArray)
                                        ↓
bt_game_node (C++)            →  /game/target (PoseStamped)
                                  /game/state (String)
                                  /servo/command (String)
                                  /game/ball_collected (UInt32)
                                        ↓
apf_controller_node / mpc_controller_node → /cmd_vel (TwistStamped)
```

**`unibots_camera`** — Python/ament_python. `CameraNode` opens a V4L2 device via OpenCV, publishes raw BGR frames.

**`unibots_perception`** — C++/ament_cmake. `PerceptionNode` runs YOLO11n via NCNN. Classes: `0=ping_pong_ball`, `1=bearing (steel ball)`, `2=robot`. Distance via pinhole model. Config: `config/perception.yaml`.

**`unibots_spatial_memory`** — C++/ament_cmake. Persistent Kalman ball tracker (state [x,y,vx,vy], Eigen3). Separate noise params for ping pong vs steel. Motion gating, switchable prediction modes (none/constant_velocity/friction). Density scoring. Publishes predicted ball positions and prediction error metric. Config: `config/spatial_memory.yaml`.

**`unibots_bt`** — C++/ament_cmake. Behaviour tree game controller. Reads `/spatial_memory/ball_map` for smart target selection (uses predicted positions). Full tree: STOP → PARK → DEPOSIT(nav+align+dump) → HUNT(capture/servo/approach) → SEARCH. State published only on change. Config: `config/bt_game.yaml`.

**`unibots_msgs`** — Custom message package. Defines `BallDetection`, `BallArray`, `ObstacleDetection`, `ObstacleArray`, `WorldBall`, `BallMap`.

**`unibots_game`** — Python/ament_python. Contains `game_state_node.py` (legacy FSM, reads `/spatial_memory/ball_map`) and launch files including `match_day.launch.py`.

**`unibots_control`** — Python. APF and MPC controllers. Both now correctly subscribe to `/vision/obstacles` as `ObstacleArray` (was incorrectly `Detection2DArray` — bug fixed).

**Rulebook §4.3 object specs:** ping pong 40 mm orange, bearing 20 mm polished steel, arena floor WHITE painted, 16 ping pong + 24 bearings per match.

## CMake Notes (Nix-specific)

- Do NOT use `ament_target_dependencies()` — it's not exported in ROS2 Lyrical. Use `target_link_libraries` with `${package_TARGETS}` variables instead (see `unibots_perception/CMakeLists.txt` as reference).
- `rclcpp::Time` has no `to_msg()` in this distro — use implicit conversion: `header.stamp = now()`.
- `rclcpp::Duration` same: `marker.lifetime = rclcpp::Duration::from_seconds(0.5)`.
- Eigen3 needs explicit HINTS path: `find_package(Eigen3 REQUIRED NO_MODULE HINTS "/nix/store/kki9hn7p0dc4186z31z5qz0kvaxmjk0s-eigen-3.4.1/share/eigen3/cmake")`.

## Camera FOV Calibration (REQUIRED for accurate distance/bearing)

`hfov_deg` defaults to 62.5°. Wrong value → wrong distances and bearings.

**Quick method** (ping pong ball + tape measure):
1. Place ball (40 mm) at exactly 1000 mm from camera
2. Capture frame, measure ball's pixel width `px_w` (use debug_visualiser output)
3. `focal_px = (1000 × px_w) / 40`
4. `hfov_deg = 2 × atan(image_width / (2 × focal_px)) × (180 / π)`

Set the result:
```bash
ros2 param set /perception_node hfov_deg <VALUE>
```

## Training (improve model)

Notebook at `training/train_unibots.ipynb` — run on Google Colab (free GPU).

After training:
```bash
cp model_256/model.ncnn.param  unibots_ws/src/unibots_perception/models/model.ncnn.param
cp model_256/model.ncnn.bin    unibots_ws/src/unibots_perception/models/model.ncnn.bin
cd unibots_ws && colcon build --packages-select unibots_perception
```

## Known Incomplete Areas

- `BallDetection.track_id` is always 0 — perception-level Kalman unused; `spatial_memory_node` assigns its own track IDs
- `ObstacleDetection.world_x/world_y` are always 0 — EKF + homography not wired; obstacle world position is computed in APF/MPC callbacks from `bearing_deg` + `distance_m` + robot pose
- `debug_visualiser` GUI display (`cv2.imshow`) is commented out; saves frames to `/tmp/unibots_debug/` every 10 frames
- Gazebo integration packages (`ros-gz*`) are commented out in `flake.nix`
- ALIGN_NET is timeout-only — AprilTag lateral/yaw correction exists in old FSM but not yet ported to C++ BT
