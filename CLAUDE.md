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

# Match day — single launch entry point (lives in unibots_bt)
# Wires camera → perception → spatial_memory → bt_game → control (+ hardware, + localization).
ros2 launch unibots_bt match.launch.py home_zone:=north use_sim_time:=false
#   controller:=mpc|apf   hardware:=true|false   camera:=true|false   localization:=true|false

# Game controller alone (dev / bench testing, no perception or hardware)
ros2 launch unibots_bt bt_game.launch.py home_zone:=north

# Start the match after launch.
# NOTE: /match/start uses a LATCHED (transient_local) subscriber — a manual
# publish MUST match its durability or the message is dropped.
ros2 topic pub --qos-durability transient_local /match/start std_msgs/msg/Bool '{data: true}' --once

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
                                        ↓
hardware_motor_node  ← /cmd_vel          (real robot only)
hardware_servo_node  ← /servo/command    (real robot only)
```

**`/servo/command` (String) table:** `SCOOP` = claw grab (fired by `FireScoop` on
capture), `OPEN` = trapdoor open / dump hopper into net (fired by `Deposit`),
`CLOSE` = trapdoor return.

**`unibots_camera`** — Python/ament_python. `CameraNode` opens a V4L2 device via OpenCV, publishes raw BGR frames.

**`unibots_perception`** — C++/ament_cmake. `PerceptionNode` runs YOLO11n via NCNN. Classes: `0=ping_pong_ball`, `1=bearing (steel ball)`, `2=robot`. Distance via pinhole model. Config: `config/perception.yaml`.

**`unibots_spatial_memory`** — C++/ament_cmake. Persistent Kalman ball tracker (state [x,y,vx,vy], Eigen3). Separate noise params for ping pong vs steel. Motion gating, switchable prediction modes (none/constant_velocity/friction). Density scoring. Publishes predicted ball positions and prediction error metric. Config: `config/spatial_memory.yaml`.

**`unibots_bt`** — C++/ament_cmake. Behaviour tree game controller, rewritten on
**BehaviorTree.CPP v4** (the tree is XML, Groot-visualisable). Structure:
- `bt/game_tree.xml` — a `ReactiveFallback` priority checklist re-read at 20 Hz:
  `PRE-START → TIME-UP → ENDGAME(nav→dump→park) → DUMP-WHEN-FULL → HUNT(select→capture/approach) → SEARCH → FAILSAFE`.
- `include/unibots_bt/game_context.hpp` — `GameContext`, the shared blackboard state
  (pose, ball map, counters, latches) + the publish helpers. The only motion output is
  `/game/target`; **it never publishes `/cmd_vel`** (that is the controller's job).
- `include/unibots_bt/bt_nodes.hpp` — the custom leaf nodes (conditions + stateful
  actions) and `registerNodes()`.
- `src/bt_game_node.cpp` — ROS plumbing: declares params, fills the context from
  subscriptions, loads the XML, ticks it.

Target selection picks the highest-utility ball
(`value × density × visibility ÷ distance`) — `value` uses the rulebook points
(`ping_pong`=4, `steel`=2). Capture is **radius-based**: within `capture_radius_m` of
the target → `SCOOP` + publish `track_id` on `/game/ball_collected` (spatial_memory then
marks it `COLLECTED`). State is published only on change. Config: `config/bt_game.yaml`.
Launch: `match.launch.py` (full stack) and `bt_game.launch.py` (node alone).

> Ball-type strings are `"ping_pong"` / `"steel"` (as published by perception /
> spatial_memory) — NOT `"ping_pong_ball"`. The old hand-rolled BT compared against
> the wrong string; the rewrite uses the correct ones.

**`unibots_msgs`** — Custom message package. Defines `BallDetection`, `BallArray`, `ObstacleDetection`, `ObstacleArray`, `WorldBall`, `BallMap`.

**`unibots_game`** — REMOVED. The legacy Python FSM and its `match_day.launch.py` were
deleted; the top-level entry point is now `unibots_bt/launch/match.launch.py`.

**`unibots_control`** — Python. APF and MPC controllers. Both now correctly subscribe to `/vision/obstacles` as `ObstacleArray` (was incorrectly `Detection2DArray` — bug fixed).

**Rulebook §4.3 object specs:** ping pong 40 mm orange, bearing 20 mm polished steel, arena floor WHITE painted, 16 ping pong + 24 bearings per match.

## CMake Notes (Nix-specific)

- Do NOT use `ament_target_dependencies()` — it's not exported in ROS2 Lyrical. Use `target_link_libraries` with `${package_TARGETS}` variables instead (see `unibots_perception/CMakeLists.txt` as reference).
- `rclcpp::Time` has no `to_msg()` in this distro — use implicit conversion: `header.stamp = now()`.
- `rclcpp::Duration` same: `marker.lifetime = rclcpp::Duration::from_seconds(0.5)`.
- Eigen3 needs explicit HINTS path: `find_package(Eigen3 REQUIRED NO_MODULE HINTS "/nix/store/kki9hn7p0dc4186z31z5qz0kvaxmjk0s-eigen-3.4.1/share/eigen3/cmake")`.
- BehaviorTree.CPP (added to `flake.nix` as `behaviortree-cpp`, resolves to v4.9.0) exports
  the target **`behaviortree_cpp::behaviortree_cpp`** — NOT `BT::behaviortree_cpp`. Link it
  directly (no `ament_target_dependencies`). See `unibots_bt/CMakeLists.txt`.

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
- Net deposit/park alignment is open-loop — the BT drives to the `home_zone` pose and
  pushes; AprilTag lateral/yaw correction at the wall is not yet wired into the C++ BT.
- The new BT capture is **radius-based** (`capture_radius_m`), not ToF/coast — the old
  `/sensors/tof_distance` blind-spot capture and the in-place yaw search were dropped
  (the holonomic controller faces its travel direction, so SEARCH is a moving patrol).
- **No wheel odometry / IMU on the real robot** — the EKF (`ekf.yaml` `odom0:/odom`)
  gets nothing; `/odom/filtered` only updates when an AprilTag is seen, which used to
  freeze the bot at start (no pose → controllers publish zero → motors never spin).
  Worked around by an **open-loop SEARCH bootstrap** in the MPC/APF controllers
  (drives a blind scan on `/cmd_vel` while `/game/state == "SEARCH"` and pose is
  missing, so the camera can find a tag and seed localization). Real fix = add
  encoders→`/odom` or an IMU. Full write-up: `docs/LOCALIZATION_ODOM_ISSUE.md`.
- Physical start button: `match_button_node` (GPIO 4, `unibots_control`) latches
  `/match/start`; launched with `hardware:=true`.
