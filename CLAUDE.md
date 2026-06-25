# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Environment Setup

```bash
nix develop          # enter nix shell (provides colcon, cmake, ncnn, opencv, etc.)
```

The `flake.nix` uses `nix-ros-overlay` with ROS distro `lyrical`. The unstable nixpkgs channel provides `nono` and `claude-code`.

## Build & Run

```bash
# Build workspace
cd unibots_ws
colcon build --symlink-install

# Source and run
source install/setup.bash

# Camera node (publishes /unibots/camera/image_raw)
ros2 run unibots_camera camera_node --ros-args -p device_id:=0 -p fps:=30

# Perception node (subscribes to camera, publishes /vision/balls and /vision/obstacles)
ros2 run unibots_perception perception_node

# Debug visualiser (laptop only — saves frames to /tmp/unibots_debug/)
ros2 run unibots_camera debug_visualiser

# Run tests for a single package
colcon test --packages-select unibots_camera
colcon test-result --verbose
```

Tune perception params live without rebuilding:
```bash
ros2 param set /perception_node conf_threshold 0.35  # lower = catches more edge/partial balls
ros2 param set /perception_node conf_threshold 0.50  # higher = fewer false positives
ros2 param set /perception_node input_size 256        # 256=~30fps RPi4 (default)
ros2 param set /perception_node input_size 320        # 320=~22fps but more accurate
ros2 param set /perception_node num_threads 4
ros2 param set /perception_node hfov_deg 62.5         # set to your calibrated value
```

## Architecture

```
camera_node (Python)          →  /unibots/camera/image_raw (sensor_msgs/Image)
                                        ↓
perception_node (C++)         →  /vision/balls (BallArray)
                                  /vision/obstacles (ObstacleArray)
                                        ↓
debug_visualiser (Python)     ←  subscribes to all three (laptop debug only)
```

**`unibots_camera`** — Python/ament_python package. `CameraNode` opens a V4L2 device via OpenCV and publishes raw BGR frames.

**`unibots_perception`** — C++/ament_cmake package. `PerceptionNode` runs a YOLO11n model via NCNN (`Detector` class). Detection classes: `0=ping_pong_ball`, `1=bearing (steel ball)`, `2=robot`. Distance estimated via pinhole model using known object diameters (`hfov_deg` ROS param, calibrate per lens). Model files live in `models/` and are installed to the ament share path at build time.

**Rulebook §4.3 object specs:** ping pong 40 mm orange, bearing 20 mm polished steel, arena floor WHITE painted, 16 ping pong + 24 bearings per match.

**`unibots_msgs`** — custom message package (separate, not in this repo's src yet). Defines `BallDetection`, `BallArray`, `ObstacleDetection`, `ObstacleArray`.

## Camera FOV Calibration (REQUIRED for accurate distance/bearing)

`hfov_deg` defaults to 60°. Wrong value → wrong distances and bearings.

**Quick method** (needs a ping pong ball and a tape measure):
1. Place ball (40 mm) at exactly 1000 mm from camera
2. Capture frame, measure ball's pixel width `px_w` (use debug_visualiser output)
3. `focal_px = (1000 × px_w) / 40`
4. `hfov_deg = 2 × atan(image_width / (2 × focal_px)) × (180 / π)`

**Precise method** (OpenCV checkerboard):
```bash
# Print 8×6 checkerboard, 25mm squares
# Capture ~20 images at different angles with the competition camera
python3 -c "
import cv2, glob, numpy as np
criteria = (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
objp = np.zeros((6*8,3), np.float32)
objp[:,:2] = np.mgrid[0:8,0:6].T.reshape(-1,2) * 25
obj_pts, img_pts = [], []
for f in glob.glob('calib/*.jpg'):
    img = cv2.imread(f)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, (8,6), None)
    if ret:
        obj_pts.append(objp)
        img_pts.append(cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria))
_, K, _, _, _ = cv2.calibrateCamera(obj_pts, img_pts, gray.shape[::-1], None, None)
fx = K[0,0]; w = gray.shape[1]
import math
hfov = 2 * math.atan(w / (2*fx)) * 180 / math.pi
print(f'fx={fx:.1f}  HFOV={hfov:.2f}°')
"
```

Set the result:
```bash
ros2 run unibots_perception perception_node --ros-args -p hfov_deg:=<VALUE> -p input_size:=256
```

## Training (improve model)

Notebook at `training/train_unibots.ipynb` — run on Google Colab (free GPU).

After training, copy outputs to models directory:
```bash
cp model_256/model.ncnn.param  unibots_ws/src/unibots_perception/models/model.ncnn.param
cp model_256/model.ncnn.bin    unibots_ws/src/unibots_perception/models/model.ncnn.bin
cd unibots_ws && colcon build --packages-select unibots_perception --symlink-install
```

## Known Incomplete Areas

- `BallDetection.track_id` is always 0 — Kalman filter tracking not wired in yet
- `ObstacleDetection.world_x/world_y` are always 0 — EKF + homography not wired in yet
- `debug_visualiser` GUI display (`cv2.imshow`) is commented out; currently saves frames to `/tmp/unibots_debug/` every 10 frames instead
- Gazebo integration packages (`ros-gz*`) are commented out in `flake.nix`
