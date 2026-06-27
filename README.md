# UnibotsUK 2025–2026 — Competition Robot

Fully autonomous competition robot built for the **Unibots UK 2025–2026** season.

---

## Competition Overview

The robot competes on a **2000 × 2000 mm arena** with four teams simultaneously.
Each match lasts **180 seconds**. The objective is to maximise game points:

| Scoring action | Points |
|----------------|--------|
| Each ping-pong ball (40 mm orange) in your net | **4 pts** |
| Each steel ball-bearing (20 mm magnetic) in your net | **2 pts** |
| Each ball-bearing held by the robot at end of match | **1 pt** |
| Robot touching its home wall at end of match (parking bonus) | **3 pts** |

At the start of each match the team is assigned one of four zones — **North, East, South, West** — identified by a coloured wall segment and a set of AprilTag fiducials (IDs 0–23). The deposit net hangs *outside* the arena wall, so balls must be launched or ejected through/over the wall.

There are **16 ping-pong balls** and **24 steel ball-bearings** distributed approximately rotationally symmetric across the white arena floor at the start of each match.

---

## Robot Strategy

The scoring system rewards deposited balls far more than held balls.
Our strategy maximises net deposits while guaranteeing the parking bonus:

1. **Utility-based target selection** — each known ball is scored
   `value × (1 + density_bonus·density) × visibility ÷ distance`, where `value`
   is the rulebook points (ping-pong = 4, steel = 2). The highest-utility ball wins,
   so the robot prefers nearby, clustered, high-value balls and re-decides every tick.
2. **Collect via radius capture** — the robot drives to the target's (Kalman-predicted)
   position. Once within `capture_radius_m` (the ball has entered the intake blind-spot
   under the camera), it fires the `SCOOP` servo and tells spatial memory the ball is
   collected, so the same ball is never chased twice.
3. **Deposit when full** — at `storage_capacity` (6) navigate to the home/net pose and
   dump the whole hopper into the net, then return to collecting.
4. **Endgame at 158 s** — return to the home/net pose, dump anything held, then **park &
   hold** on the scoring wall. One trip scores the netted balls *and* the 3-point parking
   bonus (the net hangs on our own wall).
5. **Hard stop at 180 s** — `TIME-UP` holds position for the rest of the §1.13 grace
   window; the robot never drives after the buzzer.

---

## Hardware

| Component | Details |
|-----------|---------|
| Compute | Raspberry Pi 4 (4 GB) |
| Drive | Mecanum wheel holonomic chassis (4× L298N motor drivers) |
| Camera | USB wide-angle, mounted forward-facing |
| Intake | Positional servo clutch (claw gate) |
| Deposit | MG90S continuous servo (trapdoor) |
| Localisation | AprilTag 36h11 fiducials on arena walls |
| Capture sensor | VL53L0X ToF under intake (publishes `/sensors/tof_distance`) — *present but no longer used by the BT; capture is now radius-based* |

---

## Software Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        PERCEPTION LAYER                         │
│                                                                 │
│  camera_node (Python)                                           │
│    V4L2 → /unibots/camera/image_raw (sensor_msgs/Image)         │
│         ↓                                                       │
│  perception_node (C++, NCNN)                                    │
│    YOLO11n inference → /vision/balls (BallArray)                │
│                      → /vision/obstacles (ObstacleArray)        │
└───────────────────────────┬─────────────────────────────────────┘
                            │
┌───────────────────────────▼─────────────────────────────────────┐
│                     MEMORY & LOCALISATION                       │
│                                                                 │
│  spatial_memory_node (C++, Eigen3 Kalman filter)                │
│    Tracks each ball across frames with state [x, y, vx, vy]    │
│    Predicts positions when occluded                             │
│    → /spatial_memory/ball_map (BallMap)                         │
│    → /spatial_memory/prediction_error (Float32)                 │
│                                                                 │
│  unibots_localization (Python, robot_localization EKF)          │
│    AprilTag detections → /odom/filtered (Odometry)              │
└───────────────────────────┬─────────────────────────────────────┘
                            │
┌───────────────────────────▼─────────────────────────────────────┐
│                      DECISION MAKING                            │
│                                                                 │
│  bt_game_node (C++ Behaviour Tree)                              │
│    → /game/target     (PoseStamped) — navigation goal           │
│    → /servo/command   (String)      — SCOOP / OPEN / CLOSE      │
│    → /game/state      (String)      — current BT state          │
│    → /game/ball_collected (UInt32)  — notifies spatial memory   │
└───────────┬───────────────────────────────────┬─────────────────┘
            │                                   │
┌───────────▼────────────┐         ┌────────────▼─────────────────┐
│   NAVIGATION           │         │   HARDWARE INTERFACE          │
│                        │         │                               │
│  apf_controller_node   │         │  hardware_motor_node          │
│  or                    │         │    /cmd_vel → GPIO motors     │
│  mpc_controller_node   │         │                               │
│    /game/target →      │         │  hardware_servo_node          │
│    /cmd_vel            │         │    /servo/command → I2C PWM   │
└────────────────────────┘         └───────────────────────────────┘
```

---

## Behaviour Tree

The game controller (`bt_game_node`) is built on **BehaviorTree.CPP v4**. The tree
lives in `bt/game_tree.xml` (so it is Groot-visualisable) and is ticked at **20 Hz**.
It only moves once `/match/start` is received (rulebook §1.9). The custom leaf nodes are
defined in `include/unibots_bt/bt_nodes.hpp`; all shared state lives in one
`GameContext` placed on the blackboard (`include/unibots_bt/game_context.hpp`).

Key idea: a `ReactiveFallback` re-reads the whole priority checklist top-to-bottom every
tick, so a higher-priority rule (TIME-UP, a full hopper) instantly pre-empts whatever the
robot was doing. Multi-step jobs (drive home → dump → park) use a memory `Sequence` so
completed steps are not re-run.

### Tree Structure

```
ReactiveFallback [Priorities]   ← re-read top→bottom every tick; first applicable wins
│
├── ReactiveSequence [PreStart]      MatchNotStarted → HoldIdle        (status IDLE)
│
├── ReactiveSequence [TimeUp]        MatchTimeExpired → FullStop       (status STOPPED)
│
├── Sequence [Endgame]               (elapsed ≥ endgame_enter_s)
│   ├── InEndgameWindow
│   ├── NavToHome                     drive to the home/net pose
│   ├── Deposit                       OPEN trapdoor → wait → CLOSE; empty hopper
│   └── ParkAndHold                   sit on the scoring wall until the end (PARKED)
│
├── Sequence [DumpWhenFull]          (balls_held ≥ storage_capacity)
│   ├── StorageFull
│   ├── NavToHome
│   └── Deposit                       then falls back to collecting
│
├── ReactiveSequence [Hunt]          (at least one ball known)
│   ├── BallKnown
│   ├── SelectTarget                  highest-utility ball → blackboard
│   └── Fallback [CaptureOrApproach]
│       ├── Sequence [Capture]        WithinCaptureRadius → FireScoop (SCOOP + register)
│       └── Approach                  drive toward the target
│
├── Patrol                           [SEARCH] moving patrol of arena waypoints
│
└── FailsafeStop                     nothing applied — stop and hold
```

### Capture Flow

```
SEARCH  (Patrol drives covering waypoints; camera sweeps as it moves)
  ↓  a ball appears in /spatial_memory/ball_map
HUNT
  SelectTarget → pick highest-utility ball (value × density × visibility ÷ distance)
  Approach     → publish /game/target at the ball's predicted_x/y every tick
  ↓  distance to target < capture_radius_m  (ball in the intake blind-spot)
FireScoop
   → SCOOP servo fired on /servo/command
   → track_id published on /game/ball_collected (spatial_memory marks it COLLECTED)
   → balls_held++   ;   hold scoop_duration_s   ;   next tick picks a fresh target
```

### State Labels (`/game/state`)

Published only when the state changes.

| State | Meaning |
|-------|---------|
| `IDLE` | Pre-start — holding position until `/match/start` |
| `SEARCH` | No target — patrolling the arena |
| `HUNT` | Target selected; driving toward it |
| `CAPTURE` | Within capture radius; SCOOP firing |
| `NAV_HOME` | Driving to the home/net pose |
| `DUMP` | Trapdoor open; emptying hopper into the net |
| `PARKED` | Endgame — parked & holding on the scoring wall |
| `STOPPED` | Time-up hard stop |
| `FAILSAFE` | No branch applied — stopped and holding |

```bash
ros2 topic echo /game/state
```

### Design Rationale

**Radius capture instead of a ToF trigger.**  
The camera cannot see a ball once it is under the robot, so capture fires when the target
enters the intake blind-spot: distance to the (Kalman-predicted) target drops below
`capture_radius_m`. The controller already keeps driving onto a ball it lost sight of, so
no separate ToF/coast machinery is needed.

**Reactive priorities.**  
`ReactiveFallback` re-evaluates every branch each tick. The stopping rules (PRE-START,
TIME-UP) therefore win instantly over any running job, and `Hunt` re-selects its target
every tick — if a closer or higher-value ball appears, or the current one vanishes, the
robot switches without getting stuck.

**Utility target selection.**  
`SelectTarget` scores every ball by `value × (1 + density_bonus·density) × visibility ÷
distance` and picks the best — unlike spatial_memory's `selected_target`, which ignores
point value and distance to the robot. `value` uses the rulebook points (`ping_pong` = 4,
`steel` = 2).

**One endgame trip scores twice.**  
At `endgame_enter_s` the robot drives home, dumps, and `ParkAndHold` keeps it pressed to
the scoring wall until the buzzer — the netted balls score *and* the 3-point parking
bonus is earned in a single trip.

---

## Servo Commands (`/servo/command` String topic)

| Command | Servo | Hardware action | Fired by |
|---------|-------|----------------|----------|
| `SCOOP` | Clutch (positional) | Raise claw arm → pause → lower (grab motion) | `FireScoop` on capture |
| `OPEN`  | Trapdoor (MG90S continuous) | Rotate 180° up → hold → rotate back | `Deposit` (dump into net) |
| `CLOSE` | — | No-op (servo already returned) | `Deposit` after `dump_duration_s` |

---

## Quick Start

```bash
# 1. Enter development environment
nix develop

# 2. Build (run from repo root)
cd unibots_ws
colcon build --packages-select unibots_msgs   # build messages first
colcon build

# 3. Source
source install/setup.bash

# 4. Launch (competition hardware, north zone) — single entry point in unibots_bt
ros2 launch unibots_bt match.launch.py home_zone:=north

# 5. Start the match (subscriber is LATCHED — match the durability or it is dropped)
ros2 topic pub --qos-durability transient_local /match/start \
    std_msgs/msg/Bool '{data: true}' --once
```

**Launch arguments (`match.launch.py`):**

| Argument | Default | Description |
|----------|---------|-------------|
| `home_zone` | `north` | Arena wall assigned by judges (`north/east/south/west`) |
| `controller` | `mpc` | Navigation controller (`mpc` or `apf`) |
| `hardware` | `true` | Launch motor + servo hardware bridges (`false` for sim/dev) |
| `camera` | `true` | Launch the V4L2 `camera_node` (`false` if frames come from sim) |
| `localization` | `true` | Launch AprilTag + EKF (provides `/odom/filtered`) |
| `use_sim_time` | `false` | Use Gazebo clock |

For bench testing the game controller alone (no perception/hardware):
`ros2 launch unibots_bt bt_game.launch.py home_zone:=north`.

---

## Live Tuning (no rebuild required)

```bash
# Perception
ros2 param set /perception_node conf_threshold 0.40
ros2 param set /perception_node input_size 256        # ~30 fps on RPi4
ros2 param set /perception_node input_size 320        # ~22 fps, more accurate
ros2 param set /perception_node hfov_deg 62.5         # calibrate per lens

# Spatial memory prediction model
ros2 param set /spatial_memory_node prediction_mode none
ros2 param set /spatial_memory_node prediction_mode constant_velocity
ros2 param set /spatial_memory_node prediction_mode friction   # default

# Behaviour tree
ros2 param set /bt_game_node use_predicted_position true
ros2 param set /bt_game_node capture_radius_m 0.15    # increase if missing balls
ros2 param set /bt_game_node storage_capacity 6       # balls before a mid-game dump
ros2 param set /bt_game_node weight_steel 1.0         # de-prioritise steel further
ros2 param set /bt_game_node endgame_enter_s 158.0    # earlier = safer park
ros2 param set /bt_game_node home_zone north
```

Full parameter reference with safe ranges: see `TUNING.md`.

---

## Monitoring

```bash
ros2 topic echo /game/state                            # current BT state
ros2 topic echo /spatial_memory/ball_map --no-arr      # tracked ball positions
ros2 topic echo /spatial_memory/prediction_error       # Kalman prediction quality
ros2 topic echo /game/target                           # current navigation goal
ros2 topic echo /game/ball_collected                   # capture events
```

---

## Deployment (Raspberry Pi 5)

Reproducible, infrastructure-as-code deploy lives in `deploy/`. Every C++ node is built
with **max Cortex-A76 optimization** (`-O3 -mcpu=cortex-a76+crypto -mtune=cortex-a76
-flto`); all flags live in one file, `deploy/build.env`. The Pi runs **Ubuntu 26.04
(aarch64) on apt ROS — no Nix on the robot**.

Pick a path (full step-by-step in `deploy/INSTRUCTIONS.md`):

| Path | On the Pi | Compiles on Pi? | Editable over SSH? | Perception (ncnn) |
|------|-----------|-----------------|--------------------|-------------------|
| **Source on Pi** (recommended for edits) | apt ROS + git | yes (native, fast) | **yes** | yes¹ |
| **Prebuilt tarball** (fastest boot) | apt ROS only | no | no | no¹ |
| **Docker** | Docker | no (or in-container) | rebuild in container | yes |
| **Ansible** (fallback) | nothing (laptop provisions) | yes | yes | yes |

```bash
# Source-on-Pi (the editable path):
git clone <repo> ~/unibotsUK && cd ~/unibotsUK
WS=$PWD/unibots_ws bash deploy/build.sh          # native A76 optimized build
source unibots_ws/install/setup.bash
ros2 launch unibots_bt match.launch.py home_zone:=north hardware:=true
# match-day edit: nano src/... ; WS=$PWD/unibots_ws bash deploy/build.sh ; relaunch
```

- **Cross-build the aarch64 tarball in advance** (on an x86 laptop, no Docker, via QEMU +
  bubblewrap): `deploy/cross-build/cross-build.sh` → `deploy/artifacts/unibots-ws-arm64.tar.gz`.
  Method + how to update the package: `deploy/cross-build/README.md`.
- ¹ **`ncnn` is not an apt package on Ubuntu 26.04 arm64.** Build it from source on the Pi
  to enable `unibots_perception`, or run the stack `camera:=false` for a motion/BT/control
  bring-up. The prebuilt tarball excludes perception for this reason.
- Tunables: `deploy/build.env` (optimization), `deploy/packages.apt` (deps),
  `deploy/runtime.env` (match-day `HOME_ZONE`/`CONTROLLER`/...).

---

## Package Summary

| Package | Language | Purpose |
|---------|----------|---------|
| `unibots_camera` | Python | V4L2 camera → raw frames on ROS image topic |
| `unibots_perception` | C++ (NCNN) | YOLO11n real-time ball/obstacle detection |
| `unibots_spatial_memory` | C++ (Eigen3) | Persistent Kalman tracker; ball prediction; density scoring |
| `unibots_bt` | C++ (BehaviorTree.CPP v4) | Behaviour tree game controller + `match.launch.py` (top-level entry point) |
| `unibots_control` | Python | APF/MPC navigation + physical motor/servo hardware nodes |
| `unibots_msgs` | — | Custom ROS 2 message definitions |
| `unibots_localization` | Python | AprilTag → EKF robot pose estimation |
| `unibots_behavior_tree` | Python | Legacy Python BT (superseded by `unibots_bt`; not launched) |

> `unibots_game` (legacy Python FSM + old `match_day.launch.py`) has been **removed** —
> the top-level launch now lives in `unibots_bt/launch/match.launch.py`.

---

## Camera FOV Calibration

`hfov_deg` must match the physical lens or distances will be wrong.

**Quick method** (ping-pong ball + tape measure):
1. Place a 40 mm ball exactly 1000 mm from the camera lens.
2. Capture a frame; measure ball pixel width `px_w` from the debug visualiser.
3. `focal_px = (1000 × px_w) / 40`
4. `hfov_deg = 2 × atan(image_width / (2 × focal_px)) × (180 / π)`

```bash
ros2 param set /perception_node hfov_deg <VALUE>
```

---

## Model Retraining

Training notebook: `training/train_unibots.ipynb` (run on Google Colab, free GPU).

After training:
```bash
cp model_256/model.ncnn.param  unibots_ws/src/unibots_perception/models/model.ncnn.param
cp model_256/model.ncnn.bin    unibots_ws/src/unibots_perception/models/model.ncnn.bin
colcon build --packages-select unibots_perception
```

---

## Known Limitations

- `BallDetection.track_id` is always 0 at the perception level — tracking is performed by `spatial_memory_node`, which assigns its own IDs.
- `ObstacleDetection.world_x/world_y` always 0 — obstacle world position is computed inside the APF/MPC controllers from `bearing_deg` + `distance_m` + robot pose.
- Net deposit/park is open-loop: the BT drives to the `home_zone` pose (`wall_offset_m` stand-off) and dumps. There is no AprilTag lateral/yaw correction at the wall yet — tune `wall_offset_m` / `home_zone` so the mechanism reaches the net reliably.
- `debug_visualiser` saves frames to `/tmp/unibots_debug/` every 10 frames; `cv2.imshow` is disabled for headless operation.
