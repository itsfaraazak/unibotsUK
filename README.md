# UnibotsUK 2025–2026

Autonomous competition robot — 2000×2000 mm arena, 180 s match, 16 ping-pong balls (4 pts/net) + 24 steel bearings (1 pt held / 2 pts/net).

## Quick Start

```bash
nix develop                          # enter Nix shell (colcon, cmake, ncnn, opencv)
cd unibots_ws
colcon build --packages-select unibots_msgs
colcon build
source install/setup.bash

# Competition launch (real hardware, north zone)
ros2 launch unibots_game match_day.launch.py home_zone:=north

# Start match
ros2 topic pub /match/start std_msgs/msg/Bool '{data: true}' --once
```

Pass `hardware:=false` to skip GPIO/I2C nodes during development.

---

## Node Pipeline

```
camera_node (Python)
  └─/unibots/camera/image_raw──► perception_node (C++/NCNN YOLO11n)
                                    └─/vision/balls
                                    └─/vision/obstacles
                                         │
                                         ├──► spatial_memory_node (C++ Kalman)
                                         │      └─/spatial_memory/ball_map
                                         │
                                         └──► bt_game_node (C++ BT)
                                                └─/game/target ──► apf/mpc_controller_node
                                                └─/servo/command ──► hardware_servo_node
                                                └─/cmd_vel ──► hardware_motor_node
```

---

## Behaviour Tree (`unibots_bt`)

Ticks at 20 Hz. Match starts only after `/match/start` is published.

```
Sequence [ROOT — gated on match_started]
└── Fallback [GAME]
    ├── Sequence [STOP]      elapsed ≥ t_stop_s (185 s)   → hold position
    ├── Sequence [PARK]      elapsed ≥ t_park_s (170 s)   → drive to home wall (+3 pts)
    ├── Sequence [DEPOSIT]   balls_held ≥ capacity (6)
    │     ├── NAV_HOME       drive to home wall
    │     ├── ALIGN_NET      approach standoff; timeout-based alignment
    │     └── DUMP           servo OPEN → wait → servo CLOSE; reset counter
    └── Fallback [HUNT/SEARCH]
          ├── Sequence [HUNT]   spatial_memory has a valid tracked target
          │     └── Fallback [APPROACH_STRATEGY]
          │           ├── Sequence [CAPTURE]  ball pixel_y > blind-spot threshold
          │           │     └── send SCOOP → wait for beam_broken → register +1 ball
          │           ├── Sequence [SERVO]    ball distance < approach_dist_cm
          │           │     └── drive to ball world position
          │           └── APPROACH             drive to Kalman-predicted ball position
          └── SEARCH          rotate in place to scan arena
```

### Servo Commands (`/servo/command` String topic)

| Command | Hardware action |
|---------|----------------|
| `SCOOP` | Claw (clutch positional servo): grab motion (up 1.3 s → down) |
| `OPEN`  | Deposit trapdoor (MG90S continuous): turn 180° up, hold 2 s, return |
| `CLOSE` | No-op (sent by BT after dump; servo already returned during OPEN) |

---

## Scoring Strategy

| Action | Points | How |
|--------|--------|-----|
| Park at end | 3 | BT enters PARK at t=170 s |
| Bearing in net | 2 | DEPOSIT before PARK |
| Ping-pong in net | 4 | DEPOSIT before PARK |
| Bearing held at end | 1 | Fallback if no time to deposit |

Priority order in BT: STOP → PARK → DEPOSIT → HUNT → SEARCH.  
PARK fires at 170 s leaving 10 s to reach home wall before the 180 s buzzer.

---

## Live Tuning (no rebuild)

```bash
ros2 param set /perception_node conf_threshold 0.40
ros2 param set /perception_node input_size 256          # ~30 fps on RPi4
ros2 param set /spatial_memory_node prediction_mode friction
ros2 param set /bt_game_node use_predicted_position true
ros2 param set /bt_game_node home_zone north
```

See `TUNING.md` for full parameter reference with safe ranges.

---

## Package Summary

| Package | Language | Role |
|---------|----------|------|
| `unibots_camera` | Python | V4L2 camera → raw frames |
| `unibots_perception` | C++ (NCNN) | YOLO11n ball/obstacle detection |
| `unibots_spatial_memory` | C++ (Eigen3) | Kalman ball tracker + prediction |
| `unibots_bt` | C++ | Behaviour tree game controller |
| `unibots_control` | Python | APF/MPC navigation + hardware motor/servo nodes |
| `unibots_msgs` | – | Custom message definitions |
| `unibots_localization` | Python | AprilTag EKF pose estimation |
| `unibots_game` | Python | Launch files |
| `unibots_behavior_tree` | Python | Legacy Python BT (not launched; replaced by `unibots_bt`) |
