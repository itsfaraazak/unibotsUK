# Tuning Reference

All parameters are in YAML files and can be changed without rebuilding.
Set live with `ros2 param set /node_name param_name value` (takes effect immediately).

---

## Perception Node (`unibots_perception/config/perception.yaml`)

| Parameter | Default | Range | Effect |
|---|---|---|---|
| `conf_threshold` | 0.40 | 0.20–0.70 | YOLO detection confidence cutoff. Lower = more detections (more false positives). Higher = fewer detections (may miss real balls). Start at 0.40, drop to 0.30 if balls are missed near edges. |
| `input_size` | 256 | 160–416 | YOLO input resolution (px). 256 → ~30 fps on RPi4. 320 → ~22 fps but more accurate at distance. Match with training size. |
| `num_threads` | 4 | 1–4 | CPU threads for NCNN inference. 4 = all cores; lower if robot runs hot. |
| `hfov_deg` | 62.5 | 40–90 | Horizontal field of view (degrees). **Calibrate per lens** (see CLAUDE.md). Wrong value gives wrong distances and bearings. |

---

## Spatial Memory Node (`unibots_spatial_memory/config/spatial_memory.yaml`)

### Ball Association

| Parameter | Default | Effect |
|---|---|---|
| `assoc_radius_m` | 0.25 | Max distance (m) between a detection and an existing track to be linked. Too small → new tracks created when ball moves fast. Too large → wrong balls get linked. |
| `max_occlusion_frames` | 8 | Frames before a missed track is removed. At 20 Hz: 8 frames = 0.4 s. Too high → ghost balls linger. Too low → tracks lost during brief occlusions. |

### Prediction

| Parameter | Default | Options | Effect |
|---|---|---|---|
| `prediction_mode` | `friction` | `none` \| `constant_velocity` \| `friction` | How ball position is predicted ahead of `pred_horizon_s`. **`none`**: no extrapolation (safe baseline). **`constant_velocity`**: linear; may overshoot for decelerating balls. **`friction`**: exponential velocity decay; most realistic on foam floor. |
| `pred_horizon_s` | 0.5 | 0.1–1.5 | How far ahead (seconds) to predict. Larger = robot drives to future position but prediction error grows. Match to `approach_dist_cm` / typical approach speed. |

### Kalman Filter Noise

Larger Q (process noise) → Kalman trusts measurements more (faster to update, noisier estimate).
Larger R (measurement noise) → Kalman trusts model more (smoother but slower to respond).

| Parameter | Default | Effect |
|---|---|---|
| `kalman_q_pos` | 0.01 | Position process noise (both types). Increase if tracked position jitters. |
| `kalman_q_vel_ping_pong` | 1.0 | Velocity process noise for ping pong. Higher → trusts velocity measurement more (ping pong is lighter, more erratic). |
| `kalman_q_vel_steel` | 0.3 | Velocity process noise for steel balls. Lower → smoother velocity estimate (steel rolls predictably). |
| `kalman_r_pos` | 0.05 | Measurement noise (m²). Increase if YOLO positions are very noisy. Decrease for precise cameras. |

### Motion Gating

| Parameter | Default | Effect |
|---|---|---|
| `motion_gate_m` | 0.03 | Minimum displacement (m) between frames to update Kalman velocity. Below this, velocity component is frozen. Prevents velocity drift from detector jitter when ball is stationary. Reduce if balls appear slow to be tracked when rolling. |

### Friction (for `prediction_mode: friction`)

Measure by rolling a ball and watching how far it rolls at a given speed.

| Parameter | Default | Effect |
|---|---|---|
| `friction_coeff_ping_pong` | 0.15 | Velocity decay per second for ping pong on foam. Higher = ball predicted to stop sooner. |
| `friction_coeff_steel` | 0.05 | Velocity decay per second for steel on foam. Steel is heavier and rolls further. |

### Scoring

| Parameter | Default | Effect |
|---|---|---|
| `density_radius_m` | 0.50 | Radius within which to count neighbouring balls for density scoring. |
| `density_bonus` | 0.4 | Score multiplier per unit of density: `score = confidence × (1 + bonus × density)`. Higher = robot prefers clusters more strongly over isolated balls. |
| `min_confidence` | 0.35 | Tracks below this YOLO confidence are ignored entirely. |

### Measuring Prediction Efficacy

```bash
# Compare modes: switch and watch the error topic
ros2 topic echo /spatial_memory/prediction_error
ros2 param set /spatial_memory_node prediction_mode none
ros2 param set /spatial_memory_node prediction_mode constant_velocity
ros2 param set /spatial_memory_node prediction_mode friction

# Visualise predicted positions in RViz2
ros2 topic echo /spatial_memory/debug_markers  # arrows show predicted movement
```

A lower mean value on `/spatial_memory/prediction_error` = better prediction mode for your floor.

---

## BT Game Node (`unibots_bt/config/bt_game.yaml`)

### Match Timing

| Parameter | Default | Effect |
|---|---|---|
| `t_park_s` | 170 | Seconds after match start to enter PARK mode (navigate to home wall, stop hunting). Rule: 180 s match. Set 10 s before end to give time to park. |
| `t_stop_s` | 185 | Seconds to hard-stop. Safety margin after PARK. |

### Ball Capture Thresholds

| Parameter | Default | Effect |
|---|---|---|
| `approach_dist_cm` | 150 | Distance (cm) at which BT switches from APPROACH (spatial memory goal) to SERVO (live vision goal). Lower = faster response to ball but less benefit from prediction. |
| `servo_blindspot_frac` | 0.88 | Fraction of frame height. When ball pixel_y > this × `frame_height_px`, ball is in the intake blind spot and the BT waits for beam break. Tune by watching `/vision/balls` pixel_y values as a ball enters the intake. |
| `frame_height_px` | 720 | Must match camera resolution height set in `perception_node input_size`. |
| `capacity` | 6 | Balls to collect before depositing. Rulebook §4.3: 16 ping pong + 24 steel = 40 total; robot may carry as many as physically fits. |

### Deposit Sequence

| Parameter | Default | Effect |
|---|---|---|
| `near_wall_dist_m` | 0.30 | Distance from home goal at which NAV_HOME is considered complete. Larger = BT moves to ALIGN_NET earlier (less precise approach). |
| `align_timeout_s` | 10 | Max seconds spent in ALIGN_NET before DUMP fires anyway. Increase if precise alignment is needed. Decrease on match day to ensure deposit happens. |
| `align_standoff_m` | 0.15 | How far from the wall the robot tries to be during ALIGN_NET (metres). Prevents crashing into the wall. |
| `dump_duration_s` | 2.5 | Seconds the servo stays open during deposit. Must be long enough for all balls to fall out. |

### Search Behaviour

| Parameter | Default | Effect |
|---|---|---|
| `search_yaw_step_deg` | 30 | Degrees per BT tick added to search yaw. At 20 Hz: 30°/tick × 20 = 600°/s rotation (very fast). Reduce to 5–10 for a slower sweep that gives YOLO time to detect. |

### Prediction Passthrough

| Parameter | Default | Effect |
|---|---|---|
| `use_predicted_position` | true | If true, APPROACH drives to `predicted_x/y` from spatial_memory. Set false to drive to `world_x/y` (current position). Use false to isolate prediction accuracy issues. |

---

## APF Controller (`unibots_control/apf_controller_node.py`)

Parameters declared in node (not yet in YAML — tune live with `ros2 param set`):

| Parameter | Default | Effect |
|---|---|---|
| `k_att` | 1.0 | Attractive gain toward goal. Higher = faster approach but may overshoot. |
| `k_rep` | 0.3 | Repulsive gain from obstacles. Higher = wider avoidance berth. 0 = ignores obstacles. |
| `influence_radius` | 0.5 | Distance (m) at which obstacle repulsion activates. |
| `max_vx / max_vy` | 0.35 / 0.45 | Body-frame velocity limits (m/s). Reduce if robot slides past balls. |
| `max_omega` | 1.2 | Yaw rate limit (rad/s). |
| `goal_tolerance` | 0.05 | Distance (m) from goal to consider it reached. |

---

## MPC Controller (`unibots_control/mpc_controller_node.py`)

| Parameter | Default | Effect |
|---|---|---|
| `dt` | 0.05 | MPC timestep (s). Smaller = more accurate prediction but more QP solve time. |
| `horizon_n` | 10 | Prediction horizon (timesteps). Longer = smoother paths but more compute. |
| `obstacle_safety_radius` | 0.25 | Safety padding (m) added to obstacle radius for half-plane constraints. |
| `yaw_kp` | 1.5 | Proportional gain on yaw error for the decoupled heading controller. |

---

## Known Gaps / Future Work

- **`BallDetection.track_id` always 0** — the Kalman tracker in `spatial_memory_node` assigns its own IDs; perception's `track_id` field is unused.
- **`ObstacleDetection.world_x/world_y` always 0** — EKF + homography not wired. Obstacle world positions are projected from `bearing_deg` + `distance_m` + robot pose in APF/MPC callbacks.
- **ALIGN_NET is timeout-only** — fine AprilTag lateral/yaw correction not yet implemented in the C++ BT. The timeout fallback fires after `align_timeout_s` and proceeds to DUMP.
- **Search is rotate-only** — no arena survey waypoints. If rotation doesn't expose balls within one full turn, no fallback plan exists. Add survey waypoints to the SEARCH action if needed.
