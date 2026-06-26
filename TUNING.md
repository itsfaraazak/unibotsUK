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

Behaviour tree is BehaviorTree.CPP v4 (`bt/game_tree.xml`). All params below are live-tunable via `ros2 param set /bt_game_node <name> <value>`.

### Match Timing

| Parameter | Default | Safe range | Effect |
|---|---|---|---|
| `tick_rate_hz` | 20.0 | 10–50 | Tree tick rate. Higher = more reactive but more CPU. |
| `match_duration_s` | 180.0 | — | Match length (rulebook). `TIME-UP` holds position once elapsed ≥ this. |
| `endgame_enter_s` | 158.0 | 140–168 | When `ENDGAME` fires: nav home → dump → park & hold. Earlier = safer park, less collecting. 158 leaves ~22 s runway. |

### Target Selection

`utility = value × (1 + density_bonus·density) × visibility ÷ distance`.

| Parameter | Default | Safe range | Effect |
|---|---|---|---|
| `weight_ping` | 4.0 | — | Point value of a ping-pong ball (net). Drives `value`. |
| `weight_steel` | 2.0 | 0.5–2.0 | Point value of a steel bearing. Lower to de-prioritise steel further. |
| `density_bonus` | 0.4 | 0.0–1.0 | Bonus per neighbouring ball — prefers clusters (grab several per trip). 0 = ignore density. |
| `occluded_penalty` | 0.6 | 0.3–1.0 | Utility multiplier for `OCCLUDED` balls. Lower = trust only what is currently visible. |
| `use_predicted_position` | true | — | Target the Kalman `predicted_x/y`. Set false to use `world_x/y` (last observed). |

### Capture

| Parameter | Default | Safe range | Effect |
|---|---|---|---|
| `capture_radius_m` | 0.12 | 0.08–0.20 | Distance to target at which `SCOOP` fires (ball in intake blind-spot). Too small → robot overshoots without scooping. Too large → scoops before the ball is captured. |
| `scoop_duration_s` | 1.4 | 0.8–2.5 | Hold while the claw grabs before selecting the next target. |

### Deposit / Park

| Parameter | Default | Safe range | Effect |
|---|---|---|---|
| `storage_capacity` | 6 | 1–8 | Balls held before a mid-game `DUMP-WHEN-FULL` trip. Larger = fewer interruptions but more points at risk if a deposit fails. |
| `dump_duration_s` | 2.5 | 1.5–4.0 | Seconds the trapdoor stays open. Must exceed the time for all balls to fall through. |
| `wall_offset_m` | 0.15 | 0.10–0.25 | Robot stand-off from the scoring wall at the home/net pose. Smaller = closer (risk of contact), larger = mechanism may not reach the net. Also the `NAV_HOME` arrival tolerance. |

### Arena / Search

| Parameter | Default | Safe range | Effect |
|---|---|---|---|
| `arena_min / arena_max` | 0.15 / 1.85 | — | Clamp published goals to the playable area (matches MPC). |
| `goal_tol_m` | 0.08 | 0.05–0.15 | Distance to a patrol waypoint at which it is considered reached and the cursor advances. |
| `home_zone` | "north" | north/east/south/west | Scoring wall → deposit/park pose. Set to the wall the judges assign. |
| `patrol_waypoints` | (9-pt loop) | — | Flat `[x0,y0, x1,y1, …]` covering loop the `SEARCH` patrol drives until a ball appears. |

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
