# Localization / Odometry deadlock — KNOWN ISSUE (P1)

**Status:** worked around (open-loop SEARCH bootstrap). Root cause unresolved — needs
hardware (encoders or IMU). Documented 2026-06-28.

## Symptom
On the real robot, after `match.launch.py` + start, **the motors never spin and
nothing moves**. In simulation everything works.

## Root cause
The entire motion stack gates on a robot pose from `/odom/filtered`.

```
/odom (wheel)  ─┐
/imu/data      ─┼─►  ekf_filter_node (robot_localization)  ──►  /odom/filtered
AprilTag pose  ─┘        (config: unibots_localization/config/ekf.yaml)
```

`ekf.yaml` declares two inputs:
- `odom0: /odom`   — wheel odometry (velocity x, y, yaw-rate)
- `pose0: /localization/robot_pose` — absolute pose from AprilTags (via `ekf_bridge_node`)

**On the real robot only `pose0` ever produces data.** There is:
- **No `/odom` publisher** — `/odom` is produced only by Gazebo in `unibots_sim`
  (`mecanum_drive_controller/odometry` → `/odom`). `match.launch.py` does **not**
  launch `ros2_control` / `controller_manager` / `robot_state_publisher`.
- **No IMU** — nothing publishes `/imu/data` outside sim.
- **No wheel encoders in software** — the real motor driver is `hardware_motor_node`
  → gpiozero `Motor(forward, backward, enable)` (3 L298N pins per motor, PWM only,
  **no feedback channel**).

So `/odom/filtered` is produced **only when an AprilTag is in view**, and not at all
before the first sighting. The robot must move to see a tag, but it can't move
without a pose → **deadlock** → motors stay at zero.

## Current workaround (in repo)
Open-loop SEARCH bootstrap in **both** controllers (`mpc_controller_node.py`,
`apf_controller_node.py`): when `/game/state == "SEARCH"` and there is no pose (or
no compiled solver), drive a blind advance→rotate scan directly on `/cmd_vel`
(params `scan_fwd`, `scan_yaw`, `scan_advance_s`, `scan_rotate_s`). The robot moves,
sweeps the camera, catches an AprilTag, localization seeds, and closed-loop control
takes over. Requires the BT to heartbeat `/game/state` every tick
(`game_context.hpp::publishState`).

This **guarantees motion** but NOT accurate navigation — quality depends entirely
on how often AprilTags are seen.

## Proper fix (needs hardware / pin info)
Pick one (encoders preferred for a holonomic base):

1. **Wheel encoders → `/odom`.** Requires:
   - Encoder-equipped motors (check for quadrature A/B signal wires — these go
     **directly to Pi GPIO**, NOT through the L298N). 4 motors ≈ up to 8 GPIO.
   - A new `encoder_odom_node`: read A/B counts → mecanum forward kinematics →
     `nav_msgs/Odometry` on `/odom`. Needs per-motor encoder GPIO pins,
     counts-per-rev, wheel radius, and wheelbase/track to be supplied.
2. **IMU → `/imu/data`** (e.g. MPU6050/BNO055 over I2C) and add `imu0` to `ekf.yaml`.
   Gives yaw-rate + heading but no absolute position; drifts without `/odom`.

Until then, keep the open-loop SEARCH bootstrap and ensure AprilTags (rulebook §4.5,
IDs per wall: N 0–5, E 6–11, S 12–17, W 18–23) are well-lit and within camera FOV.
