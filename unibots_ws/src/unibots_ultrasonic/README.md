# unibots_ultrasonic

Ultrasonic (HC-SR04 style) proximity sensing and collision avoidance for the
Unibots UK 2026 holonomic mecanum robot. Supports **rulebook §3.2** — *"This is a
non-contact sport and robots should implement collision avoidance capability"* —
by guarding against contact with walls and other robots, working alongside the
camera-based perception.

## What it does

A single node (`ultrasonic_node`) drives a **config-defined set** of sensors and
publishes three outputs each cycle:

| Topic | Type | Purpose |
|-------|------|---------|
| `/ultrasonic/<name>` | `sensor_msgs/Range` | Per-sensor raw range (rviz / debug). One per sensor. |
| `/ultrasonic/obstacles` | `unibots_msgs/ObstacleArray` | Near readings projected to bearing/distance, fused by the **APF / MPC** controllers exactly like camera obstacles → smooth avoidance (primary path). |
| `/safety/collision_warning` | `std_msgs/Bool` (latched) | `true` when any sensor is within `collision_warn_distance_m` → last-resort hard-stop guard. |

The ultrasonic obstacles use the **same bearing convention** (`bearing_deg`,
CCW-positive from robot forward) that perception publishes, so the existing
`apf_controller_node` / `mpc_controller_node` repulsion logic consumes them with
no change — that is the "in conjunction with the camera" fusion.

## Extensibility

Everything is driven by `config/ultrasonic.yaml`:

- **Add a sensor** → add a named block (`trigger_pin`, `echo_pin`, mount geometry)
  and put its name in the `sensors:` list. The rear sensor (`back`) block is
  already present — just add `back` to `sensors:` once it's wired.
- **Different pins** → every sensor's `trigger_pin` / `echo_pin` is independent
  (BCM GPIO numbers, gpiozero / lgpio — same stack as `hardware_motor_node`).
- **New hardware backend** (I2C sonar, serial array, sim) → add one
  `UltrasonicDriver` subclass in `sensor_driver.py` and a branch in
  `make_driver()`. No node changes.

## Run

```bash
# On the robot
ros2 launch unibots_ultrasonic ultrasonic.launch.py

# Off-Pi dev / CI (no GPIO) — mock backend reports "clear" for every sensor
ros2 launch unibots_ultrasonic ultrasonic.launch.py use_mock:=true

# Custom config
ros2 launch unibots_ultrasonic ultrasonic.launch.py config:=/abs/path/to.yaml
```

Monitor:

```bash
ros2 topic echo /ultrasonic/front
ros2 topic echo /ultrasonic/obstacles
ros2 topic echo /safety/collision_warning
```

## Live tuning

```bash
ros2 param set /ultrasonic_node collision_warn_distance_m 0.10
ros2 param set /ultrasonic_node obstacle_report_distance_m 0.60
```

(Per-sensor geometry/pins are read at startup; change those in the YAML and
restart the node.)

## Wiring notes

- HC-SR04 echo is 5 V — use a level shifter / divider to the Pi's 3.3 V GPIO.
- Pins in `ultrasonic.yaml` are defaults; set them to match your harness.
- Mount geometry (`mount_x`, `mount_y`, `mount_yaw_deg`) is in `base_link`; get it
  right or fused obstacle positions will be wrong.

## Build & test

```bash
cd unibots_ws
colcon build --packages-select unibots_msgs
colcon build --packages-select unibots_ultrasonic
colcon test --packages-select unibots_ultrasonic
```
