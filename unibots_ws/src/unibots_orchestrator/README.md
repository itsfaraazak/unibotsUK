# unibots_orchestrator

High-level **C++** match strategy state machine for Unibots 2026 — the brain that
turns the existing perception/kalman (`spatial_memory`), navigation (MPC) and
actuation (servo) stacks into a complete autonomous match. Pure `rclcpp` for
minimum-latency reaction (no Python GIL, no py_trees tick): one node, one 20 Hz
timer, plain function dispatch.

It owns **decisions and goals only** — the MPC computes motion. The node drives
`/cmd_vel` directly for exactly two manoeuvres the MPC cannot express: in-place
360° spins and the timed wall-flush.

```
SLEEP --button--> STARTUP --> SEARCH <-> CHASE
                                |          |
                                +--> DEPOSIT --(t>=150)--> SLEEP
                                            +--(t<150)---> SEARCH
ToF trigger (any running state) --> STOP + SCOOP --> resume prior state
```

## Build & run

```bash
cd unibots_ws
colcon build --packages-up-to unibots_orchestrator   # builds unibots_msgs too
source install/setup.bash
ros2 launch unibots_orchestrator match_orchestrator.launch.py home:=south

# start (toggle): press = start, press again = sleep
ros2 topic pub --once /match/button std_msgs/msg/Bool '{data: true}'
```

Expects the rest of the stack running (camera → perception → `spatial_memory`,
the MPC controller, hardware motor/servo nodes).

## I/O

| Dir | Topic | Type | Note |
|-----|-------|------|------|
| sub | `/odom/filtered` | `nav_msgs/Odometry` | robot pose |
| sub | `/spatial_memory/ball_map` | `unibots_msgs/BallMap` | kalman world balls (absolute) |
| sub | `/sensors/tof_distance` | `sensor_msgs/Range` | collection-zone ToF |
| sub | `/match/button` | `std_msgs/Bool` (data=true=press) | start/sleep toggle |
| sub | `/match/start` | `std_msgs/Bool` (latched) | explicit start(true)/sleep(false) |
| pub | `/game/target` | `geometry_msgs/PoseStamped` | MPC goal (streamed only while moving) |
| pub | `/cmd_vel` | `geometry_msgs/TwistStamped` | spin / flush / hard-stop only |
| pub | `/servo/command` | `std_msgs/String` | `SCOOP` \| `OPEN` \| `CLOSE` |
| pub | `/game/ball_collected` | `std_msgs/UInt32` | collected track_id → spatial_memory marks COLLECTED |
| pub | `/mpc/enable` | `std_msgs/Bool` (latched) | hand `/cmd_vel` authority to/from MPC |
| pub | `/game/state` | `std_msgs/String` (latched) | telemetry |

## Spec → code map

| Requirement | Where |
|-------------|-------|
| 1. Arena 0–2, home variable + valid tiles, AprilTag dict | params `home`/`arena_size_m`, `resolve_home`, `apriltag_ids`/`apriltag_xy` |
| 1. Start/Stop toggle | `/match/button` (+ latched `/match/start`) |
| 2. Continuous goal broadcast; stop on arrival | `broadcast_goal` (every tick) / `stop_broadcast` |
| 2. 2 cm stability buffer | `broadcast_goal` holds prev goal when candidate < 2 cm |
| 2. Startup: forward 0.25·arena, spin 360, → SEARCH | `tick_startup` |
| 3. Search waypoint → arrive → spin 360 → next | `tick_search` |
| 4. relative→absolute + dynamic map + kalman | upstream `spatial_memory_node` (consumed here) |
| 5. Priority value×density×visibility÷distance | `select_target` |
| 5. Lock-on safeguard | **APPLIED** (see below) |
| 6. Loss: coast to last goal then revert SEARCH w/ mandatory spin | `tick_chase` + `begin_spin`; MPC's own 1 s coast mirrors it |
| 7. ToF global interrupt: stop → scoop → resume | `tof_triggered`/`begin_collect`/`tick_collect` |
| 7. Capacity 6 → deposit | `tick_collect` |
| 8. Deposit: home → flush 2 s → dump → close → eval | `tick_deposit` |
| 8. 150 s override (finish busy servo) → deposit; 180 s → sleep | endgame block in `tick` |

## Safeguards / design notes (APPLIED)

- **Lock-on chase (Section 5 override APPLIED).** Re-running the priority
  algorithm every tick risks rapid target switching, so the node locks the
  initial selection (`lock_id_`) and does **not** re-select until the chase
  resolves (collected, or lost past the coast window).

- **`/mpc/enable` authority gate (new, also patched into the MPC node).** The MPC
  cannot command pure rotation or an open-loop push, so for spins / wall-flush
  the node publishes `false` (latched); the MPC emits one zero then goes quiet,
  yielding `/cmd_vel`. `true` hands authority back. Defaults `true`, so the C++
  BT / standalone MPC are unaffected.

- **ToF hardened against claw false-triggers.** The claw can cross the beam, so a
  capture requires `tof_consecutive` (default 3) samples below `tof_trigger_m`,
  with hysteresis re-arm above `tof_rearm_m`; ToF is ignored entirely while
  scooping and for `tof_inhibit_after_scoop_s` afterwards (claw retract), and is
  never sampled during DEPOSIT.

- **Continuous goal stream.** `/game/target` is republished every tick while
  moving (the 2 cm buffer only filters jitter, it never throttles the stream),
  so a late-subscribing or restarted MPC always has a live goal.

## Verified

`colcon build` clean (zero `-Wall -Wextra -Wpedantic` warnings); node launches,
parses YAML, streams the correct startup goal `(1.0, 0.5)` from `home=south`;
ball→LOCK→CHASE; single ToF spike rejected, sustained ToF → SCOOP → `count=1/6`;
spin-preempt produces a single clean lock.
