#!/usr/bin/env python3
"""Ultrasonic proximity node — collision avoidance for the Unibots robot.

Drives a config-defined set of ultrasonic sensors (front + sides today,
extensible to back or any number of sensors by editing ``config/ultrasonic.yaml``)
and fans out three kinds of output every cycle:

  /ultrasonic/<name>          sensor_msgs/Range   per-sensor raw range (rviz, debug)
  /ultrasonic/obstacles       unibots_msgs/ObstacleArray
                                  near sensors projected to bearing/distance so the
                                  APF / MPC controllers repel from them exactly as
                                  they already do for camera obstacles ("in
                                  conjunction with the camera").
  /safety/collision_warning   std_msgs/Bool       latched hard-stop guard: True when
                                  any sensor is closer than collision_warn_distance_m.

Supports rulebook 3.2: non-contact sport, collision avoidance is required, robots
that make contact may be disqualified. The /safety/collision_warning topic is the
last-resort guard; the obstacle fusion is the primary, smooth avoidance path.

Each sensor's pins and mounting geometry come from parameters, so multiple sensors
on arbitrary GPIO pins are supported with no code change. See config/ultrasonic.yaml.
"""
from __future__ import annotations

import math
from typing import Dict, List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile

from std_msgs.msg import Bool, Header
from sensor_msgs.msg import Range
from unibots_msgs.msg import ObstacleArray, ObstacleDetection

from unibots_ultrasonic.sensor_driver import (
    SensorConfig,
    UltrasonicDriver,
    make_driver,
)

# Per-sensor sub-parameters and their defaults (mirrors SensorConfig fields).
_SENSOR_FIELDS = {
    "trigger_pin": None,        # required
    "echo_pin": None,           # required
    "mount_x": 0.0,
    "mount_y": 0.0,
    "mount_yaw_deg": 0.0,
    "min_range_m": 0.02,
    "max_range_m": 2.0,
    "fov_deg": 15.0,
}


class UltrasonicNode(Node):
    def __init__(self):
        # Nested YAML (e.g. front.trigger_pin) arrives as dot-separated params;
        # declare them automatically from the loaded overrides.
        super().__init__(
            "ultrasonic_node",
            automatically_declare_parameters_from_overrides=True,
        )

        self._declare_with_default("publish_rate_hz", 20.0)
        self._declare_with_default("collision_warn_distance_m", 0.12)
        self._declare_with_default("obstacle_report_distance_m", 0.50)
        self._declare_with_default("obstacle_radius_m", 0.05)
        self._declare_with_default("frame_id", "base_link")
        self._declare_with_default("use_mock", False)
        self._declare_with_default("sensors", [])

        self._frame_id = self.get_parameter("frame_id").value
        self._warn_dist = float(self.get_parameter("collision_warn_distance_m").value)
        self._report_dist = float(self.get_parameter("obstacle_report_distance_m").value)
        self._obstacle_radius = float(self.get_parameter("obstacle_radius_m").value)
        use_mock = bool(self.get_parameter("use_mock").value)

        names = list(self.get_parameter("sensors").value or [])
        if not names:
            self.get_logger().warn(
                "No sensors configured (param 'sensors' is empty) — node idle.")

        # Build a driver + Range publisher per sensor.
        self._drivers: List[UltrasonicDriver] = []
        self._range_pubs: Dict[str, rclpy.publisher.Publisher] = {}
        for name in names:
            cfg = self._load_sensor_config(name)
            if cfg is None:
                continue
            driver = self._make_driver_safe(cfg, use_mock)
            if driver is None:
                continue
            self._drivers.append(driver)
            self._range_pubs[name] = self.create_publisher(
                Range, f"/ultrasonic/{name}", 10)
            self.get_logger().info(
                f"Sensor '{name}': trig={cfg.trigger_pin} echo={cfg.echo_pin} "
                f"yaw={cfg.mount_yaw_deg:.0f}deg range<={cfg.max_range_m:.2f}m "
                f"backend={type(driver).__name__}")

        # Fused obstacle output (consumed by APF / MPC) + safety guard.
        self._obstacle_pub = self.create_publisher(
            ObstacleArray, "/ultrasonic/obstacles", 10)
        # Latched so a late-joining safety subscriber sees the current state.
        latched = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._warn_pub = self.create_publisher(Bool, "/safety/collision_warning", latched)
        self._warn_state = False
        self._publish_warning(False)  # publish an initial "clear"

        rate = float(self.get_parameter("publish_rate_hz").value)
        rate = rate if rate > 0.0 else 20.0
        self._timer = self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f"Ultrasonic node up: {len(self._drivers)} sensor(s) @ {rate:.0f} Hz, "
            f"warn<{self._warn_dist:.2f}m, report<{self._report_dist:.2f}m, "
            f"mock={use_mock}")

    # ---------------------------------------------------------------- params
    def _declare_with_default(self, name: str, default):
        """Declare a param unless override-autodeclare already created it."""
        if not self.has_parameter(name):
            self.declare_parameter(name, default)

    def _load_sensor_config(self, name: str):
        """Assemble a SensorConfig for ``name`` from its dot-namespaced params."""
        values = {}
        for field, default in _SENSOR_FIELDS.items():
            param = f"{name}.{field}"
            if self.has_parameter(param):
                values[field] = self.get_parameter(param).value
            elif default is None:
                self.get_logger().error(
                    f"Sensor '{name}' missing required param '{param}' — skipped.")
                return None
            else:
                values[field] = default
        try:
            return SensorConfig(
                name=name,
                trigger_pin=int(values["trigger_pin"]),
                echo_pin=int(values["echo_pin"]),
                mount_x=float(values["mount_x"]),
                mount_y=float(values["mount_y"]),
                mount_yaw_deg=float(values["mount_yaw_deg"]),
                min_range_m=float(values["min_range_m"]),
                max_range_m=float(values["max_range_m"]),
                fov_deg=float(values["fov_deg"]),
            )
        except (TypeError, ValueError) as exc:
            self.get_logger().error(f"Sensor '{name}' bad config: {exc} — skipped.")
            return None

    def _make_driver_safe(self, cfg: SensorConfig, use_mock: bool):
        """Construct a driver; one bad/absent sensor must not kill the node."""
        try:
            return make_driver(cfg, use_mock)
        except Exception as exc:  # noqa: BLE001 - report and continue
            self.get_logger().error(
                f"Sensor '{cfg.name}' driver init failed ({exc}); "
                "sensor disabled. Check wiring / pins / gpiozero install.")
            return None

    # ----------------------------------------------------------------- loop
    def _tick(self):
        now = self.get_clock().now().to_msg()
        obstacles: List[ObstacleDetection] = []
        min_dist = math.inf

        for driver in self._drivers:
            cfg = driver.config
            try:
                dist = float(driver.read())
            except Exception as exc:  # noqa: BLE001 - skip this reading
                self.get_logger().warn(f"Sensor '{cfg.name}' read failed: {exc}")
                continue

            self._publish_range(cfg, dist, now)
            min_dist = min(min_dist, dist)

            # Near reading -> emit an obstacle for the avoidance pipeline.
            if dist < self._report_dist:
                obstacles.append(self._to_obstacle(cfg, dist))

        self._publish_obstacles(obstacles, now)

        warn = min_dist < self._warn_dist
        if warn != self._warn_state:
            self._publish_warning(warn)

    # ------------------------------------------------------------- publish
    def _publish_range(self, cfg: SensorConfig, dist: float, stamp):
        msg = Range()
        msg.header = Header(stamp=stamp, frame_id=f"ultrasonic_{cfg.name}")
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = math.radians(cfg.fov_deg)
        msg.min_range = cfg.min_range_m
        msg.max_range = cfg.max_range_m
        msg.range = dist
        self._range_pubs[cfg.name].publish(msg)

    def _to_obstacle(self, cfg: SensorConfig, dist: float) -> ObstacleDetection:
        """Project a range along the sensor's mounted bearing into base_link.

        bearing_deg follows the camera/APF convention (CCW+ from robot forward),
        so the existing controllers fuse ultrasonic and camera obstacles with the
        same projection math. world_x/world_y are filled in base_link (the
        controllers recompute arena-frame from bearing/distance + robot pose).
        """
        yaw = math.radians(cfg.mount_yaw_deg)
        obs = ObstacleDetection()
        obs.bearing_deg = float(cfg.mount_yaw_deg)
        obs.distance_m = float(dist)
        obs.radius_m = float(self._obstacle_radius)
        obs.world_x = float(cfg.mount_x + dist * math.cos(yaw))
        obs.world_y = float(cfg.mount_y + dist * math.sin(yaw))
        obs.is_confirmed_robot = False
        obs.pixel_x = 0.0
        return obs

    def _publish_obstacles(self, obstacles: List[ObstacleDetection], stamp):
        msg = ObstacleArray()
        msg.header = Header(stamp=stamp, frame_id=self._frame_id)
        msg.obstacles = obstacles
        self._obstacle_pub.publish(msg)

    def _publish_warning(self, warn: bool):
        self._warn_state = warn
        self._warn_pub.publish(Bool(data=warn))
        if warn:
            self.get_logger().warn("COLLISION WARNING — obstacle within hard-stop range")
        else:
            self.get_logger().info("Collision warning cleared")

    # ----------------------------------------------------------------- exit
    def destroy_node(self):
        for driver in self._drivers:
            driver.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
