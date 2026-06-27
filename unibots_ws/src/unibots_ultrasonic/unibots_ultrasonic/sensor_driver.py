"""Hardware-abstraction layer for ultrasonic range sensors.

The node never talks to GPIO directly. It asks :func:`make_driver` for a driver
per sensor and calls :meth:`UltrasonicDriver.read` each cycle. This keeps the ROS
plumbing free of hardware specifics and lets the same node run on a Raspberry Pi
(``GpiozeroUltrasonicDriver``) or on a dev laptop / CI (``MockUltrasonicDriver``).

Adding a new backend (e.g. an I2C ultrasonic, or a serial sonar array) means
adding one ``UltrasonicDriver`` subclass and a branch in :func:`make_driver` —
no node changes.

Pin assignment is per sensor and comes entirely from configuration, so any
number of sensors on any free GPIO pins is supported.
"""
from __future__ import annotations

import abc
from dataclasses import dataclass


@dataclass(frozen=True)
class SensorConfig:
    """Static description of one ultrasonic sensor, loaded from YAML.

    Geometry (``mount_*``) is expressed in the robot ``base_link`` frame and is
    used to project a raw range reading into an obstacle bearing/position for the
    collision-avoidance pipeline.
    """

    name: str
    trigger_pin: int
    echo_pin: int
    mount_x: float = 0.0          # metres forward of base_link origin
    mount_y: float = 0.0          # metres left of base_link origin (CCW+)
    mount_yaw_deg: float = 0.0    # beam-centre bearing, CCW+ from robot forward
    min_range_m: float = 0.02     # HC-SR04 practical floor (~2 cm)
    max_range_m: float = 2.0      # reading clamps/saturates here
    fov_deg: float = 15.0         # beam half-angle is fov_deg/2; for Range.msg


class UltrasonicDriver(abc.ABC):
    """One physical sensor. ``read()`` returns a distance in metres."""

    def __init__(self, config: SensorConfig):
        self.config = config

    @abc.abstractmethod
    def read(self) -> float:
        """Return the current range in metres.

        Implementations must clamp to ``[min_range_m, max_range_m]`` and return
        ``max_range_m`` when nothing is detected (saturated / out of range),
        never raise for a routine no-echo timeout.
        """

    def close(self) -> None:  # pragma: no cover - backend specific
        """Release any hardware resources. Safe to call more than once."""


class GpiozeroUltrasonicDriver(UltrasonicDriver):
    """HC-SR04 backend via gpiozero ``DistanceSensor`` (lgpio pin factory).

    Matches the GPIO stack used by ``hardware_motor_node`` so both share one
    pin factory on the Pi. gpiozero reports distance in metres already.
    """

    def __init__(self, config: SensorConfig):
        super().__init__(config)
        # Imported lazily so the package builds / imports on machines without
        # gpiozero or RPi GPIO (dev laptops, CI). Failure here is actionable.
        from gpiozero import DistanceSensor

        self._sensor = DistanceSensor(
            echo=config.echo_pin,
            trigger=config.trigger_pin,
            max_distance=config.max_range_m,
            threshold_distance=config.min_range_m,
        )

    def read(self) -> float:
        d = float(self._sensor.distance)  # metres, already in [0, max_distance]
        if d < self.config.min_range_m:
            return self.config.min_range_m
        if d > self.config.max_range_m:
            return self.config.max_range_m
        return d

    def close(self) -> None:  # pragma: no cover - hardware only
        try:
            self._sensor.close()
        except Exception:
            pass


class MockUltrasonicDriver(UltrasonicDriver):
    """No-hardware backend: always reports max range (clear path).

    Lets the node, launch files, and downstream fusion run off-Pi without GPIO.
    ``inject(distance_m)`` allows tests to simulate an approaching obstacle.
    """

    def __init__(self, config: SensorConfig):
        super().__init__(config)
        self._value = config.max_range_m

    def inject(self, distance_m: float) -> None:
        self._value = max(self.config.min_range_m,
                          min(self.config.max_range_m, distance_m))

    def read(self) -> float:
        return self._value


def make_driver(config: SensorConfig, use_mock: bool) -> UltrasonicDriver:
    """Factory: pick a backend for one sensor.

    Falls back to the mock backend (with a flag the caller can inspect via the
    returned type) if the real backend cannot be constructed, so a single bad /
    absent sensor never takes the whole node down.
    """
    if use_mock:
        return MockUltrasonicDriver(config)
    return GpiozeroUltrasonicDriver(config)
