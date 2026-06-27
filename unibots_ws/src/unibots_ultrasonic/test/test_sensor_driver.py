"""Unit tests for the ultrasonic driver layer (no ROS graph required)."""
import math

from unibots_ultrasonic.sensor_driver import (
    MockUltrasonicDriver,
    SensorConfig,
    make_driver,
)


def _cfg(**kw):
    base = dict(name="front", trigger_pin=23, echo_pin=24,
                min_range_m=0.02, max_range_m=2.0)
    base.update(kw)
    return SensorConfig(**base)


def test_mock_defaults_to_max_range():
    d = MockUltrasonicDriver(_cfg())
    assert d.read() == 2.0


def test_mock_inject_clamps_to_bounds():
    d = MockUltrasonicDriver(_cfg())
    d.inject(0.30)
    assert math.isclose(d.read(), 0.30)
    d.inject(99.0)      # above max
    assert d.read() == 2.0
    d.inject(-5.0)      # below min
    assert d.read() == 0.02


def test_make_driver_mock_selects_mock_backend():
    d = make_driver(_cfg(), use_mock=True)
    assert isinstance(d, MockUltrasonicDriver)


def test_config_carries_geometry():
    c = _cfg(mount_x=0.1, mount_yaw_deg=90.0)
    assert c.mount_x == 0.1
    assert c.mount_yaw_deg == 90.0
