#!/usr/bin/env python3
"""Physical START button -> /match/start for the Unibots 2026 robot.

Rulebook 1.9: the robot must be started by a team member pressing a physical
button. This node watches a GPIO button and, on press, latches a single
``std_msgs/Bool{data: true}`` onto ``/match/start`` so bt_game_node starts the
180 s match clock. It replaces the manual::

    ros2 topic pub --qos-durability transient_local /match/start \
        std_msgs/msg/Bool '{data: true}' --once

The publisher uses TRANSIENT_LOCAL durability so a late-joining bt_game_node
still receives the latched start (matches the BT's transient_local subscriber).

GPIO 4 (BCM) is free on this robot: motors use the L298N pins and the servos run
over I2C (PCA9685), so there is no conflict.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import Bool

# gpiozero is optional so the node can be unit-run on a dev laptop.
try:
    from gpiozero import Button, Device
    from gpiozero.pins.lgpio import LGPIOFactory, LGPIOPin
    from gpiozero.pins.local import LocalPiFactory
    import lgpio

    HAS_GPIOZERO = True
except Exception:  # ImportError on a laptop, or lgpio missing.
    HAS_GPIOZERO = False


BUTTON_BCM_PIN = 4          # BCM pin for the physical start button.
DEBOUNCE_S = 0.1            # Button bounce time.
TOPIC_MATCH_START = "/match/start"


if HAS_GPIOZERO:
    class RP1LGPIOFactory(LGPIOFactory):
        """Force gpiochip0 on the Pi 5 (RP1) — same fix as MotorsController.

        gpiozero 2.0.1's LGPIOFactory auto-detects gpiochip4, which no longer
        exists on recent Pi 5 kernels (the RP1 bank is gpiochip0), so the stock
        factory raises 'can not open gpiochip'. Force chip 0.
        """

        def __init__(self, chip=0):
            LocalPiFactory.__init__(self)
            self._handle = lgpio.gpiochip_open(chip)
            self._chip = chip
            self.pin_class = LGPIOPin


class MatchButtonNode(Node):
    """Publish a latched /match/start when the physical button is pressed."""

    def __init__(self) -> None:
        super().__init__("match_button_node")

        # Latched start so a bt_game_node that starts AFTER the press still sees it.
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._pub = self.create_publisher(Bool, TOPIC_MATCH_START, qos)
        self._started = False

        self._button = None
        if HAS_GPIOZERO:
            try:
                Device.pin_factory = RP1LGPIOFactory(chip=0)
                self._button = Button(BUTTON_BCM_PIN, bounce_time=DEBOUNCE_S)
                self._button.when_pressed = self._on_press
                self.get_logger().info(
                    f"Start button ready on BCM {BUTTON_BCM_PIN}. "
                    "Press to start the match."
                )
            except Exception as exc:  # pin busy / wrong chip / no hardware.
                self.get_logger().error(
                    f"Failed to bind start button on BCM {BUTTON_BCM_PIN}: {exc}. "
                    "Fall back to: ros2 topic pub --qos-durability transient_local "
                    "/match/start std_msgs/msg/Bool '{data: true}' --once"
                )
        else:
            self.get_logger().warn(
                "gpiozero unavailable — no physical button. Start the match with: "
                "ros2 topic pub --qos-durability transient_local /match/start "
                "std_msgs/msg/Bool '{data: true}' --once"
            )

    def _on_press(self) -> None:
        """GPIO callback: latch the start exactly once."""
        if self._started:
            return
        self._started = True
        self._pub.publish(Bool(data=True))
        self.get_logger().info("START button pressed — match start published.")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MatchButtonNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
