#!/usr/bin/env python3
"""Hardware servo node — bridges /servo/command (String) to physical servos.

Command mapping (published by bt_game_node):
  "SCOOP"  → claw (clutch positional servo): clutch_grabbing_motion()
              Fired once when ball enters intake blind-spot.
  "OPEN"   → deposit trapdoor (MG90S continuous servo): turn 180° up, wait, return.
              Fired at start of DUMP state.
  "CLOSE"  → no-op (bt_game_node sends this after dump_duration_s; servo already returned).

Both SCOOP and OPEN run in daemon threads to avoid blocking the ROS spin loop.
A lock prevents two servo actions running simultaneously.
"""
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import unibots_control.ServoController_MK as SC


class HardwareServoNode(Node):
    def __init__(self):
        super().__init__('hardware_servo_node')

        self._lock = threading.Lock()

        self.servo_sub = self.create_subscription(
            String, '/servo/command', self.servo_callback, 10)

        self.get_logger().info("Hardware Servo Node started — listening on /servo/command")

        # Clutch starts in resting (down) position.
        SC.clutch_down()

    def servo_callback(self, msg: String):
        cmd = msg.data.strip().upper()
        if cmd == 'SCOOP':
            self._run_async(self._do_scoop, "SCOOP")
        elif cmd == 'OPEN':
            self._run_async(self._do_deposit, "OPEN/deposit")
        elif cmd == 'CLOSE':
            self.get_logger().debug("CLOSE received (no-op)")
        else:
            self.get_logger().warn(f"Unknown servo command: '{msg.data}'")

    def _run_async(self, fn, label: str):
        if self._lock.locked():
            self.get_logger().warn(f"Servo busy — dropping {label} command")
            return
        t = threading.Thread(target=self._locked_run, args=(fn, label), daemon=True)
        t.start()

    def _locked_run(self, fn, label: str):
        with self._lock:
            self.get_logger().info(f"Servo: executing {label}")
            fn()

    def _do_scoop(self):
        SC.clutch_grabbing_motion()   # up → sleep(1.3) → down

    def _do_deposit(self):
        SC.mg90s_turn_by_180_up()
        SC.safe_sleep(2.0)
        SC.mg90s_turn_by_180_down()


def main(args=None):
    rclpy.init(args=args)
    node = HardwareServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
