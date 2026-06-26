#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Twist

import unibots_control.MotorsController_M_bugFix as MC


class HardwareMotorNode(Node):
    def __init__(self):
        super().__init__('hardware_motor_node')

        self.cmd_sub_stamped = self.create_subscription(
            TwistStamped, '/cmd_vel', self.cmd_stamped_callback, 10)
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel_unstamped', self.cmd_callback, 10)

        self.get_logger().info("Hardware Motor Node started — listening on /cmd_vel")

    def cmd_stamped_callback(self, msg: TwistStamped):
        self.apply_movement(msg.twist)

    def cmd_callback(self, msg: Twist):
        self.apply_movement(msg)

    def apply_movement(self, twist: Twist):
        # MPC publishes: linear.x = lateral, linear.y = forward, angular.z = CCW-positive.
        # MotorsController expects CW-positive omega, so invert.
        MC.mecanum_move(
            vx=twist.linear.x,
            vy=twist.linear.y,
            omega=-twist.angular.z,
        )


def main(args=None):
    rclpy.init(args=args)
    node = HardwareMotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        MC.stop_robot()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
