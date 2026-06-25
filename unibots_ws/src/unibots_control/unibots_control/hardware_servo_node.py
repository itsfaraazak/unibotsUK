#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

# Import your existing physical servo controller
import unibots_control.ServoController_MK as SC

class HardwareServoNode(Node):
    def __init__(self):
        super().__init__('hardware_servo_node')
        
        self.scoop_sub = self.create_subscription(
            Bool, '/servos/scoop_cmd', self.scoop_callback, 10)
            
        self.deposit_sub = self.create_subscription(
            Bool, '/servos/deposit_cmd', self.deposit_callback, 10)

        self.get_logger().info("Hardware Servo Node Started.")
        
        # Ensure claw is open on startup
        SC.clutch_down()

    def scoop_callback(self, msg):
        if msg.data:
            self.get_logger().info("Scoop command received. Activating clutch.")
            SC.clutch_grabbing_motion()

    def deposit_callback(self, msg):
        if msg.data:
            self.get_logger().info("Deposit command received. Dumping storage.")
            # Unload sequence from your old thread
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