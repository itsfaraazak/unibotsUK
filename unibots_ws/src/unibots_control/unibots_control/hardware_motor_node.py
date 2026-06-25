    #!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Twist

# Import your existing physical motor controller
import unibots_control.MotorsController_M_bugFix as MC

class HardwareMotorNode(Node):
    def __init__(self):
        super().__init__('hardware_motor_node')
        
        # Subscribe to standard Twist and TwistStamped (used by MPC)
        self.cmd_sub_stamped = self.create_subscription(
            TwistStamped, '/cmd_vel', self.cmd_stamped_callback, 10)
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel_unstamped', self.cmd_callback, 10)

        self.get_logger().info("Hardware Motor Node Started. Listening to /cmd_vel")

    def cmd_stamped_callback(self, msg: TwistStamped):
        self.apply_movement(msg.twist)

    def cmd_callback(self, msg: Twist):
        self.apply_movement(msg)

   def apply_movement(self, twist: Twist):
        # Swap X and Y to match the MPC's non-standard Twist mapping
        lateral_speed = twist.linear.x 
        forward_speed = twist.linear.y 
        
        # Invert the Z-axis to match the motor controller's CW-positive expectation
        rotation_speed = -twist.angular.z 
        
        # Pass them to your existing mecanum mixer
        # MC.mecanum_move uses: vx (lateral), vy (forward), omega (rotation)
        MC.mecanum_move(vx=lateral_speed, vy=forward_speed, omega=rotation_speed)

def main(args=None):
    rclpy.init(args=args)
    node = HardwareMotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        MC.stop_robot() # Fail-safe stop
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()