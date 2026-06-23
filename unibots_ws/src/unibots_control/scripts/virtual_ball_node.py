#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class VirtualBallNode(Node):
    def __init__(self):
        super().__init__('virtual_ball_node')
        
        # 1. Listen to RViz's default 2D Goal Pose tool
        self.create_subscription(PoseStamped, '/goal_pose', self.rviz_cb, 10)
        
        # 2. Stream to your MPC's target topic at 20Hz
        self.pub = self.create_publisher(PoseStamped, '/game/target', 10)
        self.create_timer(0.05, self.timer_cb)  # 20 Hz
        
        self.target_pose = None
        self.current_pose = None
        
        # Simulate how fast the ball rolls across the floor (meters/second)
        self.speed = 0.5  

        self.get_logger().info("Virtual Ball Node started! Click '2D Goal Pose' in RViz to roll the ball.")

    def rviz_cb(self, msg: PoseStamped):
        """Catches your mouse click from RViz."""
        self.target_pose = msg
        if self.current_pose is None:
            self.current_pose = msg

    def timer_cb(self):
        """Runs at 20 Hz to simulate camera frames."""
        if self.current_pose is None or self.target_pose is None:
            return

        # Calculate distance to the new RViz click
        dx = self.target_pose.pose.position.x - self.current_pose.pose.position.x
        dy = self.target_pose.pose.position.y - self.current_pose.pose.position.y
        dist = math.hypot(dx, dy)

        # Move the virtual ball smoothly towards the click
        step = self.speed * 0.05
        if dist > step:
            self.current_pose.pose.position.x += (dx / dist) * step
            self.current_pose.pose.position.y += (dy / dist) * step
        else:
            self.current_pose.pose.position.x = self.target_pose.pose.position.x
            self.current_pose.pose.position.y = self.target_pose.pose.position.y
        
        self.current_pose.pose.orientation = self.target_pose.pose.orientation

        # Update timestamp to right NOW (prevents your timeout logic from triggering)
        self.current_pose.header.stamp = self.get_clock().now().to_msg()
        
        # Stream the simulated camera frame to the MPC
        self.pub.publish(self.current_pose)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(VirtualBallNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()