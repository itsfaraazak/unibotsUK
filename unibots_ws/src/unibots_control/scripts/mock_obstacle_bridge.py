#!/usr/bin/env python3
"""
Mock Obstacle Bridge for MPC Testing.
Simulates a camera detecting obstacles (like opponent robots).
It checks distance and FOV from the robot's Ground Truth pose and 
publishes visible obstacles to /vision/obstacles for the MPC to avoid.
"""

import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from vision_msgs.msg import Detection2DArray, Detection2D
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

# ---------------------------------------------------------------------------
# Obstacle Specification: (x, y, radius) in meters
# We use radius=0.15m as a safe bounding sphere for the 20x20cm cuboid in the MPC.
# ---------------------------------------------------------------------------
MOCK_OBSTACLES = [
    (0.6, 0.6, 0.15),   # Bottom-Left quadrant
    (1.4, 0.6, 0.15),   # Bottom-Right quadrant
    (0.6, 1.4, 0.15),   # Top-Left quadrant
    (1.4, 1.4, 0.15)    # Top-Right quadrant
]

def _yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))

class MockObstacleBridge(Node):
    def __init__(self):
        super().__init__("mock_obstacle_bridge")

        # Camera Settings
        self._camera_range = 2.0  # Meters
        self._half_fov = math.radians(70.0) / 2.0  # 70 degree FOV

        # Publishers
        # The MPC node expects this specific topic and message type
        self._obs_pub = self.create_publisher(Detection2DArray, "/vision/obstacles", 10)
        # RViz markers so you can visually see when the robot "sees" the obstacle
        self._marker_pub = self.create_publisher(MarkerArray, "/vision/debug_obstacles", 10)

        # Ground Truth Subscription (Same as your tag bridge)
        self.create_subscription(Pose, "/unibots_robot/pose", self._gt_cb, 10)

        self.get_logger().info("Mock Obstacle Bridge Ready. Waiting for GT pose...")

    def _gt_cb(self, msg: Pose) -> None:
        """Runs every time Gazebo publishes the exact robot location."""
        rx = msg.position.x
        ry = msg.position.y
        ryaw = _yaw_from_quaternion(msg.orientation)

        visible_obstacles = []
        
        # 1. FOV and Distance Math
        for idx, (ox, oy, radius) in enumerate(MOCK_OBSTACLES):
            dx = ox - rx
            dy = oy - ry
            dist = math.hypot(dx, dy)

            if dist > self._camera_range:
                continue

            bearing_to_obs = _wrap(math.atan2(dy, dx) - ryaw)
            if abs(bearing_to_obs) > self._half_fov:
                continue

            # It's inside the camera cone!
            visible_obstacles.append((idx, ox, oy, radius))

        # 2. Publish to MPC
        self._publish_detections(visible_obstacles)
        
        # 3. Publish to RViz for debugging
        self._publish_markers(visible_obstacles)

    def _publish_detections(self, visible_obs: List[Tuple[int, float, float, float]]):
        """Creates the Detection2DArray the MPC expects."""
        msg = Detection2DArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        for _, ox, oy, radius in visible_obs:
            det = Detection2D()
            # MPC reads these exact fields for World X, World Y, and Radius
            det.bbox.center.position.x = ox
            det.bbox.center.position.y = oy
            det.bbox.size_x = radius
            msg.detections.append(det)

        self._obs_pub.publish(msg)

    def _publish_markers(self, visible_obs: List[Tuple[int, float, float, float]]):
        """Draws obstacles in RViz. Turns RED when seen, GREY when hidden."""
        visible_indices = {obs[0] for obs in visible_obs}
        stamp = self.get_clock().now().to_msg()
        array = MarkerArray()

        for idx, (ox, oy, radius) in enumerate(MOCK_OBSTACLES):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = stamp
            marker.ns = "mock_obstacles"
            marker.id = idx
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = ox
            marker.pose.position.y = oy
            marker.pose.position.z = 0.15 # Half-height of 0.3m

            # Cuboid actual dimensions (20x20x30 cm)
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.3

            # Red if visible, semi-transparent grey if hidden
            if idx in visible_indices:
                marker.color = ColorRGBA(r=1.0, g=0.1, b=0.1, a=0.9)
            else:
                marker.color = ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.3)

            array.markers.append(marker)

        self._marker_pub.publish(array)

def main(args=None):
    rclpy.init(args=args)
    node = MockObstacleBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()