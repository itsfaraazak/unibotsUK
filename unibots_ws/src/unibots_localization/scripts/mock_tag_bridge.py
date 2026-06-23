#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Mock AprilTag-to-pose bridge for Unibots UK 2026 -- simulation / bench testing only.

Stands in for the real ``camera -> apriltag_node -> ekf_bridge_node`` pipeline
so the EKF and motion controller can be exercised in Gazebo/sim before the
physical camera and detector are ready. Publishes to the SAME topic
ekf_bridge_node uses (``/localization/robot_pose``), so the EKF config
(``ekf.yaml``'s ``pose0``) and any RViz setup work unmodified against either
this node or the real one. NEVER run this alongside ekf_bridge_node -- they
would both publish to that topic and fight each other.
"""

import math
import random
from collections import namedtuple
from typing import Dict, List, Optional, Set, Tuple

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from geometry_msgs.msg import Point, PoseWithCovarianceStamped, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


# ---------------------------------------------------------------------------
# Tag specification
# ---------------------------------------------------------------------------
TagSpec = namedtuple("TagSpec", ["x", "y", "yaw_normal"])

ARENA_TAGS: Dict[int, TagSpec] = {
    # ---- North wall (y = 2.00, face normal -> SOUTH = -pi/2) --------------
    0:  TagSpec(0.15, 2.00, -math.pi / 2),
    1:  TagSpec(0.45, 2.00, -math.pi / 2),
    2:  TagSpec(0.75, 2.00, -math.pi / 2),
    3:  TagSpec(1.05, 2.00, -math.pi / 2),
    4:  TagSpec(1.35, 2.00, -math.pi / 2),
    5:  TagSpec(1.65, 2.00, -math.pi / 2),
    # ---- East wall  (x = 2.00, face normal -> WEST  = pi) ------------------
    6:  TagSpec(2.00, 0.15,  math.pi),
    7:  TagSpec(2.00, 0.45,  math.pi),
    8:  TagSpec(2.00, 0.75,  math.pi),
    9:  TagSpec(2.00, 1.05,  math.pi),
    10: TagSpec(2.00, 1.35,  math.pi),
    11: TagSpec(2.00, 1.65,  math.pi),
    # ---- South wall (y = 0.00, face normal -> NORTH = +pi/2) ---------------
    12: TagSpec(0.15, 0.00,  math.pi / 2),
    13: TagSpec(0.45, 0.00,  math.pi / 2),
    14: TagSpec(0.75, 0.00,  math.pi / 2),
    15: TagSpec(1.05, 0.00,  math.pi / 2),
    16: TagSpec(1.35, 0.00,  math.pi / 2),
    17: TagSpec(1.65, 0.00,  math.pi / 2),
    # ---- West wall  (x = 0.00, face normal -> EAST  = 0.0) -----------------
    18: TagSpec(0.00, 0.15,  0.0),
    19: TagSpec(0.00, 0.45,  0.0),
    20: TagSpec(0.00, 0.75,  0.0),
    21: TagSpec(0.00, 1.05,  0.0),
    22: TagSpec(0.00, 1.35,  0.0),
    23: TagSpec(0.00, 1.65,  0.0),
}

TAG_HEIGHT_M: float = 0.075

# Covariance model -- MUST MATCH ekf_bridge_node.py's DEFAULT_* constants
BASE_LINEAR_COV = 0.0100   
COV_DIST_K = 0.0400        
YAW_BASE_COV = 0.0120      
YAW_DIST_K = 0.0200        

UNUSED_COV: float = 1.0e6
MARKER_VISIBLE_RGBA = (0.1, 1.0, 0.2, 0.9)
MARKER_HIDDEN_RGBA = (0.5, 0.5, 0.5, 0.25)
MARKER_SCALE_M = 0.06
FACE_VISIBILITY_HALF_ANGLE: float = math.pi / 2.0


def _yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class MockTagBridge(Node):

    def __init__(self) -> None:
        super().__init__("mock_tag_bridge")

        self.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])

        self.declare_parameter("camera_range", 2.0)
        self.declare_parameter("camera_fov", 1.2)
        self.declare_parameter("publish_rate", 10.0)

        self._camera_range: float = self.get_parameter("camera_range").value
        self._half_fov: float = self.get_parameter("camera_fov").value / 2.0
        publish_rate: float = self.get_parameter("publish_rate").value

        self._tags: Dict[int, TagSpec] = ARENA_TAGS

        # ---- State ----------------------------------------------------------
        self._latest_gt: Optional[Tuple[float, float, float]] = None
        self._prev_visible_ids: Set[int] = set()
        self._pose_source: str = "none"

        # ---- Publishers / subscriptions -------------------------------------
        self._pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/localization/robot_pose", 10
        )
        self._marker_pub = self.create_publisher(
            MarkerArray, "/localization/debug_markers", 10
        )
        
        # Primary: Gazebo GT via specific model pose topic
        self.create_subscription(
            Pose, "/unibots_robot/pose", self._ground_truth_cb, 10
        )
        
        # Fallback: Raw odom (we map this mathematically to world pose if GT is dead)
        self.create_subscription(
            Odometry, "/odom", self._odom_fallback_cb, 10
        )
        
        self.create_timer(1.0 / publish_rate, self._timer_cb)

        self.get_logger().info(
            f"MockTagBridge ready | {publish_rate:.0f} Hz | "
            f"range {self._camera_range:.1f} m | "
            f"FOV {math.degrees(self._half_fov * 2):.0f} deg"
        )

    # ------------------------------------------------------------------ #
    # Callbacks
    # ------------------------------------------------------------------ #
    def _ground_truth_cb(self, msg: Pose) -> None:
        """Extract exact Gazebo simulation pose directly from the model topic."""
        # Note: Removed the extra .pose layer
        rx = msg.position.x
        ry = msg.position.y
        ryaw = _yaw_from_quaternion(msg.orientation)
        self._latest_gt = (rx, ry, ryaw)
        
        if self._pose_source != "gazebo":
            self.get_logger().info("Receiving True GT from Gazebo model pose topic!")
            self._pose_source = "gazebo"

    def _odom_fallback_cb(self, msg: Odometry) -> None:
        """Fallback mathematically mapping raw wheels to the world if GT is dead."""
        if self._pose_source == "gazebo":
            return  # Prefer Gazebo GT if it works
            
        ox = msg.pose.pose.position.x
        oy = msg.pose.pose.position.y
        oyaw = _yaw_from_quaternion(msg.pose.pose.orientation)

        # Apply the exact spawn coordinates from your sim.launch.py
        sx, sy, syaw = 1.0, 0.14, 1.5708

        # Rotate the relative odom vector into the world map, then add spawn offset
        rx = sx + (ox * math.cos(syaw) - oy * math.sin(syaw))
        ry = sy + (ox * math.sin(syaw) + oy * math.cos(syaw))
        ryaw = _wrap(syaw + oyaw)

        self._latest_gt = (rx, ry, ryaw)
        
        if self._pose_source != "odom":
            self.get_logger().info("Gazebo GT missing. Falling back to exact odom-mapping.")
            self._pose_source = "odom"

    # ------------------------------------------------------------------ #
    # Visibility
    # ------------------------------------------------------------------ #
    def _visible_tags(self, rx: float, ry: float, ryaw: float) -> Dict[int, float]:
        visible: Dict[int, float] = {}
        for tag_id, spec in self._tags.items():
            dx = spec.x - rx
            dy = spec.y - ry
            dist = math.hypot(dx, dy)

            if dist >= self._camera_range:
                continue

            bearing_robot_to_tag = _wrap(math.atan2(dy, dx) - ryaw)
            if abs(bearing_robot_to_tag) >= self._half_fov:
                continue

            bearing_tag_to_robot = math.atan2(ry - spec.y, rx - spec.x)
            face_error = abs(_wrap(bearing_tag_to_robot - spec.yaw_normal))
            if face_error >= FACE_VISIBILITY_HALF_ANGLE:
                continue

            visible[tag_id] = dist
        return visible

    # ------------------------------------------------------------------ #
    # Main timer callback
    # ------------------------------------------------------------------ #
    def _timer_cb(self) -> None:
        if self._latest_gt is None:
            self.get_logger().warning(
                "Waiting for robot pose... Neither Gazebo GT nor /odom received yet. Is the robot fully spawned?", 
                throttle_duration_sec=2.0
            )
            return

        rx, ry, ryaw = self._latest_gt
        visible = self._visible_tags(rx, ry, ryaw)

        self._publish_markers(visible.keys())

        cur_ids = set(visible.keys())
        if cur_ids != self._prev_visible_ids:
            gained = cur_ids - self._prev_visible_ids
            lost = self._prev_visible_ids - cur_ids
            parts = []
            if gained:
                parts.append(f'+[{",".join(str(i) for i in sorted(gained))}]')
            if lost:
                parts.append(f'-[{",".join(str(i) for i in sorted(lost))}]')
            ranges_str = ", ".join(
                f"{tid}:{r:.2f}m" for tid, r in sorted(visible.items())
            )
            self.get_logger().info(
                f"Tags visible: [{ranges_str}] ({len(visible)})  "
                f'{"  ".join(parts)}'
            )
            self._prev_visible_ids = cur_ids

        if not visible:
            return

        lin_w_sum = sum(1.0 / (BASE_LINEAR_COV + COV_DIST_K * r * r) for r in visible.values())
        yaw_w_sum = sum(1.0 / (YAW_BASE_COV + YAW_DIST_K * r * r) for r in visible.values())
        fused_lin_var = 1.0 / lin_w_sum
        fused_yaw_var = 1.0 / yaw_w_sum

        noisy_x = rx + random.gauss(0.0, math.sqrt(fused_lin_var))
        noisy_y = ry + random.gauss(0.0, math.sqrt(fused_lin_var))
        noisy_yaw = ryaw + random.gauss(0.0, math.sqrt(fused_yaw_var))

        self._publish_pose(noisy_x, noisy_y, noisy_yaw, fused_lin_var, fused_yaw_var)

    def _publish_pose(self, x: float, y: float, yaw: float, lin_var: float, yaw_var: float) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0

        half = yaw * 0.5
        msg.pose.pose.orientation.z = math.sin(half)
        msg.pose.pose.orientation.w = math.cos(half)

        cov = [0.0] * 36
        cov[0] = lin_var     
        cov[7] = lin_var     
        cov[14] = UNUSED_COV 
        cov[21] = UNUSED_COV 
        cov[28] = UNUSED_COV 
        cov[35] = yaw_var    
        msg.pose.covariance = cov

        self._pose_pub.publish(msg)

    def _publish_markers(self, visible_ids) -> None:
        visible = frozenset(visible_ids)
        stamp = self.get_clock().now().to_msg()
        array = MarkerArray()

        for tag_id, spec in self._tags.items():
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = stamp
            marker.ns = "arena_tags"
            marker.id = tag_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = Point(x=spec.x, y=spec.y, z=TAG_HEIGHT_M)
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = MARKER_SCALE_M

            r, g, b, a = MARKER_VISIBLE_RGBA if tag_id in visible else MARKER_HIDDEN_RGBA
            marker.color = ColorRGBA(r=r, g=g, b=b, a=a)
            array.markers.append(marker)

        self._marker_pub.publish(array)

def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = MockTagBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()