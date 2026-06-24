#!/usr/bin/env python3
"""
camera_node.py
Reads from a webcam (or any V4L2 device) and publishes to
/unibots/camera/image_raw as sensor_msgs/Image at a target FPS.

Works on:
  - Laptop integrated webcam  → /dev/video0
  - Pi USB webcam             → /dev/video0 (or video1 if Pi cam is video0)

Usage:
  python3 camera_node.py                     # default: /dev/video0, 30fps, 640x480
  python3 camera_node.py --device 2          # use /dev/video2
  python3 camera_node.py --width 320 --height 240 --fps 15

  Or as a ROS 2 node with params:
  ros2 run unibots_perception camera_node --ros-args \
    -p device_id:=0 -p fps:=15 -p width:=320 -p height:=240
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import argparse
import sys


class CameraNode(Node):
    def __init__(self):
        super().__init__("camera_node")

        # ── ROS params (override on command line with --ros-args -p key:=val) ─
        self.declare_parameter("device_id", 0)      # /dev/video<N>
        self.declare_parameter("width",     640)
        self.declare_parameter("height",    480)
        self.declare_parameter("fps",       30)
        self.declare_parameter("topic",     "/unibots/camera/image_raw")
        self.declare_parameter("frame_id",  "camera")

        device_id = self.get_parameter("device_id").value
        width     = self.get_parameter("width").value
        height    = self.get_parameter("height").value
        fps       = self.get_parameter("fps").value
        topic     = self.get_parameter("topic").value
        frame_id  = self.get_parameter("frame_id").value

        # ── Open webcam ───────────────────────────────────────────────────────
        self.cap = cv2.VideoCapture(device_id)

        if not self.cap.isOpened():
            self.get_logger().error(
                f"Could not open /dev/video{device_id}. "
                f"Check: ls /dev/video*"
            )
            raise SystemExit(1)

        # Request resolution + FPS from the driver
        # (driver may not honour exactly — actual values printed below)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS,          fps)

        # Read back what we actually got
        actual_w   = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h   = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)

        self.bridge   = CvBridge()
        self.frame_id = frame_id

        # ── Publisher ─────────────────────────────────────────────────────────
        self.pub = self.create_publisher(Image, topic, 10)

        # Timer fires at requested FPS
        self.timer = self.create_timer(1.0 / fps, self.capture_and_publish)

        self.get_logger().info(
            f"Camera /dev/video{device_id} opened | "
            f"requested {width}x{height}@{fps}fps | "
            f"actual {actual_w}x{actual_h}@{actual_fps:.1f}fps | "
            f"publishing → {topic}"
        )

    def capture_and_publish(self):
        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().warn("Failed to capture frame — is the camera disconnected?")
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.pub.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main():
    rclpy.init()
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
