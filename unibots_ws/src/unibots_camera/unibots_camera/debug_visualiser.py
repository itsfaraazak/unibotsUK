#!/usr/bin/env python3
"""
debug_visualiser.py
Laptop-only debug tool. Shows live camera feed with detection overlays.
Not deployed on the Pi.

Run after starting camera_node and perception_node:
  ros2 run unibots_camera debug_visualiser
Press Q or Esc in the window to quit.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

import os

from unibots_msgs.msg import BallArray, ObstacleArray

# BGR colours
COL_PING_PONG = (0,   255,  50)   # green
COL_STEEL     = (0,   200, 255)   # yellow
COL_HUD       = (200, 200, 200)   # grey


class DebugVisualiser(Node):
    def __init__(self):
        super().__init__("debug_visualiser")

        self.bridge           = CvBridge()
        self.latest_balls     = None
        self.latest_obstacles = None

        sensor_qos = rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value

        self.create_subscription(
            Image, "/unibots/camera/image_raw", self.on_image, sensor_qos)
        self.create_subscription(
            BallArray, "/vision/balls", self.on_balls, 10)
        self.create_subscription(
            ObstacleArray, "/vision/obstacles", self.on_obstacles, 10)

        self._frame_count = 0
        os.makedirs("/tmp/unibots_debug", exist_ok=True)
        self.get_logger().info("Saving debug frames to /tmp/unibots_debug/")
        self.get_logger().info("Debug visualiser ready — press Q to quit")

    def on_balls(self, msg: BallArray):
        self.latest_balls = msg

    def on_obstacles(self, msg: ObstacleArray):
        self.latest_obstacles = msg

    def on_image(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge: {e}")
            return

        h, w = frame.shape[:2]

        # ── Balls ─────────────────────────────────────────────────────────────
        # BallDetection fields: ball_type, pixel_x, pixel_y, distance_cm,
        #                       bearing_deg, confidence, yolo_confirmed, track_id
        # No bbox — draw circle sized by distance
        if self.latest_balls:
            for ball in self.latest_balls.balls:
                cx = int(ball.pixel_x)
                cy = int(ball.pixel_y)

                # Radius scales with closeness — closer = bigger
                dist   = max(ball.distance_cm, 1.0)
                radius = int(max(8, min(60, 300 / dist)))
                colour = COL_PING_PONG if ball.ball_type == "ping_pong" else COL_STEEL

                cv2.circle(frame, (cx, cy), radius, colour, 2)
                cv2.drawMarker(frame, (cx, cy), colour, cv2.MARKER_CROSS, 10, 1)

                label = (f"{ball.ball_type} "
                         f"conf={ball.confidence:.2f} "
                         f"{ball.distance_cm:.0f}cm "
                         f"{ball.bearing_deg:+.1f}deg")
                cv2.putText(frame, label, (cx - radius, cy - radius - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, colour, 1)

        # ── Obstacles ─────────────────────────────────────────────────────────
        # ObstacleDetection fields: world_x, world_y, radius_m, is_confirmed_robot
        # No pixel coords at all — show warning bar when any are present
        if self.latest_obstacles and self.latest_obstacles.obstacles:
            n = len(self.latest_obstacles.obstacles)
            cv2.rectangle(frame, (0, h - 30), (w, h), (0, 0, 180), -1)
            cv2.putText(frame, f"WARNING: {n} robot obstacle(s)",
                        (10, h - 8), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (255, 255, 255), 2)

        # ── HUD ───────────────────────────────────────────────────────────────
        n_balls = len(self.latest_balls.balls)         if self.latest_balls     else 0
        n_obs   = len(self.latest_obstacles.obstacles) if self.latest_obstacles else 0
        cv2.putText(frame, f"balls={n_balls}  obstacles={n_obs}",
                    (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.65, COL_HUD, 2)

        # Centre bearing line
        cv2.line(frame, (w // 2, 0), (w // 2, 12), COL_HUD, 1)

        # Requires GUI OpenCV
        #cv2.imshow("Unibots perception debug", frame)
        #key = cv2.waitKey(1) & 0xFF
        #if key in (ord('q'), 27):
        #    raise SystemExit(0)

        # Saves detection to files in /tmp
        self._frame_count += 1
        if self._frame_count % 10 == 0:
            path = f"/tmp/unibots_debug/frame_{self._frame_count:06d}.jpg"
            cv2.imwrite(path, frame)
            self.get_logger().info(
                f"Saved frame {self._frame_count} | balls={n_balls} obstacles={n_obs}")


def main():
    rclpy.init()
    node = DebugVisualiser()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        # Only for GUI OpenCV
        # cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
