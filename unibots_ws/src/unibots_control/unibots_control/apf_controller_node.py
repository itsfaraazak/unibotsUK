#!/usr/bin/env python3
"""Artificial Potential Field (APF) fallback controller for the mecanum robot.

Used when the compiled iMPC solver is not available (not yet generated) or when
a lighter-weight controller is desired. It reproduces the team's existing Python
APF logic in ROS 2 and is a drop-in replacement for the MPC node: it subscribes
to the SAME topics and publishes the SAME /cmd_vel TwistStamped.

Method (standard APF):
    F_att = k_att * (goal - pos)                       (attractive, world frame)
    F_rep = sum over obstacles within influence radius of
            k_rep * (1/d - 1/d0) * (1/d^2) * (pos - obs)/||pos - obs||
    F     = F_att + F_rep                               (resultant, world frame)
The world-frame force is converted to a BODY-frame velocity command via the
inverse body-to-world rotation, and yaw is commanded to face the travel
direction. All velocities are clamped.

State  x = [px, py, theta]^T   (arena/map frame)
Control u = [vx, vy, omega]^T  (body frame: lateral, forward, yaw rate)
"""

import math
from typing import List, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from vision_msgs.msg import Detection2DArray

# ----------------------------------------------------------------------------
# Named constants (NO magic numbers).
# ----------------------------------------------------------------------------
QOS_DEPTH: int = 10
LOG_THROTTLE_S: float = 2.0

# Below this distance two points are treated as coincident (avoid div-by-zero).
MIN_DISTANCE: float = 1e-6
# Below this resultant-force magnitude we treat the robot as "at rest" and do
# not command a new heading (avoids jittery yaw near the goal).
MIN_FORCE_FOR_HEADING: float = 1e-3

# Gain on the yaw P-controller that turns the robot toward its travel direction.
YAW_KP: float = 1.5

TOPIC_ODOM: str = "/odom/filtered"
TOPIC_TARGET: str = "/game/target"
TOPIC_OBSTACLES: str = "/vision/obstacles"
TOPIC_CMD_VEL: str = "/cmd_vel"


def yaw_from_quaternion(q: Quaternion) -> float:
    """Extract the yaw angle from a geometry_msgs Quaternion.

    Args:
        q: orientation quaternion.

    Returns:
        Yaw [rad] in (-pi, pi].
    """
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(angle: float) -> float:
    """Wrap an angle to (-pi, pi].

    Args:
        angle: input angle [rad].

    Returns:
        Wrapped angle [rad].
    """
    return math.atan2(math.sin(angle), math.cos(angle))


class ApfControllerNode(Node):
    """ROS 2 node implementing an APF holonomic controller on /cmd_vel."""

    def __init__(self) -> None:
        """Construct the node, declare params, set up I/O and the control timer."""
        super().__init__("apf_controller_node")

        # --- Declare + read parameters --------------------------------------
        self.declare_parameter("k_att", 1.0)
        self.declare_parameter("k_rep", 0.3)
        self.declare_parameter("influence_radius", 0.5)
        self.declare_parameter("max_vx", 0.35)
        self.declare_parameter("max_vy", 0.45)
        self.declare_parameter("max_omega", 1.2)
        self.declare_parameter("goal_tolerance", 0.05)
        self.declare_parameter("control_frequency", 20.0)

        self._k_att: float = self.get_parameter("k_att").value
        self._k_rep: float = self.get_parameter("k_rep").value
        self._influence: float = self.get_parameter("influence_radius").value
        self._max_vx: float = self.get_parameter("max_vx").value
        self._max_vy: float = self.get_parameter("max_vy").value
        self._max_omega: float = self.get_parameter("max_omega").value
        self._goal_tolerance: float = self.get_parameter("goal_tolerance").value
        self._control_freq: float = self.get_parameter("control_frequency").value

        # --- Runtime state ---------------------------------------------------
        self._current_pose: Optional[np.ndarray] = None   # [px, py, theta]
        self._goal: Optional[np.ndarray] = None           # [px, py, theta]
        self._obstacles: List[np.ndarray] = []            # list of [ox, oy, r]
        # Latches "arrival" so we log "Goal reached" once on transition rather
        # than every control tick while holding. Cleared when a new goal arrives.
        self._goal_reached: bool = False

        # --- QoS: RELIABLE, depth 10 ----------------------------------------
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=QOS_DEPTH,
        )

        self.create_subscription(Odometry, TOPIC_ODOM, self._odom_cb, qos)
        self.create_subscription(PoseStamped, TOPIC_TARGET, self._target_cb, qos)
        self.create_subscription(
            Detection2DArray, TOPIC_OBSTACLES, self._obstacles_cb, qos
        )

        self._cmd_pub = self.create_publisher(TwistStamped, TOPIC_CMD_VEL, qos)

        self.create_timer(1.0 / self._control_freq, self._control_loop)

        self.get_logger().info(
            f"ApfControllerNode up: k_att={self._k_att}, k_rep={self._k_rep}, "
            f"influence={self._influence} m, freq={self._control_freq} Hz."
        )

    # ------------------------------------------------------------------------
    # Subscription callbacks
    # ------------------------------------------------------------------------
    def _odom_cb(self, msg: Odometry) -> None:
        """Store current pose, extracting yaw from the odometry quaternion."""
        p = msg.pose.pose.position
        theta = yaw_from_quaternion(msg.pose.pose.orientation)
        self._current_pose = np.array([p.x, p.y, theta], dtype=float)

    def _target_cb(self, msg: PoseStamped) -> None:
        """Store the goal pose and re-arm the arrival latch for the new goal."""
        p = msg.pose.position
        theta = yaw_from_quaternion(msg.pose.orientation)
        self._goal = np.array([p.x, p.y, theta], dtype=float)
        self._goal_reached = False

    def _obstacles_cb(self, msg: Detection2DArray) -> None:
        """Store obstacles from a vision_msgs/Detection2DArray.

        Field mapping (same as the MPC node):
            bbox.center.position.x -> obstacle world_x [m]
            bbox.center.position.y -> obstacle world_y [m]
            bbox.size_x            -> obstacle radius   [m]
        """
        obstacles: List[np.ndarray] = []
        for det in msg.detections:
            ox = det.bbox.center.position.x
            oy = det.bbox.center.position.y
            radius = det.bbox.size_x
            obstacles.append(np.array([ox, oy, radius], dtype=float))
        self._obstacles = obstacles

    # ------------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------------
    def _publish_zero(self) -> None:
        """Publish a zero-velocity TwistStamped (safe stop)."""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        self._cmd_pub.publish(msg)

    def _compute_force(self) -> np.ndarray:
        """Compute the resultant APF force in the WORLD frame.

        Returns:
            2-vector [Fx, Fy] in the arena/map frame.
        """
        assert self._current_pose is not None and self._goal is not None
        pos = self._current_pose[0:2]

        # Attractive force pulls the robot toward the goal.
        f_att = self._k_att * (self._goal[0:2] - pos)

        # Repulsive forces push away from each obstacle within influence radius.
        f_rep = np.zeros(2, dtype=float)
        for obs in self._obstacles:
            obs_xy = obs[0:2]
            obs_radius = float(obs[2])
            diff = pos - obs_xy
            dist = float(np.linalg.norm(diff))
            # Effective surface distance accounts for the obstacle's radius.
            surf_dist = max(dist - obs_radius, MIN_DISTANCE)
            if surf_dist < self._influence:
                # Standard APF repulsion magnitude.
                mag = self._k_rep * (1.0 / surf_dist - 1.0 / self._influence) \
                    / (surf_dist * surf_dist)
                f_rep += mag * (diff / max(dist, MIN_DISTANCE))

        return f_att + f_rep

    # ------------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------------
    def _control_loop(self) -> None:
        """APF control tick (runs on the control-frequency timer)."""
        # No goal or no pose -> publish zero.
        if self._current_pose is None or self._goal is None:
            self._publish_zero()
            return

        # Goal reached -> stop and hold.
        dist_to_goal = float(
            np.linalg.norm(self._goal[0:2] - self._current_pose[0:2])
        )
        if dist_to_goal < self._goal_tolerance:
            self._publish_zero()
            # Log once on arrival, not every tick while holding.
            if not self._goal_reached:
                self._goal_reached = True
                self.get_logger().info("Goal reached (within tolerance) -> holding.")
            return
        # Moved away from the goal again -> re-arm so a future arrival re-logs.
        self._goal_reached = False

        # Resultant world-frame force.
        force_world = self._compute_force()

        # Convert world-frame force to a BODY-frame velocity command using the
        # inverse body-to-world rotation R(theta)^T = R(-theta).
        # WHY full (vx, vy): a mecanum base is holonomic, so we command BOTH a
        # body-x (strafe) and body-y velocity at once -- true omnidirectional
        # motion. This is the key advantage over differential drive, which must
        # rotate to change travel direction; the mecanum can slide directly
        # along the resultant force without reorienting.
        theta = float(self._current_pose[2])
        c, s = math.cos(theta), math.sin(theta)
        # R(-theta) applied to the world force gives the body-frame command.
        vx_body = c * force_world[0] + s * force_world[1]
        vy_body = -s * force_world[0] + c * force_world[1]

        # Yaw control: face the direction of travel (world-frame force heading).
        # DESIGN CHOICE: we steer heading toward the travel direction rather
        # than toward the goal, so the robot's front leads its actual motion
        # (cleaner for sensors mounted forward). Skip when nearly at rest to
        # avoid yaw jitter.
        force_mag = float(np.linalg.norm(force_world))
        if force_mag > MIN_FORCE_FOR_HEADING:
            desired_yaw = math.atan2(force_world[1], force_world[0])
            yaw_err = wrap_angle(desired_yaw - theta)
            omega = YAW_KP * yaw_err
        else:
            omega = 0.0

        # Clamp velocities to the configured limits.
        vx = float(np.clip(vx_body, -self._max_vx, self._max_vx))
        vy = float(np.clip(vy_body, -self._max_vy, self._max_vy))
        omega = float(np.clip(omega, -self._max_omega, self._max_omega))

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.angular.z = omega
        self._cmd_pub.publish(msg)


def main(args=None) -> None:
    """Entry point: init rclpy, spin the APF controller node, clean up."""
    rclpy.init(args=args)
    node = ApfControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
