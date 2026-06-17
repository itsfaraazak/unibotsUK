#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""AprilTag-to-absolute-pose bridge for the Unibots UK 2026 mecanum robot.

WHY THIS NODE EXISTS
====================
``apriltag_ros`` (the ``apriltag_node``) detects AprilTags and publishes their
pose *relative to the camera* -- i.e. "tag-in-camera" transforms (where the tag
is, expressed in the camera optical frame). It has no idea where the robot is in
the arena.

``robot_localization``'s EKF (``ekf_filter_node``), on the other hand, wants to
fuse an *absolute* robot pose -- "robot-in-world" -- expressed in the ``map``
frame. It cannot consume a tag-relative camera pose directly.

This node is the bridge between the two:

    apriltag_node  --(tag-in-camera)-->  EkfBridgeNode  --(robot-in-map)-->  EKF

It works because the 24 arena tags are at *known, fixed* world positions (the
arena is surveyed -- see ``TAG_WORLD_POSITIONS``). Given:

  * the known world position of a detected tag, and
  * the measured position of that tag relative to the camera,

we can invert the chain and solve for the robot's absolute pose in the arena.
The result is published as a ``PoseWithCovarianceStamped`` on
``/localization/robot_pose`` for the EKF to fuse against the wheel odometry and
IMU.

GEOMETRY (documented in detail at the call sites below)
=======================================================
We treat localization as a 2-D problem (flat arena, ``two_d_mode`` in the EKF).
For each detected tag we obtain, via a TF lookup, the transform from the camera
optical frame to the tag frame. We project that into the robot ``base_link``
ground plane to get a range ``r`` and bearing ``beta`` to the tag. Because we
know the tag's absolute ``(world_x, world_y)`` *and* the direction its face
normal points (it is mounted flat on a known wall), we can back out the robot's
absolute ``(x, y, yaw)``.

COVARIANCE
==========
Reported covariance grows with detection distance: a tag seen from far away is
measured less accurately (pixel error maps to larger metric error, and pose
ambiguity at shallow angles worsens). We use ``var = base + k * r**2`` so the EKF
automatically trusts close detections more than distant ones.

MULTIPLE TAGS
=============
When several tags are visible we *average* the resulting robot pose estimates,
inverse-variance weighted by their per-detection covariance. Averaging across
independent tags reduces noise and is robust to a single bad/occluded tag (an
opponent robot may block one tag, but rarely all of them simultaneously). The
closest tag dominates the weighted mean, which is the behaviour we want.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

# tf2 is used to look up the camera->tag transform that carries the actual
# measured tag-relative pose. apriltag_node publishes these on /tf.
import tf2_ros
from rclpy.duration import Duration
from rclpy.time import Time

# ---------------------------------------------------------------------------
# AprilTag message import.
#
# In ROS 2 Jazzy the modern apriltag stack (christianrauch/apriltag_ros) ships
# AprilTagDetectionArray in the `apriltag_msgs` package. If your distro instead
# provides the older `apriltag_ros/AprilTagDetectionArray`, swap the import
# below. The detection message in apriltag_msgs does NOT carry a full 6-DoF
# pose -- it carries the tag id, centre and corner pixels -- so the actual
# metric tag pose is obtained via the TF tree (camera_optical_frame -> tagXX),
# which apriltag_node broadcasts. We therefore subscribe to the detections only
# to learn *which* tag ids are currently visible, then TF-lookup each one.
# ---------------------------------------------------------------------------
try:
    from apriltag_msgs.msg import AprilTagDetectionArray

    _HAVE_APRILTAG_MSGS = True
except ImportError:  # pragma: no cover - depends on system install
    AprilTagDetectionArray = None  # type: ignore[assignment]
    _HAVE_APRILTAG_MSGS = False


# ===========================================================================
# Arena / tag specification (Unibots UK 2026)
# ===========================================================================
# 24 AprilTags (36h11, 100x100 mm). Arena is 2.0 x 2.0 m, SW corner = (0, 0).
# Tags are spaced 300 mm apart, the first one 150 mm in from the corner, so the
# i-th tag on a wall sits at offset 0.15 + 0.30*i for i in 0..5.
ARENA_SIZE_M: float = 2.0
TAG_SIZE_M: float = 0.100
TAG_HEIGHT_M: float = 0.075  # centre height above the floor
TAG_FIRST_OFFSET_M: float = 0.15
TAG_SPACING_M: float = 0.30
TAGS_PER_WALL: int = 6

# Wall names.
WALL_NORTH = "north"  # y = 2.0, normal points -y (into the arena, i.e. south)
WALL_EAST = "east"    # x = 2.0, normal points -x (west)
WALL_SOUTH = "south"  # y = 0.0, normal points +y (north)
WALL_WEST = "west"    # x = 0.0, normal points +x (east)


@dataclass(frozen=True)
class TagPose:
    """Surveyed world pose of a single arena tag.

    Attributes:
        id: AprilTag id (0..23).
        x: Tag centre world X in metres (``map`` frame).
        y: Tag centre world Y in metres (``map`` frame).
        wall: Which arena wall the tag is mounted on.
        yaw_normal: Yaw (rad, world frame) of the tag's outward face normal --
            the direction the tag *faces*, i.e. the direction it looks toward
            the centre of the arena. A camera must be roughly opposite this to
            see the tag.
    """

    id: int
    x: float
    y: float
    wall: str
    yaw_normal: float


def _build_tag_table() -> Dict[int, TagPose]:
    """Construct the surveyed world positions for all 24 arena tags.

    Returns:
        Mapping of tag id -> :class:`TagPose`. Computed directly from the arena
        spec so there are no transcription errors.
    """
    table: Dict[int, TagPose] = {}

    def offset(i: int) -> float:
        return TAG_FIRST_OFFSET_M + TAG_SPACING_M * i

    # North wall: IDs 0-5 at y = 2.0, x = 0.15 + 0.30*i. Face normal -> south.
    for i in range(TAGS_PER_WALL):
        tid = 0 + i
        table[tid] = TagPose(tid, offset(i), ARENA_SIZE_M, WALL_NORTH,
                             yaw_normal=-math.pi / 2.0)

    # East wall: IDs 6-11 at x = 2.0, y = 0.15 + 0.30*i. Face normal -> west.
    for i in range(TAGS_PER_WALL):
        tid = 6 + i
        table[tid] = TagPose(tid, ARENA_SIZE_M, offset(i), WALL_EAST,
                             yaw_normal=math.pi)

    # South wall: IDs 12-17 at y = 0.0, x = 0.15 + 0.30*i. Face normal -> north.
    for i in range(TAGS_PER_WALL):
        tid = 12 + i
        table[tid] = TagPose(tid, offset(i), 0.0, WALL_SOUTH,
                             yaw_normal=math.pi / 2.0)

    # West wall: IDs 18-23 at x = 0.0, y = 0.15 + 0.30*i. Face normal -> east.
    for i in range(TAGS_PER_WALL):
        tid = 18 + i
        table[tid] = TagPose(tid, 0.0, offset(i), WALL_WEST, yaw_normal=0.0)

    return table


# Hardcoded surveyed tag world positions. The node can also optionally load
# config/tags.yaml, but having them in code guarantees the bridge works even if
# the param file is missing.
TAG_WORLD_POSITIONS: Dict[int, TagPose] = _build_tag_table()


# ===========================================================================
# Default node parameters (overridable via the parameter server / launch file)
# ===========================================================================
DEFAULT_DETECTIONS_TOPIC = "/apriltag/detections"
DEFAULT_OUTPUT_TOPIC = "/localization/robot_pose"
DEFAULT_MAP_FRAME = "map"
DEFAULT_BASE_FRAME = "base_link"
DEFAULT_CAMERA_OPTICAL_FRAME = "camera_optical_frame"
DEFAULT_TAG_FRAME_PREFIX = "tag36h11:"  # apriltag_node default child frame name

# Covariance model: var(r) = base_cov + cov_dist_k * r**2  [m^2].
DEFAULT_BASE_LINEAR_COV = 0.0025   # 5 cm 1-sigma at zero range
DEFAULT_COV_DIST_K = 0.01          # grows with range^2
DEFAULT_YAW_BASE_COV = 0.0030      # rad^2, ~3.1 deg 1-sigma at zero range
DEFAULT_YAW_DIST_K = 0.0050        # rad^2 per m^2

# Robust limits.
DEFAULT_MAX_RANGE_M = 3.5          # ignore implausibly distant detections
DEFAULT_TF_TIMEOUT_S = 0.05        # how long to wait for a TF when looking up

# Large value placed on the unused Z / roll / pitch covariance diagonal entries
# so the EKF effectively ignores them (we are a 2-D filter).
UNUSED_COV = 1.0e6


@dataclass
class RobotEstimate:
    """A single robot-pose estimate derived from one tag detection.

    Attributes:
        x: Estimated robot world X (m).
        y: Estimated robot world Y (m).
        yaw: Estimated robot world yaw (rad).
        lin_var: Linear (x, y) variance for this estimate (m^2).
        yaw_var: Yaw variance for this estimate (rad^2).
        range_m: Distance from robot to the tag (m), for diagnostics/weighting.
    """

    x: float
    y: float
    yaw: float
    lin_var: float
    yaw_var: float
    range_m: float


def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Extract the Z (yaw) Euler angle from a quaternion.

    Args:
        qx, qy, qz, qw: Quaternion components.

    Returns:
        Yaw in radians in ``(-pi, pi]``.
    """
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    """Convert a planar yaw into a quaternion (x, y, z, w).

    Args:
        yaw: Yaw angle in radians.

    Returns:
        Quaternion tuple ``(x, y, z, w)`` representing a rotation about +Z.
    """
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def normalize_angle(angle: float) -> float:
    """Wrap an angle to ``(-pi, pi]``.

    Args:
        angle: Angle in radians.

    Returns:
        Equivalent angle wrapped into ``(-pi, pi]``.
    """
    return math.atan2(math.sin(angle), math.cos(angle))


class EkfBridgeNode(Node):
    """Converts tag-relative AprilTag observations into absolute robot poses.

    See the module docstring for the rationale and geometry. The node is
    callback-driven: every ``AprilTagDetectionArray`` triggers a TF lookup for
    each visible tag, computes a per-tag robot-pose estimate, fuses them with
    inverse-variance weighting, and publishes the result for the EKF.
    """

    def __init__(self) -> None:
        super().__init__("ekf_bridge_node")

        # ---- Declared parameters (read once at startup) -------------------
        self._detections_topic = self.declare_parameter(
            "detections_topic", DEFAULT_DETECTIONS_TOPIC
        ).get_parameter_value().string_value
        self._output_topic = self.declare_parameter(
            "output_topic", DEFAULT_OUTPUT_TOPIC
        ).get_parameter_value().string_value
        self._map_frame = self.declare_parameter(
            "map_frame", DEFAULT_MAP_FRAME
        ).get_parameter_value().string_value
        self._base_frame = self.declare_parameter(
            "base_frame", DEFAULT_BASE_FRAME
        ).get_parameter_value().string_value
        self._camera_frame = self.declare_parameter(
            "camera_optical_frame", DEFAULT_CAMERA_OPTICAL_FRAME
        ).get_parameter_value().string_value
        self._tag_frame_prefix = self.declare_parameter(
            "tag_frame_prefix", DEFAULT_TAG_FRAME_PREFIX
        ).get_parameter_value().string_value

        self._base_linear_cov = self.declare_parameter(
            "base_linear_cov", DEFAULT_BASE_LINEAR_COV
        ).get_parameter_value().double_value
        self._cov_dist_k = self.declare_parameter(
            "cov_dist_k", DEFAULT_COV_DIST_K
        ).get_parameter_value().double_value
        self._yaw_base_cov = self.declare_parameter(
            "yaw_base_cov", DEFAULT_YAW_BASE_COV
        ).get_parameter_value().double_value
        self._yaw_dist_k = self.declare_parameter(
            "yaw_dist_k", DEFAULT_YAW_DIST_K
        ).get_parameter_value().double_value
        self._max_range = self.declare_parameter(
            "max_range_m", DEFAULT_MAX_RANGE_M
        ).get_parameter_value().double_value
        self._tf_timeout = self.declare_parameter(
            "tf_timeout_s", DEFAULT_TF_TIMEOUT_S
        ).get_parameter_value().double_value

        # ---- TF buffer / listener (camera_optical_frame -> tag) -----------
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ---- QoS ----------------------------------------------------------
        # Sensor-style: best-effort, keep-last small queue. Detections are a
        # high-rate stream where the latest is what matters.
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        # Reliable output so the EKF does not miss absolute corrections.
        pose_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, self._output_topic, pose_qos
        )

        if _HAVE_APRILTAG_MSGS:
            self._det_sub = self.create_subscription(
                AprilTagDetectionArray,
                self._detections_topic,
                self._on_detections,
                sensor_qos,
            )
        else:
            self.get_logger().error(
                "apriltag_msgs not found: cannot subscribe to "
                f"'{self._detections_topic}'. Install ros-jazzy-apriltag-msgs."
            )

        self.get_logger().info(
            f"EkfBridgeNode ready. Listening on '{self._detections_topic}', "
            f"publishing absolute robot pose on '{self._output_topic}' "
            f"(frame '{self._map_frame}')."
        )

    # ------------------------------------------------------------------ #
    # Covariance model
    # ------------------------------------------------------------------ #
    def _linear_variance(self, range_m: float) -> float:
        """Linear position variance for a detection at the given range.

        ``var = base_linear_cov + cov_dist_k * range^2``. Distant tags are
        measured less accurately, so the EKF should trust them less.

        Args:
            range_m: Robot-to-tag distance in metres.

        Returns:
            Variance in m^2.
        """
        return self._base_linear_cov + self._cov_dist_k * range_m * range_m

    def _yaw_variance(self, range_m: float) -> float:
        """Yaw variance for a detection at the given range.

        Args:
            range_m: Robot-to-tag distance in metres.

        Returns:
            Variance in rad^2.
        """
        return self._yaw_base_cov + self._yaw_dist_k * range_m * range_m

    # ------------------------------------------------------------------ #
    # Detection callback
    # ------------------------------------------------------------------ #
    def _on_detections(self, msg) -> None:  # noqa: ANN001 (msg type optional)
        """Handle an incoming detection array and publish a fused robot pose.

        Args:
            msg: ``apriltag_msgs/AprilTagDetectionArray`` with the ids of the
                tags currently visible. The metric pose of each is fetched via
                TF.
        """
        if not msg.detections:
            return

        estimates: List[RobotEstimate] = []
        for det in msg.detections:
            tag_id = int(det.id)
            tag = TAG_WORLD_POSITIONS.get(tag_id)
            if tag is None:
                self.get_logger().warn(
                    f"Detected unknown tag id {tag_id}; ignoring."
                )
                continue
            est = self._estimate_from_tag(tag, msg.header.stamp)
            if est is not None:
                estimates.append(est)

        if not estimates:
            return

        fused = self._fuse_estimates(estimates)
        self._publish_pose(fused, msg.header.stamp)

    # ------------------------------------------------------------------ #
    # Per-tag geometry
    # ------------------------------------------------------------------ #
    def _estimate_from_tag(
        self, tag: TagPose, stamp
    ) -> Optional[RobotEstimate]:
        """Compute an absolute robot pose from one detected tag via TF.

        GEOMETRY
        --------
        apriltag_node broadcasts the transform ``camera_optical_frame -> tagXX``
        (the tag expressed in the camera). We instead ask TF for the transform
        ``base_link -> tag`` (TF composes the camera mount transform for us),
        which gives the tag position ``(tx, ty)`` in the robot body frame and
        the relative orientation of the tag.

        From ``(tx, ty)`` we get the range and bearing of the tag *as seen by
        the robot*::

            r    = hypot(tx, ty)                 # how far the tag is
            beta = atan2(ty, tx)                 # bearing in the robot frame

        We know the tag's absolute world position ``(Wx, Wy)`` and the world
        yaw of its outward normal ``n``. The tag faces the robot, so the world
        bearing *from the tag back toward the robot* is approximately the tag's
        outward normal direction ``n`` (the camera must be roughly in front of
        the tag to read it). The robot's absolute yaw is then recovered from the
        relationship between the world direction to the tag and the in-body
        bearing::

            yaw_robot = (n + pi) - beta

        because the direction from the robot to the tag in the world is
        ``n + pi`` (opposite the tag normal), and that same direction is
        ``yaw_robot + beta`` in body terms.

        The robot world position is the tag world position minus the
        robot->tag vector rotated into the world frame::

            x_robot = Wx - r * cos(yaw_robot + beta)
            y_robot = Wy - r * sin(yaw_robot + beta)

        ASSUMPTION: planar arena (z, roll, pitch ignored). ASSUMPTION: the
        camera is mounted looking forward with a known static transform to
        ``base_link`` (published on /tf, e.g. by robot_state_publisher); TF
        composes it so we never hardcode the mount here. ASSUMPTION: the tag's
        in-body yaw is dominated by its known wall normal, so we derive robot
        yaw from the wall normal rather than the (noisier at distance) measured
        tag orientation.

        Args:
            tag: Surveyed world pose of the detected tag.
            stamp: Detection timestamp (for the TF lookup).

        Returns:
            A :class:`RobotEstimate`, or ``None`` if the TF was unavailable or
            the detection is implausible (beyond ``max_range_m``).
        """
        tag_frame = f"{self._tag_frame_prefix}{tag.id}"
        try:
            # We want where the tag is relative to the robot body: lookup
            # base_link -> tag. (transform.translation is the tag in base_link.)
            tf = self._tf_buffer.lookup_transform(
                self._base_frame,
                tag_frame,
                Time.from_msg(stamp),
                timeout=Duration(seconds=self._tf_timeout),
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException, tf2_ros.TransformException) as exc:
            self.get_logger().debug(
                f"TF lookup {self._base_frame}->{tag_frame} failed: {exc}"
            )
            return None

        tx = tf.transform.translation.x
        ty = tf.transform.translation.y

        range_m = math.hypot(tx, ty)
        if range_m < 1.0e-3 or range_m > self._max_range:
            return None

        beta = math.atan2(ty, tx)  # bearing of tag in robot body frame

        # Direction (in world) from the robot toward the tag is opposite the
        # tag's outward face normal.
        world_dir_robot_to_tag = normalize_angle(tag.yaw_normal + math.pi)

        # world_dir_robot_to_tag = yaw_robot + beta  ->  yaw_robot = dir - beta
        yaw_robot = normalize_angle(world_dir_robot_to_tag - beta)

        # Robot world position = tag world position - robot->tag world vector.
        x_robot = tag.x - range_m * math.cos(world_dir_robot_to_tag)
        y_robot = tag.y - range_m * math.sin(world_dir_robot_to_tag)

        return RobotEstimate(
            x=x_robot,
            y=y_robot,
            yaw=yaw_robot,
            lin_var=self._linear_variance(range_m),
            yaw_var=self._yaw_variance(range_m),
            range_m=range_m,
        )

    # ------------------------------------------------------------------ #
    # Fusion across multiple tags
    # ------------------------------------------------------------------ #
    @staticmethod
    def _fuse_estimates(estimates: List[RobotEstimate]) -> RobotEstimate:
        """Inverse-variance-weighted average of several robot-pose estimates.

        Closer tags have smaller variance and therefore dominate the mean. Yaw
        is averaged on the unit circle to handle wraparound. The fused variance
        is the standard ``1 / sum(1/var_i)`` of independent measurements, so
        seeing more tags legitimately *increases* confidence (redundancy
        benefit -- see module docstring).

        Args:
            estimates: Non-empty list of per-tag estimates.

        Returns:
            A single fused :class:`RobotEstimate`.
        """
        if len(estimates) == 1:
            return estimates[0]

        # Linear (x, y) inverse-variance weighting.
        wsum = 0.0
        x_acc = 0.0
        y_acc = 0.0
        for e in estimates:
            w = 1.0 / e.lin_var
            wsum += w
            x_acc += w * e.x
            y_acc += w * e.y
        fused_x = x_acc / wsum
        fused_y = y_acc / wsum
        fused_lin_var = 1.0 / wsum

        # Yaw inverse-variance weighting on the unit circle.
        ysum = 0.0
        sin_acc = 0.0
        cos_acc = 0.0
        for e in estimates:
            w = 1.0 / e.yaw_var
            ysum += w
            sin_acc += w * math.sin(e.yaw)
            cos_acc += w * math.cos(e.yaw)
        fused_yaw = math.atan2(sin_acc, cos_acc)
        fused_yaw_var = 1.0 / ysum

        nearest_range = min(e.range_m for e in estimates)
        return RobotEstimate(
            x=fused_x,
            y=fused_y,
            yaw=fused_yaw,
            lin_var=fused_lin_var,
            yaw_var=fused_yaw_var,
            range_m=nearest_range,
        )

    # ------------------------------------------------------------------ #
    # Publishing
    # ------------------------------------------------------------------ #
    def _publish_pose(self, est: RobotEstimate, stamp) -> None:
        """Publish a fused estimate as a PoseWithCovarianceStamped.

        Args:
            est: Fused robot-pose estimate.
            stamp: Timestamp to copy onto the outgoing message.
        """
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self._map_frame

        msg.pose.pose.position.x = est.x
        msg.pose.pose.position.y = est.y
        msg.pose.pose.position.z = 0.0
        qx, qy, qz, qw = yaw_to_quaternion(est.yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        # 6x6 row-major covariance: [x, y, z, roll, pitch, yaw].
        cov = [0.0] * 36
        cov[0] = est.lin_var          # x
        cov[7] = est.lin_var          # y
        cov[14] = UNUSED_COV          # z (unused -> huge)
        cov[21] = UNUSED_COV          # roll (unused)
        cov[28] = UNUSED_COV          # pitch (unused)
        cov[35] = est.yaw_var         # yaw
        msg.pose.covariance = cov

        self._pose_pub.publish(msg)


def main(args: Optional[List[str]] = None) -> None:
    """Entry point: spin the EKF bridge node.

    Args:
        args: Optional argv override (mainly for testing).
    """
    rclpy.init(args=args)
    node = EkfBridgeNode()
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
