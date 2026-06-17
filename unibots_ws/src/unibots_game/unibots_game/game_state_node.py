"""Top-level match finite-state machine for the Unibots UK 2026 mecanum robot.

This node owns the high-level *match strategy*. It does not perform any of the
low-level control itself; instead it acts as a supervisor that:

  * tracks the global match clock (``MatchTimer``),
  * decides which behavioural state the robot is in (``MatchState``),
  * publishes the *current navigation goal* (``/game/target``) which is consumed
    by the downstream MPC / visual-servo nodes, and
  * publishes discrete servo commands (``/servo/command``).

Division of responsibility
--------------------------
The actual motion is produced elsewhere:

  * ``APPROACH``  -> an iMPC node drives the robot toward ``/game/target``.
  * ``SERVO``     -> a camera-PID node closes the loop on the ball pixel error.
  * ``CAPTURE``   -> a blind dead-reckoning node scoops once the ball enters the
    camera blind spot, confirmed by the IR beam.
  * ``ALIGN_NET`` -> a fine AprilTag alignment node nudges the robot to the net.

This FSM only *selects the mode* (by publishing the goal/servo command) and then
*watches for the transition condition* on the relevant topic. Keeping the
strategy and the control decoupled means each piece can be tuned independently.

Arena frame convention: ``(0, 0)`` is the south-west corner, +x points east,
+y points north, the arena is ~2.0 m square.
"""

from __future__ import annotations

import enum
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.time import Time
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Odometry

from unibots_msgs.msg import BallMap, BallArray

# AprilTag detections are used for the fine net alignment in ALIGN_NET.
# The message package is an external dependency; if it is not available on a
# given machine we degrade gracefully (ALIGN_NET then uses a best-effort
# timeout-based transition). See _handle_align_net() for details.
try:
    from apriltag_msgs.msg import AprilTagDetectionArray

    _APRILTAG_AVAILABLE = True
except ImportError:  # pragma: no cover - depends on environment
    AprilTagDetectionArray = None  # type: ignore[assignment]
    _APRILTAG_AVAILABLE = False


# --------------------------------------------------------------------------- #
# Named constants (no magic numbers)
# --------------------------------------------------------------------------- #

# Match timing (seconds). The official match is 180 s long.
MATCH_DURATION_S = 180.0

# Topic names.
TOPIC_BALL_MAP = "/spatial_memory/ball_map"
TOPIC_VISION_BALLS = "/vision/balls"
TOPIC_BEAM_BROKEN = "/intake/beam_broken"
TOPIC_ODOM = "/odom/filtered"
TOPIC_APRILTAG = "/apriltag/detections"
TOPIC_MATCH_START = "/match/start"
TOPIC_GAME_STATE = "/game/state"
TOPIC_GAME_TARGET = "/game/target"
TOPIC_SERVO_COMMAND = "/servo/command"

# Frame used for all published navigation goals.
MAP_FRAME = "map"

# Servo command strings (contract with the intake/deposit servo node).
SERVO_DEPOSIT = "DEPOSIT"
SERVO_SCOOP_OPEN = "SCOOP_OPEN"
SERVO_SCOOP_CLOSE = "SCOOP_CLOSE"

# Deposit (DUMP) timed-sequence duration in seconds. The DUMP state holds while
# the deposit servo runs its open/dump/close cycle before returning to SEARCH.
DUMP_SEQUENCE_S = 2.5

# SEARCH rotates in place to find balls; this is the yaw rate it requests by
# publishing a goal pose offset from the current heading. The actual rotation is
# executed by the controller; the FSM only nudges the heading goal.
SEARCH_YAW_STEP_RAD = math.radians(30.0)

# Unit conversions.
CM_PER_M = 100.0

# Home-wall goal stand-off: how far in front of the wall the DEPOSIT goal sits.
# (The exact coordinates per zone are documented in _home_wall_goal().)


class MatchState(enum.Enum):
    """High-level match states for the competition FSM."""

    STARTUP = "STARTUP"      # Waiting for the match-start signal.
    SEARCH = "SEARCH"        # Rotating in place to find balls (uses ball_map).
    APPROACH = "APPROACH"    # iMPC drive toward a known ball (Zone 1, far).
    SERVO = "SERVO"          # Camera-PID closing on the ball (Zone 2, near).
    CAPTURE = "CAPTURE"      # Blind dead-reckoning scoop (Zone 3, blind spot).
    DEPOSIT = "DEPOSIT"      # Navigate back to the home wall / net.
    ALIGN_NET = "ALIGN_NET"  # Fine AprilTag alignment in front of the net.
    DUMP = "DUMP"            # Servo deposit sequence (release the balls).
    PARK = "PARK"            # Drive to touch a wall for the end-of-match bonus.
    STOPPED = "STOPPED"      # Match ended; all motion halted.


class MatchTimer:
    """Monotonic match clock. Started once when ``match_start`` is received.

    The timer takes a ROS ``Clock`` (typically ``node.get_clock()``) so that it
    honours simulated time when ``use_sim_time`` is enabled. ``time.time()`` is
    deliberately avoided because it would not track Gazebo sim time.
    """

    # Seconds from match start at which the parking interrupt fires.
    T_PARK = 170.0
    # Seconds from match start at which the hard stop (grace) fires.
    T_STOP = 185.0

    def __init__(self, clock: Clock) -> None:
        """Create a timer bound to ``clock`` (not yet started)."""
        self._clock = clock
        self._start_time: Optional[Time] = None

    def start(self) -> None:
        """Record the match start instant from the bound clock (idempotent)."""
        if self._start_time is None:
            self._start_time = self._clock.now()

    @property
    def started(self) -> bool:
        """True once :meth:`start` has been called."""
        return self._start_time is not None

    def elapsed(self) -> float:
        """Return seconds elapsed since start, or 0.0 if not yet started."""
        if self._start_time is None:
            return 0.0
        delta = self._clock.now() - self._start_time
        return delta.nanoseconds / 1e9

    def is_park_time(self) -> bool:
        """True once the parking interrupt threshold (T_PARK) is reached."""
        return self.started and self.elapsed() >= self.T_PARK

    def is_stop_time(self) -> bool:
        """True once the hard-stop grace threshold (T_STOP) is reached."""
        return self.started and self.elapsed() >= self.T_STOP


class GameStateNode(Node):
    """Supervisory match FSM node.

    Runs a fixed-rate ``tick()`` that publishes the current state, applies the
    global timing interrupts (PARK / STOPPED) and dispatches to a per-state
    handler. Each handler publishes the navigation goal and/or servo command for
    its state and decides whether to transition.
    """

    def __init__(self) -> None:
        super().__init__("game_state_node")

        # ---- Declared parameters (read once at startup) ---- #
        self.home_zone: str = (
            self.declare_parameter("home_zone", "north")
            .get_parameter_value()
            .string_value
        )
        self.capacity: int = (
            self.declare_parameter("capacity", 6)
            .get_parameter_value()
            .integer_value
        )
        self.target_confidence_threshold: float = (
            self.declare_parameter("target_confidence_threshold", 0.65)
            .get_parameter_value()
            .double_value
        )
        self.approach_distance_cm: float = (
            self.declare_parameter("approach_distance_cm", 150.0)
            .get_parameter_value()
            .double_value
        )
        self.servo_blindspot_frac: float = (
            self.declare_parameter("servo_blindspot_frac", 0.88)
            .get_parameter_value()
            .double_value
        )
        self.frame_height_px: int = (
            self.declare_parameter("frame_height_px", 720)
            .get_parameter_value()
            .integer_value
        )
        self.deposit_near_dist: float = (
            self.declare_parameter("deposit_near_dist", 0.5)
            .get_parameter_value()
            .double_value
        )
        self.align_yaw_deg: float = (
            self.declare_parameter("align_yaw_deg", 5.0)
            .get_parameter_value()
            .double_value
        )
        self.align_lateral_cm: float = (
            self.declare_parameter("align_lateral_cm", 2.0)
            .get_parameter_value()
            .double_value
        )
        self.align_dist_cm: float = (
            self.declare_parameter("align_dist_cm", 35.0)
            .get_parameter_value()
            .double_value
        )
        # control_frequency drives the FSM tick. Both aliases are declared so
        # either may be set from a launch file; fsm_frequency takes precedence.
        control_frequency: float = (
            self.declare_parameter("control_frequency", 10.0)
            .get_parameter_value()
            .double_value
        )
        self.fsm_frequency: float = (
            self.declare_parameter("fsm_frequency", control_frequency)
            .get_parameter_value()
            .double_value
        )

        # ---- QoS profiles (explicit) ---- #
        # Sensor-like streams: best-effort, keep-last(=1), volatile.
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        # Reliable control/command streams: reliable, keep-last(=10).
        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        # Latched-style one-shot events (match start): reliable + transient-local
        # so a late subscriber/publisher still sees the start edge.
        latched_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        # ---- State ---- #
        self.state: MatchState = MatchState.STARTUP
        self.balls_held: int = 0
        self.match_timer = MatchTimer(self.get_clock())

        # Latest received messages (None until first arrival).
        self._ball_map: Optional[BallMap] = None
        self._vision_balls: Optional[BallArray] = None
        self._beam_broken: bool = False
        self._odom: Optional[Odometry] = None
        self._apriltags = None  # AprilTagDetectionArray or None

        # Timestamp (sim-time seconds) marking the start of the DUMP sequence.
        self._dump_start_s: Optional[float] = None
        # Current SEARCH heading goal, advanced step-by-step while searching.
        self._search_yaw: float = 0.0

        # ---- Subscriptions ---- #
        self.create_subscription(
            BallMap, TOPIC_BALL_MAP, self._on_ball_map, sensor_qos
        )
        self.create_subscription(
            BallArray, TOPIC_VISION_BALLS, self._on_vision_balls, sensor_qos
        )
        self.create_subscription(
            Bool, TOPIC_BEAM_BROKEN, self._on_beam_broken, sensor_qos
        )
        self.create_subscription(
            Odometry, TOPIC_ODOM, self._on_odom, sensor_qos
        )
        if _APRILTAG_AVAILABLE:
            self.create_subscription(
                AprilTagDetectionArray,
                TOPIC_APRILTAG,
                self._on_apriltags,
                sensor_qos,
            )
        else:
            self.get_logger().warn(
                "apriltag_msgs not available; ALIGN_NET will use a best-effort "
                "timeout-based transition instead of AprilTag feedback."
            )
        self.create_subscription(
            Bool, TOPIC_MATCH_START, self._on_match_start, latched_qos
        )

        # ---- Publishers ---- #
        self._state_pub = self.create_publisher(
            String, TOPIC_GAME_STATE, reliable_qos
        )
        self._target_pub = self.create_publisher(
            PoseStamped, TOPIC_GAME_TARGET, reliable_qos
        )
        self._servo_pub = self.create_publisher(
            String, TOPIC_SERVO_COMMAND, reliable_qos
        )

        # ---- FSM timer ---- #
        period_s = 1.0 / self.fsm_frequency
        self._timer = self.create_timer(period_s, self.tick)

        self.get_logger().info(
            f"GameStateNode up: home_zone={self.home_zone}, "
            f"capacity={self.capacity}, fsm_frequency={self.fsm_frequency} Hz"
        )

    # ------------------------------------------------------------------ #
    # Subscription callbacks (store latest message only)
    # ------------------------------------------------------------------ #

    def _on_ball_map(self, msg: BallMap) -> None:
        self._ball_map = msg

    def _on_vision_balls(self, msg: BallArray) -> None:
        self._vision_balls = msg

    def _on_beam_broken(self, msg: Bool) -> None:
        self._beam_broken = msg.data

    def _on_odom(self, msg: Odometry) -> None:
        self._odom = msg

    def _on_apriltags(self, msg) -> None:
        self._apriltags = msg

    def _on_match_start(self, msg: Bool) -> None:
        """Latch the match-start edge; the actual transition runs in tick()."""
        if msg.data and self.state is MatchState.STARTUP:
            self.match_timer.start()
            self.get_logger().info("Match start received.")
            self.set_state(MatchState.SEARCH)

    # ------------------------------------------------------------------ #
    # Helpers
    # ------------------------------------------------------------------ #

    def set_state(self, new_state: MatchState) -> None:
        """Transition to ``new_state``, logging the change."""
        if new_state is self.state:
            return
        self.get_logger().info(
            f"STATE {self.state.value} -> {new_state.value}"
        )
        self.state = new_state

    def get_robot_pose(self) -> Optional[tuple[float, float, float]]:
        """Return the latest robot ``(x, y, yaw)`` from odometry, or None."""
        if self._odom is None:
            return None
        p = self._odom.pose.pose.position
        yaw = _yaw_from_quaternion(self._odom.pose.pose.orientation)
        return (p.x, p.y, yaw)

    def _home_wall_goal(self) -> tuple[float, float, float]:
        """Return the ``(x, y, yaw)`` deposit goal for the configured home zone.

        The arena is ~2.0 m square with origin at the SW corner. The deposit
        goal sits just in front of the net on the home wall, facing the wall:

          * north => (1.0, 1.85), facing +y  (yaw =  pi/2)
          * east  => (1.85, 1.0), facing +x  (yaw =  0)
          * south => (1.0, 0.15), facing -y  (yaw = -pi/2)
          * west  => (0.15, 1.0), facing -x  (yaw =  pi)
        """
        zone = self.home_zone.lower()
        if zone == "north":
            return (1.0, 1.85, math.pi / 2.0)
        if zone == "east":
            return (1.85, 1.0, 0.0)
        if zone == "south":
            return (1.0, 0.15, -math.pi / 2.0)
        if zone == "west":
            return (0.15, 1.0, math.pi)
        # Unknown zone: default to north and warn.
        self.get_logger().warn(
            f"Unknown home_zone '{self.home_zone}', defaulting to 'north'."
        )
        return (1.0, 1.85, math.pi / 2.0)

    def publish_target(self, x: float, y: float, yaw: float) -> None:
        """Publish a navigation goal pose on /game/target (consumed by MPC)."""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = MAP_FRAME
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0
        msg.pose.orientation = _quaternion_from_yaw(yaw)
        self._target_pub.publish(msg)

    def publish_servo(self, command: str) -> None:
        """Publish a discrete servo command on /servo/command."""
        msg = String()
        msg.data = command
        self._servo_pub.publish(msg)

    def _now_s(self) -> float:
        """Current sim-time clock value in seconds."""
        return self.get_clock().now().nanoseconds / 1e9

    # ------------------------------------------------------------------ #
    # Main tick
    # ------------------------------------------------------------------ #

    def tick(self) -> None:
        """Fixed-rate FSM step: publish state, apply interrupts, dispatch."""
        # Always broadcast the current state for debugging/telemetry.
        state_msg = String()
        state_msg.data = self.state.value
        self._state_pub.publish(state_msg)

        # ---- Global timing interrupts ---- #
        if self.match_timer.started:
            # Hard stop takes precedence over everything.
            if self.match_timer.is_stop_time():
                if self.state is not MatchState.STOPPED:
                    self.set_state(MatchState.STOPPED)
                self._handle_stopped()
                return
            # Parking interrupt: ANY active state -> PARK at T_PARK.
            if self.match_timer.is_park_time() and self.state not in (
                MatchState.PARK,
                MatchState.STOPPED,
            ):
                self.set_state(MatchState.PARK)

        # ---- Per-state dispatch ---- #
        handler = {
            MatchState.STARTUP: self._handle_startup,
            MatchState.SEARCH: self._handle_search,
            MatchState.APPROACH: self._handle_approach,
            MatchState.SERVO: self._handle_servo,
            MatchState.CAPTURE: self._handle_capture,
            MatchState.DEPOSIT: self._handle_deposit,
            MatchState.ALIGN_NET: self._handle_align_net,
            MatchState.DUMP: self._handle_dump,
            MatchState.PARK: self._handle_park,
            MatchState.STOPPED: self._handle_stopped,
        }[self.state]
        handler()

    # ------------------------------------------------------------------ #
    # Per-state handlers
    # ------------------------------------------------------------------ #

    def _handle_startup(self) -> None:
        """Wait for /match/start. Transition handled in _on_match_start()."""
        # Intentionally idle; the robot must not move before the start signal.

    def _handle_search(self) -> None:
        """Rotate in place to find balls; switch to APPROACH on a good target.

        Transition SEARCH -> APPROACH when the spatial-memory ball map exposes a
        ``selected_target`` whose confidence exceeds the threshold. We publish
        that ball's world pose as the nav goal.
        """
        target = self._best_map_target()
        if target is not None:
            tx, ty = target.world_x, target.world_y
            pose = self.get_robot_pose()
            # Face the target if we know where we are; otherwise keep heading.
            yaw = (
                math.atan2(ty - pose[1], tx - pose[0])
                if pose is not None
                else self._search_yaw
            )
            self.publish_target(tx, ty, yaw)
            self.set_state(MatchState.APPROACH)
            return

        # No good target yet: advance the search heading to sweep the arena.
        pose = self.get_robot_pose()
        if pose is not None:
            self._search_yaw = pose[2] + SEARCH_YAW_STEP_RAD
            self.publish_target(pose[0], pose[1], self._search_yaw)

    def _handle_approach(self) -> None:
        """iMPC drives to the target; switch to SERVO when the ball is near.

        Transition APPROACH -> SERVO when the target ball is seen in
        /vision/balls with ``distance_cm`` below the approach threshold.
        The goal pose was already set on entry; we re-publish the map target so
        the controller keeps a fresh goal if the ball moves.
        """
        target = self._best_map_target()
        if target is not None:
            pose = self.get_robot_pose()
            yaw = (
                math.atan2(target.world_y - pose[1], target.world_x - pose[0])
                if pose is not None
                else 0.0
            )
            self.publish_target(target.world_x, target.world_y, yaw)

        nearest = self._nearest_vision_ball()
        if nearest is not None and nearest.distance_cm < self.approach_distance_cm:
            self.set_state(MatchState.SERVO)

    def _handle_servo(self) -> None:
        """Camera-PID node closes the loop; switch to CAPTURE at the blind spot.

        Transition SERVO -> CAPTURE when the ball's ``pixel_y`` crosses below the
        camera blind-spot line (``servo_blindspot_frac * frame_height_px``),
        meaning it is about to leave the field of view into the scoop.

        The visual-servo math lives in a separate camera-PID node; this FSM only
        holds the SERVO mode and watches the pixel position.
        """
        nearest = self._nearest_vision_ball()
        if nearest is None:
            # Lost sight of the ball during servo: fall back to APPROACH.
            self.get_logger().warn("SERVO lost ball; reverting to APPROACH.")
            self.set_state(MatchState.APPROACH)
            return

        blindspot_line = self.servo_blindspot_frac * float(self.frame_height_px)
        if nearest.pixel_y > blindspot_line:
            self.set_state(MatchState.CAPTURE)

    def _handle_capture(self) -> None:
        """Blind dead-reckoning scoop; confirmed by the IR beam.

        Transitions on the IR beam edge:
          * CAPTURE -> APPROACH if balls_held < capacity (got one, keep going),
          * CAPTURE -> DEPOSIT  if balls_held >= capacity (intake full).
        On a successful capture we increment the count and pulse the scoop servo.
        """
        if self._beam_broken:
            self.balls_held += 1
            self.publish_servo(SERVO_SCOOP_CLOSE)
            self.get_logger().info(
                f"Ball captured (balls_held={self.balls_held}/{self.capacity})."
            )
            # Consume the beam edge so we don't double-count on the next tick.
            self._beam_broken = False

            if self.balls_held >= self.capacity:
                self.set_state(MatchState.DEPOSIT)
            else:
                # Re-open scoop for the next ball and resume the hunt.
                self.publish_servo(SERVO_SCOOP_OPEN)
                self.set_state(MatchState.APPROACH)

    def _handle_deposit(self) -> None:
        """Navigate to the home wall; switch to ALIGN_NET when close.

        Publishes the home-wall goal (derived from ``home_zone``) and transitions
        DEPOSIT -> ALIGN_NET once the robot is within ``deposit_near_dist`` of it.
        """
        goal_x, goal_y, goal_yaw = self._home_wall_goal()
        self.publish_target(goal_x, goal_y, goal_yaw)

        pose = self.get_robot_pose()
        if pose is None:
            return
        dist = math.hypot(goal_x - pose[0], goal_y - pose[1])
        if dist < self.deposit_near_dist:
            self.set_state(MatchState.ALIGN_NET)

    def _handle_align_net(self) -> None:
        """Fine AprilTag alignment in front of the net; switch to DUMP on lock.

        Transition ALIGN_NET -> DUMP when the AprilTag pose error is within all
        three tolerances: yaw < align_yaw_deg, lateral < align_lateral_cm and
        range < align_dist_cm.

        AprilTag pose estimation (best-effort): the tag is mounted on the net.
        From the detection pose (tag expressed in the camera/robot frame) we take
          * range    = ||(x, z)|| projected onto the ground plane,
          * lateral  = the sideways offset (x component),
          * yaw      = the tag's heading relative to the camera bore.
        The detailed alignment control lives in a dedicated AprilTag node; here we
        only evaluate whether the published detection already meets tolerance.
        """
        if not _APRILTAG_AVAILABLE or self._apriltags is None:
            # Best-effort fallback: without tag feedback we cannot verify the
            # alignment, so assume the dedicated aligner has converged and hand
            # off to DUMP. Documented limitation when apriltag_msgs is absent.
            self.set_state(MatchState.DUMP)
            return

        est = self._estimate_apriltag_error()
        if est is None:
            return  # No usable detection this tick; keep aligning.

        yaw_deg, lateral_cm, dist_cm = est
        if (
            abs(yaw_deg) < self.align_yaw_deg
            and abs(lateral_cm) < self.align_lateral_cm
            and dist_cm < self.align_dist_cm
        ):
            self.set_state(MatchState.DUMP)

    def _handle_dump(self) -> None:
        """Run the timed deposit sequence, then return to SEARCH.

        On entry we command the deposit servo; after ``DUMP_SEQUENCE_S`` seconds
        the balls have been released, so we reset the count and go back to hunt.
        """
        if self._dump_start_s is None:
            self._dump_start_s = self._now_s()
            self.publish_servo(SERVO_DEPOSIT)
            self.get_logger().info("DUMP: releasing balls.")
            return

        if self._now_s() - self._dump_start_s >= DUMP_SEQUENCE_S:
            self.balls_held = 0
            self._dump_start_s = None
            self.publish_servo(SERVO_SCOOP_CLOSE)
            self.set_state(MatchState.SEARCH)

    def _handle_park(self) -> None:
        """Drive to touch the nearest wall for the parking bonus.

        Reuses the home-wall goal as a guaranteed-reachable wall contact point.
        Transition PARK -> STOPPED happens at T_STOP (handled in tick()) or when
        the robot is judged to be touching the wall.
        """
        goal_x, goal_y, goal_yaw = self._home_wall_goal()
        self.publish_target(goal_x, goal_y, goal_yaw)

        pose = self.get_robot_pose()
        if pose is not None:
            dist = math.hypot(goal_x - pose[0], goal_y - pose[1])
            # "Touching the wall": within the deposit stand-off distance.
            if dist < self.deposit_near_dist:
                self.set_state(MatchState.STOPPED)

    def _handle_stopped(self) -> None:
        """Match over: hold position and silence the servo.

        We publish the current pose as the goal so the controller commands zero
        motion, and request the scoop closed for a tidy end state.
        """
        pose = self.get_robot_pose()
        if pose is not None:
            self.publish_target(pose[0], pose[1], pose[2])
        self.publish_servo(SERVO_SCOOP_CLOSE)

    # ------------------------------------------------------------------ #
    # Perception helpers
    # ------------------------------------------------------------------ #

    def _best_map_target(self):
        """Return the ball-map target above the confidence threshold, or None.

        Prefers the map's ``selected_target`` when its confidence is high enough;
        otherwise scans all mapped balls for the highest-confidence candidate.
        """
        if self._ball_map is None:
            return None

        sel = self._ball_map.selected_target
        if sel is not None and sel.confidence > self.target_confidence_threshold:
            return sel

        best = None
        for ball in self._ball_map.balls:
            if ball.confidence <= self.target_confidence_threshold:
                continue
            if best is None or ball.confidence > best.confidence:
                best = ball
        return best

    def _nearest_vision_ball(self):
        """Return the closest ball currently seen in /vision/balls, or None."""
        if self._vision_balls is None or not self._vision_balls.balls:
            return None
        return min(self._vision_balls.balls, key=lambda b: b.distance_cm)

    def _estimate_apriltag_error(self) -> Optional[tuple[float, float, float]]:
        """Best-effort ``(yaw_deg, lateral_cm, dist_cm)`` from the first tag.

        ``apriltag_msgs/AprilTagDetection`` does not by itself carry a metric 3D
        pose (that depends on the pipeline); fields vary by build. We read the
        most common layout defensively and fall back to the pixel ``centre`` when
        no pose is present, returning None if nothing usable is found. The
        dedicated AprilTag aligner does the real estimation; this is only a guard
        so the FSM does not advance prematurely.
        """
        if self._apriltags is None or not self._apriltags.detections:
            return None

        det = self._apriltags.detections[0]

        # Preferred: a metric pose attached to the detection (pipeline-dependent).
        pose = getattr(det, "pose", None)
        if pose is not None:
            try:
                # pose may be a PoseWithCovarianceStamped or Pose; dig for x,y,z.
                pos = (
                    pose.pose.pose.position
                    if hasattr(pose, "pose")
                    else pose.position
                )
                lateral_cm = pos.x * CM_PER_M
                dist_cm = math.hypot(pos.x, pos.z) * CM_PER_M
                quat = (
                    pose.pose.pose.orientation
                    if hasattr(pose, "pose")
                    else pose.orientation
                )
                yaw_deg = math.degrees(_yaw_from_quaternion(quat))
                return (yaw_deg, lateral_cm, dist_cm)
            except AttributeError:
                pass  # Fall through to the pixel-centre fallback.

        # Fallback: use the pixel centre offset only as a coarse lateral proxy.
        # We cannot recover a true range/yaw from pixels alone, so we report a
        # large distance to keep the FSM in ALIGN_NET until real pose data
        # arrives (or the dedicated aligner converges).
        return None


# --------------------------------------------------------------------------- #
# Math utilities
# --------------------------------------------------------------------------- #


def _yaw_from_quaternion(q: Quaternion) -> float:
    """Extract the yaw (rotation about +z) from a quaternion, in radians."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _quaternion_from_yaw(yaw: float) -> Quaternion:
    """Build a quaternion representing a pure yaw rotation about +z."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def main(args: Optional[list[str]] = None) -> None:
    """Entry point: initialise rclpy, spin the GameStateNode, clean up."""
    rclpy.init(args=args)
    node = GameStateNode()
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
