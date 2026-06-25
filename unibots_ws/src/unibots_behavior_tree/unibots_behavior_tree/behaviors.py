import time
import math
import py_trees
import rclpy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

class CheckEndgame(py_trees.behaviour.Behaviour):
    """Condition: Returns SUCCESS if elapsed time is greater than endgame_time_s."""
    def __init__(self, name, start_time, endgame_time):
        super().__init__(name)
        self.start_time = start_time
        self.endgame_time = endgame_time

    def update(self):
        elapsed = time.time() - self.start_time
        if elapsed >= self.endgame_time:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

class CheckCapacity(py_trees.behaviour.Behaviour):
    """Condition: Returns SUCCESS if the robot is full."""
    def __init__(self, name, max_capacity):
        super().__init__(name)
        self.max_capacity = max_capacity
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(key="ball_count", access=py_trees.common.Access.READ)

    def update(self):
        count = self.blackboard.get("ball_count")
        if count is not None and count >= self.max_capacity:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

class BallDetected(py_trees.behaviour.Behaviour):
    """Condition: Returns SUCCESS if YOLO sees a ball."""
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(key="ball_visible", access=py_trees.common.Access.READ)

    def update(self):
        if self.blackboard.get("ball_visible"):
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

class NavigateToTarget(py_trees.behaviour.Behaviour):
    """Action: Publishes a goal to mpc_controller_node.py and waits until reached."""
    # Added target_topic argument here
    def __init__(self, name, node, target_topic, target_x, target_y, target_yaw, tolerance):
        super().__init__(name)
        self.node = node
        self.target_x = target_x
        self.target_y = target_y
        self.target_yaw = target_yaw
        self.tolerance = tolerance
        
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(key="current_pose", access=py_trees.common.Access.READ)
        
        # Uses the dynamic topic from the config
        self.target_pub = self.node.create_publisher(PoseStamped, target_topic, 10)

    def update(self):
        # 1. ALWAYS publish the goal to keep the MPC node from timing out
        msg = PoseStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = self.target_x
        msg.pose.position.y = self.target_y
        msg.pose.orientation.z = math.sin(self.target_yaw / 2.0)
        msg.pose.orientation.w = math.cos(self.target_yaw / 2.0)
        
        self.target_pub.publish(msg)

        # 2. Check distance to target
        current_pose = self.blackboard.get("current_pose")
        
        if current_pose is None or current_pose == [0.0, 0.0, 0.0]:
            return py_trees.common.Status.RUNNING

        dist = math.hypot(current_pose[0] - self.target_x, current_pose[1] - self.target_y)
        if dist <= self.tolerance:
            self.node.get_logger().info(f"Waypoint Reached: [{self.target_x}, {self.target_y}]")
            return py_trees.common.Status.SUCCESS
            
        return py_trees.common.Status.RUNNING

class CaptureSequence(py_trees.behaviour.Behaviour):
    """Action: Reads ToF. If close, triggers scoop servo and increments capacity."""
    # Added scoop_topic argument here
    def __init__(self, name, node, scoop_topic, tof_threshold):
        super().__init__(name)
        self.node = node
        self.tof_threshold = tof_threshold
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(key="tof_distance", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="ball_count", access=py_trees.common.Access.WRITE)
        
        # Uses the dynamic topic from the config
        self.scoop_pub = self.node.create_publisher(Bool, scoop_topic, 10)

    def update(self):
        dist = self.blackboard.get("tof_distance")
        if dist is not None and dist <= self.tof_threshold:
            msg = Bool()
            msg.data = True
            self.scoop_pub.publish(msg)
            
            current_count = self.blackboard.get("ball_count")
            self.blackboard.set("ball_count", current_count + 1)
            
            self.node.get_logger().info("Ball Captured!")
            time.sleep(1.0)
            return py_trees.common.Status.SUCCESS
            
        return py_trees.common.Status.RUNNING

class DepositSequence(py_trees.behaviour.Behaviour):
    """Action: Opens dump servo, waits, resets capacity."""
    # Added deposit_topic argument here
    def __init__(self, name, node, deposit_topic):
        super().__init__(name)
        self.node = node
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(key="ball_count", access=py_trees.common.Access.WRITE)
        
        # Uses the dynamic topic from the config
        self.deposit_pub = self.node.create_publisher(Bool, deposit_topic, 10)

    def update(self):
        msg = Bool()
        msg.data = True
        self.deposit_pub.publish(msg)
        time.sleep(2.0)
        
        self.blackboard.set("ball_count", 0)
        self.node.get_logger().info("Deposited Balls. Storage Empty.")
        return py_trees.common.Status.SUCCESS