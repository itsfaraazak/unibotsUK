import time
import math
import py_trees
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool

class SpinBehavior(py_trees.behaviour.Behaviour):
    """Action: Spin 360 degrees."""
    def __init__(self, name, node, spin_topic="/cmd_vel"):
        super().__init__(name)
        self.node = node
        self.cmd_pub = self.node.create_publisher(Twist, spin_topic, 10)
        self.start_time = None

    def initialise(self):
        self.start_time = time.time()

    def update(self):
        msg = Twist()
        msg.angular.z = 1.0 # Adjust speed as needed
        self.cmd_pub.publish(msg)
        
        # Check if 360 spin is done
        if time.time() - self.start_time > 4.0:
            # STOP the robot before returning SUCCESS
            msg.angular.z = 0.0
            self.cmd_pub.publish(msg)
            return py_trees.common.Status.SUCCESS
            
        return py_trees.common.Status.RUNNING

class SelectBallTarget(py_trees.behaviour.Behaviour):
    """Action: Priority algorithm to pick the best ball."""
    def __init__(self, name, node):
        super().__init__(name)
        self.node = node
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(key="detected_balls", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="target_pose", access=py_trees.common.Access.WRITE)

    def update(self):
        balls = self.blackboard.get("detected_balls")
        if not balls:
            return py_trees.common.Status.FAILURE
        
        best_ball = min(balls, key=lambda b: b['dist'])
        self.blackboard.set("target_pose", best_ball['pose'])
        return py_trees.common.Status.SUCCESS

class CheckEndgame(py_trees.behaviour.Behaviour):
    """Condition: Returns SUCCESS if elapsed time is greater than endgame_time_s."""
    def __init__(self, name, endgame_time):
        super().__init__(name)
        self.endgame_time = endgame_time
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(key="start_time", access=py_trees.common.Access.READ)

    def update(self):
        start_time = self.blackboard.get("start_time")
        if start_time is None:
            return py_trees.common.Status.FAILURE
            
        elapsed = time.time() - start_time
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
    def __init__(self, name, node, target_topic, target_x=None, target_y=None, target_yaw=None, tolerance=0.15):
        super().__init__(name)
        self.node = node
        self.target_x = target_x
        self.target_y = target_y
        self.target_yaw = target_yaw
        self.tolerance = tolerance
        
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(key="current_pose", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="target_pose", access=py_trees.common.Access.READ)
        
        self.target_pub = self.node.create_publisher(PoseStamped, target_topic, 10)

    def update(self):
        if self.target_x is None:
            dynamic_target = self.blackboard.get("target_pose")
            if dynamic_target is None:
                return py_trees.common.Status.FAILURE
            tx, ty, tyaw = dynamic_target
        else:
            tx, ty, tyaw = self.target_x, self.target_y, self.target_yaw

        msg = PoseStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(tx)
        msg.pose.position.y = float(ty)
        msg.pose.orientation.z = math.sin(tyaw / 2.0)
        msg.pose.orientation.w = math.cos(tyaw / 2.0)
        
        self.target_pub.publish(msg)

        current_pose = self.blackboard.get("current_pose")
        if current_pose is None:
            return py_trees.common.Status.RUNNING

        dist = math.hypot(current_pose[0] - tx, current_pose[1] - ty)
        if dist <= self.tolerance:
            self.node.get_logger().info(f"Waypoint Reached: [{tx:.2f}, {ty:.2f}]")
            return py_trees.common.Status.SUCCESS
            
        return py_trees.common.Status.RUNNING

class CaptureSequence(py_trees.behaviour.Behaviour):
    """Action: Reads ToF. If close, triggers scoop servo and increments capacity."""
    def __init__(self, name, node, scoop_topic, tof_threshold):
        super().__init__(name)
        self.node = node
        self.tof_threshold = tof_threshold
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(key="tof_distance", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="ball_count", access=py_trees.common.Access.WRITE)
        self.scoop_pub = self.node.create_publisher(Bool, scoop_topic, 10)
        
        self.triggered = False
        self.action_start_time = None

    def initialise(self):
        self.triggered = False
        self.action_start_time = None

    def update(self):
        if not self.triggered:
            dist = self.blackboard.get("tof_distance")
            if dist is not None and dist <= self.tof_threshold:
                msg = Bool()
                msg.data = True
                self.scoop_pub.publish(msg)
                
                current_count = self.blackboard.get("ball_count")
                if current_count is None: current_count = 0
                self.blackboard.set("ball_count", current_count + 1)
                
                self.node.get_logger().info("Ball Captured!")
                self.triggered = True
                self.action_start_time = time.time()
            else:
                return py_trees.common.Status.RUNNING
                
        # Non-blocking delay (replaces time.sleep)
        if self.triggered and (time.time() - self.action_start_time > 1.0):
            return py_trees.common.Status.SUCCESS
            
        return py_trees.common.Status.RUNNING

class DepositSequence(py_trees.behaviour.Behaviour):
    """Action: Opens dump servo, waits, resets capacity."""
    def __init__(self, name, node, deposit_topic):
        super().__init__(name)
        self.node = node
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(key="ball_count", access=py_trees.common.Access.WRITE)
        self.deposit_pub = self.node.create_publisher(Bool, deposit_topic, 10)
        self.action_start_time = None

    def initialise(self):
        msg = Bool()
        msg.data = True
        self.deposit_pub.publish(msg)
        self.action_start_time = time.time()

    def update(self):
        # Non-blocking delay (replaces time.sleep)
        if time.time() - self.action_start_time > 2.0:
            self.blackboard.set("ball_count", 0)
            self.node.get_logger().info("Deposited Balls. Storage Empty.")
            return py_trees.common.Status.SUCCESS
            
        return py_trees.common.Status.RUNNING