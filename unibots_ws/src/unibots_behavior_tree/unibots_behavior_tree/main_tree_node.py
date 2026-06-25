#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import py_trees
import time
import math
from nav_msgs.msg import Odometry

from unibots_behavior_tree.behaviors import (
    CheckEndgame, CheckCapacity, BallDetected, 
    NavigateToTarget, CaptureSequence, DepositSequence
)

class UnibotsBehaviorTree(Node):
    def __init__(self):
        super().__init__('behavior_tree_node')
        
        # Declare parameters (These act as defaults if the YAML isn't loaded)
        self.declare_parameter('endgame_time_s', 150.0)
        self.declare_parameter('max_ball_capacity', 6)
        self.declare_parameter('tof_capture_threshold_m', 0.05)
        self.declare_parameter('goal_tolerance_m', 0.15)
        self.declare_parameter('waypoints', [0.0, 0.0, 0.0])
        self.declare_parameter('home_x', 0.0)
        self.declare_parameter('home_y', 1.0)
        self.declare_parameter('home_yaw', 3.14)
        
        # Declare TOPIC parameters explicitly 
        self.declare_parameter('pose_topic', '/odom/filtered')
        self.declare_parameter('target_topic', '/game/target')
        self.declare_parameter('ball_vision_topic', '/vision/detected_balls')
        self.declare_parameter('tof_sensor_topic', '/sensors/tof_distance')
        self.declare_parameter('servo_scoop_topic', '/servos/scoop_cmd')
        self.declare_parameter('servo_deposit_topic', '/servos/deposit_cmd')

        # Init Blackboard
        self.blackboard = py_trees.blackboard.Client(name="TreeRoot")
        self.blackboard.register_key(key="ball_count", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="ball_visible", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="tof_distance", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="current_pose", access=py_trees.common.Access.WRITE)
        
        self.blackboard.set("ball_count", 0)
        self.blackboard.set("ball_visible", False)
        self.blackboard.set("tof_distance", 99.0)
        self.blackboard.set("current_pose", [0.0, 0.0, 0.0])

        self.start_time = time.time()
        
        # Build the tree using the dynamic parameters
        self.root = self.create_behavior_tree()
        self.tree = py_trees.trees.BehaviourTree(self.root)
        
        # Fetch the pose topic from config and subscribe
        pose_topic = self.get_parameter('pose_topic').value
        self.odom_sub = self.create_subscription(Odometry, pose_topic, self.odom_callback, 10)

        # Tick the tree at 10Hz
        self.create_timer(0.1, self.tick_tree)

    def odom_callback(self, msg):
        """Updates the Blackboard with the robot's real position."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self.blackboard.set("current_pose", [x, y, yaw])

    def create_behavior_tree(self):
        """Constructs the Selector and Sequence hierarchy."""
        
        # Retrieve Logic Params
        endgame_time = self.get_parameter('endgame_time_s').value
        max_cap = self.get_parameter('max_ball_capacity').value
        tof_thresh = self.get_parameter('tof_capture_threshold_m').value
        tol = self.get_parameter('goal_tolerance_m').value
        
        home_x = self.get_parameter('home_x').value
        home_y = self.get_parameter('home_y').value
        home_yaw = self.get_parameter('home_yaw').value
        
        wp_flat = self.get_parameter('waypoints').value
        waypoints = [wp_flat[i:i+3] for i in range(0, len(wp_flat), 3)]

        # Retrieve Topic Params
        target_topic = self.get_parameter('target_topic').value
        scoop_topic = self.get_parameter('servo_scoop_topic').value
        deposit_topic = self.get_parameter('servo_deposit_topic').value

        # --- Tree Root ---
        root = py_trees.composites.Selector(name="Main_Strategy", memory=False)

        # 1. ENDGAME SEQUENCE
        endgame_seq = py_trees.composites.Sequence(name="Endgame_Seq", memory=True)
        endgame_seq.add_children([
            CheckEndgame("Is_Endgame", self.start_time, endgame_time),
            NavigateToTarget("Go_Home", self, target_topic, home_x, home_y, home_yaw, tol),
            DepositSequence("Dump_Balls", self, deposit_topic),
            py_trees.behaviours.Running("Sleep") 
        ])

        # 2. FULL CAPACITY SEQUENCE
        full_seq = py_trees.composites.Sequence(name="Dump_Full_Storage", memory=True)
        full_seq.add_children([
            CheckCapacity("Is_Full", max_cap),
            NavigateToTarget("Go_Home_To_Dump", self, target_topic, home_x, home_y, home_yaw, tol),
            DepositSequence("Dump_Balls_Midgame", self, deposit_topic),
            NavigateToTarget("Back_Up", self, target_topic, home_x + 0.5, home_y, home_yaw, tol)
        ])

        # 3. BALL TRACKING SEQUENCE
        track_seq = py_trees.composites.Sequence(name="Capture_Ball", memory=False)
        track_seq.add_children([
            BallDetected("See_Ball?"),
            CaptureSequence("Trigger_Scoop", self, scoop_topic, tof_thresh)
        ])

        # 4. EXPLORATION SEQUENCE
        explore_seq = py_trees.composites.Sequence(name="Explore_Quadrants", memory=True)
        for i, wp in enumerate(waypoints):
            explore_seq.add_child(
                NavigateToTarget(f"Nav_WP_{i}", self, target_topic, wp[0], wp[1], wp[2], tol)
            )

        root.add_children([endgame_seq, full_seq, track_seq, explore_seq])
        return root

    def tick_tree(self):
        self.tree.tick()

def main(args=None):
    rclpy.init(args=args)
    node = UnibotsBehaviorTree()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()