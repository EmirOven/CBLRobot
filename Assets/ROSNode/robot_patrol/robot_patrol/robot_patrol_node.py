import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from collections import defaultdict
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import os
import csv
import time
import math
import random

class RobotPatrolNode(Node):
    def __init__(self):
        super().__init__('robot_patrol_node')

        self.override = False
        self.goal_active = False
        self.current_pose = None
        self.current_goal_handle = None

        # ROS 2 Action Client for Nav2
        self.nav_action = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # QoS profile to ensure reliable communication with Unity
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.create_subscription(Bool, '/unity_override', self.override_callback, qos_profile)
        self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile)

        # Load patrol grid from CSV or fall back
        self.cell_weights, self.cell_poses = self.load_csv_grid()

        self.get_logger().info("Waiting for NavigateToPose action server...")
        self.nav_action.wait_for_server()
        self.get_logger().info("Action server ready.")

        # Start patrol logic as a repeating timer
        self.create_timer(2.0, self.patrol_tick)

    def override_callback(self, msg):
        self.override = msg.data
        self.get_logger().info(f"[Unity Override] Received: {self.override}")

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def patrol_tick(self):
        if self.override or self.goal_active or not self.cell_poses:
            return

        pose = random.choices(self.cell_poses, weights=self.cell_weights, k=1)[0]

        if not self.is_far_enough(pose) or not self.is_in_map_bounds(pose):
            return

        self.send_goal(pose)

    def send_goal(self, pose):
        pose.header.stamp = self.get_clock().now().to_msg()
        goal = NavigateToPose.Goal()
        goal.pose = pose

        self.get_logger().info(f"[Patrol] Sending goal: ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})")
        future = self.nav_action.send_goal_async(goal, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)
        self.goal_active = True

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("[Nav2] Goal was rejected.")
            self.goal_active = False
            return

        self.get_logger().info("[Nav2] Goal accepted.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)
        self.current_goal_handle = goal_handle

    def result_callback(self, future):
        result = future.result().result
        status = future.result().status

        status_map = {1: "SUCCEEDED", 2: "CANCELED", 3: "ABORTED"}
        status_str = status_map.get(status, f"UNKNOWN({status})")
        self.get_logger().info(f"[Nav2] Goal completed with status: {status_str}")

        self.goal_active = False
        self.current_goal_handle = None

    def feedback_callback(self, feedback_msg):
        # Optional: Handle feedback here
        pass

    def is_far_enough(self, pose, min_distance=0.5):
        if self.current_pose is None:
            return True
        dx = pose.pose.position.x - self.current_pose.position.x
        dy = pose.pose.position.y - self.current_pose.position.y
        return math.sqrt(dx**2 + dy**2) >= min_distance

    def is_in_map_bounds(self, pose, x_min=0.0, x_max=5.0, y_min=0.0, y_max=5.0):
        x = pose.pose.position.x
        y = pose.pose.position.y
        return x_min <= x <= x_max and y_min <= y <= y_max

    def load_csv_grid(self):
        csv_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'data.csv')

        if not os.path.exists(csv_path):
            self.get_logger().warn("CSV not found. Falling back to uniform patrol.")
            return self.generate_uniform_patrol()

        try:
            with open(csv_path, 'r') as f:
                reader = csv.DictReader(f)
                rows = list(reader)
        except Exception as e:
            self.get_logger().warn(f"CSV read error: {e}")
            return self.generate_uniform_patrol()

        if not rows or not {'posx', 'posy', 'time'}.issubset(rows[0].keys()):
            self.get_logger().warn("CSV missing required columns. Using fallback.")
            return self.generate_uniform_patrol()

        cell_data = defaultdict(list)
        for row in rows:
            try:
                x = float(row['posx'])
                y = float(row['posy'])
                t = float(row['time'])
                cell_data[(int(x), int(y))].append(t)
            except Exception:
                continue

        now = time.time()
        alpha, beta = 1.0, 3.0
        weights, poses = [], []

        for (cx, cy), timestamps in cell_data.items():
            score = alpha * len(timestamps) + beta / (now - max(timestamps) + 1)
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = cx + 0.5
            pose.pose.position.y = cy + 0.5
            pose.pose.orientation.w = 1.0
            weights.append(score)
            poses.append(pose)

        total = sum(weights)
        if total == 0:
            return self.generate_uniform_patrol()

        weights = [w / total for w in weights]
        self.get_logger().info(f"Loaded {len(poses)} patrol zones from CSV.")
        return weights, poses

    def generate_uniform_patrol(self, x_min=0.0, x_max=5.0, y_min=0.0, y_max=5.0, spacing=1.0):
        poses = []
        x = x_min
        while x < x_max:
            y = y_min
            while y < y_max:
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = x + spacing / 2
                pose.pose.position.y = y + spacing / 2
                pose.pose.orientation.w = 1.0
                poses.append(pose)
                y += spacing
            x += spacing
        weights = [1.0 / len(poses)] * len(poses)
        self.get_logger().info(f"Generated {len(poses)} uniform patrol points.")
        return weights, poses

def main(args=None):
    rclpy.init(args=args)
    node = RobotPatrolNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

