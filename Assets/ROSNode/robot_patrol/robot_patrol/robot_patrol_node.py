#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import pandas as pd
import random
import time
import os
import threading
import math


class RobotPatrolNode(Node):
    def __init__(self):
        super().__init__('robot_patrol_node')

        self.nav_action = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.override = False
        self.goal_handle = None
        self.current_pose = None

        # Load patrol data
        self.cell_weights, self.cell_poses = self.load_csv_grid()

        # Subscribers
        self.create_subscription(Bool, '/unity_override', self.override_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Start patrol thread
        self.patrol_thread = threading.Thread(target=self.patrol_loop)
        self.patrol_thread.start()

    def override_callback(self, msg):
        self.override = msg.data
        self.get_logger().info(f"Unity override: {self.override}")
        if self.override and self.goal_handle:
            self.get_logger().info("Cancelling current goal due to override...")
            self.goal_handle.cancel_goal_async()

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def is_far_enough(self, goal_pose, min_distance=0.5):
        if self.current_pose is None:
            return True
        dx = goal_pose.pose.position.x - self.current_pose.position.x
        dy = goal_pose.pose.position.y - self.current_pose.position.y
        return math.sqrt(dx ** 2 + dy ** 2) >= min_distance

    def is_in_map_bounds(self, pose, x_min=0.0, x_max=5.0, y_min=0.0, y_max=5.0):
        x = pose.pose.position.x
        y = pose.pose.position.y
        return x_min <= x <= x_max and y_min <= y <= y_max

    def load_csv_grid(self):
        package_path = os.path.dirname(os.path.realpath(__file__))
        csv_path = os.path.join(package_path, 'data.csv')

        try:
            df = pd.read_csv(csv_path)
        except Exception as e:
            self.get_logger().warn(f"Could not load CSV: {e}")
            df = pd.DataFrame()

        if df.empty or not {'posx', 'posy', 'time'}.issubset(df.columns):
            self.get_logger().warn("CSV empty or missing columns. Falling back to uniform patrol.")
            return self.generate_uniform_patrol()

        df['cell_x'] = (df['posx'] // 1).astype(int)
        df['cell_y'] = (df['posy'] // 1).astype(int)

        grouped = df.groupby(['cell_x', 'cell_y'])
        now = time.time()
        alpha, beta = 1.0, 3.0

        weights, poses = [], []
        for (cx, cy), group in grouped:
            count = len(group)
            latest = group['time'].max()
            score = alpha * count + beta / (now - latest + 1)

            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = cx + 0.5
            pose.pose.position.y = cy + 0.5
            pose.pose.orientation.w = 1.0

            weights.append(score)
            poses.append(pose)

        total = sum(weights)
        weights = [w / total for w in weights]
        self.get_logger().info(f"Loaded {len(poses)} patrol zones from heatmap.")
        return weights, poses

    def generate_uniform_patrol(self, x_min=0, x_max=5, y_min=0, y_max=5, spacing=1.0):
        poses = []
        for x in range(int(x_min), int(x_max), int(spacing)):
            for y in range(int(y_min), int(y_max), int(spacing)):
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = x + 0.5
                pose.pose.position.y = y + 0.5
                pose.pose.orientation.w = 1.0
                poses.append(pose)

        weights = [1.0 / len(poses)] * len(poses)
        self.get_logger().info(f"Generated {len(poses)} uniform patrol points.")
        return weights, poses

    def patrol_loop(self):
        rclpy.spin_until_future_complete(self, self.nav_action.wait_for_server_async())
        self.get_logger().info("Nav2 action server is ready.")

        while rclpy.ok():
            if self.override:
                time.sleep(0.5)
                continue

            pose = random.choices(self.cell_poses, weights=self.cell_weights, k=1)[0]

            if not self.is_far_enough(pose):
                self.get_logger().info("Skipping goal: too close to current position.")
                time.sleep(1.0)
                continue

            if not self.is_in_map_bounds(pose):
                self.get_logger().info("Skipping goal: outside map bounds.")
                time.sleep(1.0)
                continue

            pose.header.stamp = self.get_clock().now().to_msg()
            goal = NavigateToPose.Goal()
            goal.pose = pose

            self.get_logger().info(f"Sending patrol goal: ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})")
            send_goal_future = self.nav_action.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, send_goal_future)

            self.goal_handle = send_goal_future.result()
            if not self.goal_handle.accepted:
                self.get_logger().warn("Goal was rejected by Nav2.")
                self.goal_handle = None
                time.sleep(1.0)
                continue

            self.get_logger().info("Goal accepted. Waiting for it to complete...")

            result_future = self.goal_handle.get_result_async()
            start_time = time.time()
            max_wait_time = 60.0  # seconds

            while not result_future.done():
                if self.override:
                    self.get_logger().info("Override active. Cancelling goal.")
                    self.goal_handle.cancel_goal_async()
                    break

                if time.time() - start_time > max_wait_time:
                    self.get_logger().warn("Goal timeout exceeded. Cancelling...")
                    self.goal_handle.cancel_goal_async()
                    break

                time.sleep(0.5)

            if result_future.done():
                result = result_future.result()
                status = result.result.status
                self.get_logger().info(f"Goal finished with status: {status}")
            else:
                self.get_logger().warn("Goal did not return a result.")

            self.goal_handle = None
            time.sleep(1.0)  # short delay before next patrol


def main(args=None):
    rclpy.init(args=args)
    node = RobotPatrolNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

