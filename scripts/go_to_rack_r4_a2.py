#!/usr/bin/env python3
"""
Navigate to Rack R4 A2, trigger scanner detection via /bot_status,
wait for completion, stop camera system, and exit cleanly.
"""

import math
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import tf_transformations


def create_pose_stamped(navigator, position_x, position_y, rotation_z):
    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, rotation_z)
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = float(position_x)
    goal_pose.pose.position.y = float(position_y)
    goal_pose.pose.position.z = 0.0
    goal_pose.pose.orientation.x = q_x
    goal_pose.pose.orientation.y = q_y
    goal_pose.pose.orientation.z = q_z
    goal_pose.pose.orientation.w = q_w
    return goal_pose


class SingleRackNavigatorNode(Node):
    def __init__(self):
        super().__init__('go_to_rack_r4_a2_node')
        self.bot_status_pub = self.create_publisher(String, '/bot_status', 10)
        self.bot_status_sub = self.create_subscription(
            String, '/bot_status', self.bot_status_callback, 10
        )
        self.camera_system_pub = self.create_publisher(String, '/camera_system', 10)
        self.scan_completed = False

    def bot_status_callback(self, msg: String):
        data = msg.data.strip()
        self.get_logger().info(f"[/bot_status Subscriber] Received: '{data}'")

        if data.lower().startswith("succeeded"):
            self.get_logger().info(f"Scan complete confirmation received: '{data}'")
            self.scan_completed = True

    def publish_reached(self, location_str):
        self.scan_completed = False
        msg = String()
        msg.data = f"Reached {location_str}"
        self.bot_status_pub.publish(msg)
        self.get_logger().info(f"Published to /bot_status: '{msg.data}'")

    def publish_stop_camera(self):
        msg = String()
        msg.data = "stop"
        self.camera_system_pub.publish(msg)
        self.get_logger().info("Published 'stop' to /camera_system to stop camera scanner node.")


def main():
    rclpy.init()

    node = SingleRackNavigatorNode()
    nav = BasicNavigator()

    location_name = "rack_r4_a2"
    location_str = "Aisle_2/Row_4/Rack_1"
    gx, gy, gyaw = 3.658, -2.684, 0.044

    node.get_logger().info(f"Waiting for Nav2 to activate...")
    nav.waitUntilNav2Active()

    try:
        node.get_logger().info(f"Navigating to {location_name} ({location_str}) at (x={gx}, y={gy}, yaw={gyaw:.2f})...")
        goal_pose = create_pose_stamped(nav, gx, gy, gyaw)
        nav.goToPose(goal_pose)

        while not nav.isTaskComplete():
            rclpy.spin_once(node, timeout_sec=0.1)

        result = nav.getResult()
        if result == TaskResult.SUCCEEDED:
            node.get_logger().info(f"Successfully reached {location_name}!")
        else:
            node.get_logger().error(f"Navigation to {location_name} failed! Result: {result}")

        # Trigger scanning
        node.get_logger().info(f"Stopping at {location_name} and triggering scanner via /bot_status...")
        node.publish_reached(location_str)

        node.get_logger().info("Waiting for camera scanner completion (Succeeded)...")
        while not node.scan_completed and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(0.05)

        node.get_logger().info(f"Scanner finished for {location_name}!")

        # Stop camera system
        node.publish_stop_camera()
        time.sleep(0.5)

    except KeyboardInterrupt:
        node.get_logger().info("Operation interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
