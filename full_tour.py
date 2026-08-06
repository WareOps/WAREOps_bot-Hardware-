#!/usr/bin/env python3
"""
Full Warehouse Tour Navigation & Active Scanner Coordination Node.

Flow:
 1. Navigates sequentially to each target rack using Nav2 goToPose.
 2. Upon arrival at a goal, stops and publishes "Reached <location>" to /bot_status.
 3. Waits for the rack scanner node to perform scanning and publish "Succeeded <location>".
 4. Proceeds to the next goal in a loop.
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


class TourManagerNode(Node):
    def __init__(self):
        super().__init__('tour_manager_node')
        self.bot_status_pub = self.create_publisher(String, '/bot_status', 10)
        self.bot_status_sub = self.create_subscription(
            String, '/bot_status', self.bot_status_callback, 10
        )
        self.camera_system_pub = self.create_publisher(String, '/camera_system', 10)
        self.scan_completed = False
        self.current_target_location = ""

    def bot_status_callback(self, msg: String):
        data = msg.data.strip()
        self.get_logger().info(f"[/bot_status Subscriber] Received: '{data}'")

        # Check if scanner published "Succeeded <location>"
        if data.lower().startswith("succeeded"):
            self.get_logger().info(f"Scan complete confirmation received for target: '{data}'")
            self.scan_completed = True

    def publish_reached(self, location_str):
        self.scan_completed = False
        self.current_target_location = location_str
        msg = String()
        msg.data = f"Reached {location_str}"
        self.bot_status_pub.publish(msg)
        self.get_logger().info(f"Published to /bot_status: '{msg.data}'")

    def publish_stop_camera(self):
        msg = String()
        msg.data = "stop"
        self.camera_system_pub.publish(msg)
        self.get_logger().info("Published 'stop' to /camera_system to shut down camera scanner node.")


def main():
    rclpy.init()

    tour_node = TourManagerNode()
    nav = BasicNavigator()

    # Ensure Nav2 active
    tour_node.get_logger().info("Waiting for Nav2 to activate...")
    nav.waitUntilNav2Active()
    tour_node.get_logger().info("Nav2 is active! Starting warehouse tour loop...")

    # Inspection Goals (Target Racks + Intermediate Waypoints + Origin)
    goals = [
        {
            "name": "intermediate_point_1",
            "location": "WayPoint_1",
            "x": 0.613,
            "y": -2.945,
            "yaw": 0.067,
            "scan": False
        },
        {
            "name": "rack_r4_a3",
            "location": "Aisle_3/Row_4/Rack_1",
            "x": 1.922,
            "y": -2.745,
            "yaw": 0.051,
            "scan": True
        },

        {
            "name": "rack_r4_a2",
            "location": "Aisle_2/Row_4/Rack_1",
            "x": 3.658,
            "y": -2.684,
            "yaw": 0.044,
            "scan": True
        },

        {
            "name": "intermediate_point_2",
            "location": "WayPoint_2",
            "x": 7.692,
            "y": -2.926,
            "yaw": 3.139,
            "scan": False
        },

        {
            "name": "rack_r3_b3",
            "location": "Aisle_B3/Row_3/Rack_1",
            "x": 2.981,
            "y": -2.825,
            "yaw": 3.097,
            "scan": True
        },
        {
            "name": "Origin / Home",
            "location": "Origin",
            "x": 0.0,
            "y": 0.0,
            "yaw": math.pi,
            "scan": False
        }
    ]

    tour_count = 1

    try:
        tour_node.get_logger().info("========== STARTING SINGLE WAREHOUSE TOUR ==========")

        for target in goals:
            loc_name = target["name"]
            loc_str = target["location"]
            gx, gy, gyaw = target["x"], target["y"], target["yaw"]
            do_scan = target["scan"]

            tour_node.get_logger().info(
                f"Navigating to {loc_name} ({loc_str}) at (x={gx}, y={gy}, yaw={gyaw:.2f})..."
            )

            goal_pose = create_pose_stamped(nav, gx, gy, gyaw)
            nav.goToPose(goal_pose)

            # Navigation loop with spin_once to service ROS2 callbacks
            while not nav.isTaskComplete():
                rclpy.spin_once(tour_node, timeout_sec=0.1)

            result = nav.getResult()
            if result == TaskResult.SUCCEEDED:
                tour_node.get_logger().info(f"Successfully reached {loc_name}!")
            else:
                tour_node.get_logger().error(f"Navigation to {loc_name} failed or was canceled! Result: {result}")

            # If this location requires scanning
            if do_scan:
                tour_node.get_logger().info(
                    f"Stopping at {loc_name} and triggering scanner via /bot_status..."
                )
                tour_node.publish_reached(loc_str)

                # Wait until scanner completes and publishes "Succeeded"
                tour_node.get_logger().info("Waiting for camera scanner completion (Succeeded)...")
                while not tour_node.scan_completed and rclpy.ok():
                    rclpy.spin_once(tour_node, timeout_sec=0.1)
                    time.sleep(0.05)

                tour_node.get_logger().info(f"Scanner finished at {loc_name}! Moving to next destination.")
                time.sleep(1.0)

        tour_node.get_logger().info("========== WAREHOUSE TOUR COMPLETED SUCCESSFULLY ==========")
        tour_node.publish_stop_camera()
        time.sleep(0.5)

    except KeyboardInterrupt:
        tour_node.get_logger().info("Tour interrupted by user.")
    finally:
        tour_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
