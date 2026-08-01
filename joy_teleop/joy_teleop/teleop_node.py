#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class PS2Teleop(Node):

    def __init__(self):
        super().__init__('ps2_teleop')

        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10)

        self.publisher = self.create_publisher(
            Twist,
            '/diff_cont/cmd_vel_unstamped',
            10)

        # -----------------------------
        # Axis Mapping (Your Controller)
        # -----------------------------
        self.linear_axis = 1          # Left stick Up/Down
        self.angular_axis = 2         # Right stick Left/Right

        self.dpad_horizontal = 6      # Left/Right
        self.dpad_vertical = 7        # Up/Down

        # -----------------------------
        # Speed Limits
        # -----------------------------
        self.linear_scale = 0.5       # m/s
        self.angular_scale = 1.5      # rad/s

        self.min_linear = 0.1
        self.max_linear = 5.0

        self.min_angular = 0.2
        self.max_angular = 5.0

        # Debounce variables
        self.prev_dpad_vertical = 0
        self.prev_dpad_horizontal = 0

        self.get_logger().info("PS2 Teleop Started")

        self.print_speed()

    def print_speed(self):
        self.get_logger().info(
            "\n"
            "=============================\n"
            f" Linear Speed  : {self.linear_scale:.2f} m/s\n"
            f" Angular Speed : {self.angular_scale:.2f} rad/s\n"
            "============================="
        )

    def joy_callback(self, msg):

        # -----------------------------
        # D-Pad Up / Down
        # -----------------------------
        dpad_v = int(msg.axes[self.dpad_vertical])

        if dpad_v != self.prev_dpad_vertical:

            if dpad_v == 1:

                self.linear_scale *= 1.10
                self.linear_scale = min(self.linear_scale,
                                        self.max_linear)

                self.print_speed()

            elif dpad_v == -1:

                self.linear_scale *= 0.90
                self.linear_scale = max(self.linear_scale,
                                        self.min_linear)

                self.print_speed()

        self.prev_dpad_vertical = dpad_v

        # -----------------------------
        # D-Pad Left / Right
        # -----------------------------
        dpad_h = int(msg.axes[self.dpad_horizontal])

        if dpad_h != self.prev_dpad_horizontal:

            if dpad_h == -1:

                self.angular_scale *= 1.10
                self.angular_scale = min(self.angular_scale,
                                         self.max_angular)

                self.print_speed()

            elif dpad_h == 1:

                self.angular_scale *= 0.90
                self.angular_scale = max(self.angular_scale,
                                         self.min_angular)

                self.print_speed()

        self.prev_dpad_horizontal = dpad_h

        # -----------------------------
        # Publish Twist
        # -----------------------------
        twist = Twist()

        # Forward joystick gives negative value
        twist.linear.x = msg.axes[self.linear_axis] * self.linear_scale

        # Right joystick may also need inversion
        twist.angular.z = msg.axes[self.angular_axis] * self.angular_scale

        self.publisher.publish(twist)


def main(args=None):

    rclpy.init(args=args)

    node = PS2Teleop()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()