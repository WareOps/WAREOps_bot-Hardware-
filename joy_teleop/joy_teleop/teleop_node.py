#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import json
import subprocess


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

        # Publisher: camera servo commands → esp32_wifi_bridge
        self.servo_cmd_pub = self.create_publisher(
            String,
            '/camera_servo_cmd',
            10)

        # -----------------------------
        # Axis Mapping (Your Controller)
        # -----------------------------
        self.linear_axis = 1          # Left stick Up/Down
        self.angular_axis = 2         # Right stick Left/Right

        self.dpad_horizontal = 6      # Left/Right
        self.dpad_vertical = 7        # Up/Down

        # -----------------------------
        # Right-Side Face Button Mapping
        # PS2/Xbox style: A=0, B=1, X=3, Y=4
        # (Adjust indices here if your controller differs)
        # -----------------------------
        self.btn_A = 0   # Pitch DOWN  (+5 degrees, camera tilts down)
        self.btn_B = 1   # Yaw RIGHT   (+5 degrees)
        self.btn_X = 3   # Yaw LEFT    (-5 degrees)
        self.btn_Y = 4   # Pitch UP    (-5 degrees, camera tilts up)

        # -----------------------------
        # Select / Start Button Mapping
        # PS2 style: Select=8, Start=9
        # (Adjust indices here if your controller differs)
        # -----------------------------
        self.btn_SELECT = 10   # Go to PARKED position  (yaw=100°, pitch=90°)
        self.btn_START  = 11   # Go to DEFAULT SCAN pos (yaw=10°,  pitch=90°)

        # -----------------------------
        # Status Shortcut Button Mapping
        # Buttons 6-9 publish preset /bot_status messages
        # -----------------------------
        self.btn_STATUS_6 = 6   # Reached Aisle_2/Row_1/Rack_1
        self.btn_STATUS_7 = 7   # Reached Aisle_1/Row_1/Rack_1
        self.btn_STATUS_8 = 8   # Reached Aisle_2/Row_1/Rack_2
        self.btn_STATUS_9 = 9   # Reached Aisle_1/Row_1/Rack_2

        self.STATUS_MESSAGES = {
            6: "Reached Aisle_2/Row_1/Rack_1",
            7: "Reached Aisle_1/Row_1/Rack_1",
            8: "Reached Aisle_2/Row_1/Rack_2",
            9: "Reached Aisle_1/Row_1/Rack_2",
        }

        # Preset positions — must match rack_scanner_node constants
        self.PARKED_YAW    = 100
        self.PARKED_PITCH  = 90
        self.DEFAULT_YAW   = 10
        self.DEFAULT_PITCH = 90

        # -----------------------------
        # Servo State
        # -----------------------------
        self.servo_yaw   = 100      # Start at parked yaw position
        self.servo_pitch = 90       # Start at parked pitch position

        self.PITCH_MIN = 20         # Servo hardware limit (tilt up)
        self.PITCH_MAX = 160        # Servo hardware limit (tilt down)
        self.YAW_MIN   = 0
        self.YAW_MAX   = 180
        self.SERVO_STEP = 5         # Degrees per button press
        self.SERVO_SPEED = 20       # Speed value forwarded to ESP32

        # Debounce: track previous button states to detect press edges
        self.prev_btn_A      = 0
        self.prev_btn_B      = 0
        self.prev_btn_X      = 0
        self.prev_btn_Y      = 0
        self.prev_btn_SELECT = 0
        self.prev_btn_START  = 0
        self.prev_btn_6      = 0
        self.prev_btn_7      = 0
        self.prev_btn_8      = 0
        self.prev_btn_9      = 0

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
        self.get_logger().info(
            f"Camera servo parked at yaw={self.servo_yaw}°, pitch={self.servo_pitch}° "
            f"| Step={self.SERVO_STEP}° | Pitch limits=[{self.PITCH_MIN}°, {self.PITCH_MAX}°]"
        )

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
        # Face Button Servo Control
        # Y = Pitch Up | A = Pitch Down
        # B = Yaw Right | X = Yaw Left
        # (Each press = SERVO_STEP degrees)
        # -----------------------------
        num_buttons = len(msg.buttons)

        def btn(idx):
            return int(msg.buttons[idx]) if idx < num_buttons else 0

        cur_A = btn(self.btn_A)
        cur_B = btn(self.btn_B)
        cur_X = btn(self.btn_X)
        cur_Y = btn(self.btn_Y)

        servo_changed = False

        # Y pressed (rising edge) → Pitch UP (decrease pitch angle)
        if cur_Y == 1 and self.prev_btn_Y == 0:
            new_pitch = max(self.PITCH_MIN, self.servo_pitch - self.SERVO_STEP)
            if new_pitch != self.servo_pitch:
                self.servo_pitch = new_pitch
                servo_changed = True
                self.get_logger().info(f"[Y] Pitch UP → {self.servo_pitch}°")
            else:
                self.get_logger().warn(f"[Y] Pitch UP blocked — at minimum ({self.PITCH_MIN}°)")

        # A pressed (rising edge) → Pitch DOWN (increase pitch angle)
        if cur_A == 1 and self.prev_btn_A == 0:
            new_pitch = min(self.PITCH_MAX, self.servo_pitch + self.SERVO_STEP)
            if new_pitch != self.servo_pitch:
                self.servo_pitch = new_pitch
                servo_changed = True
                self.get_logger().info(f"[A] Pitch DOWN → {self.servo_pitch}°")
            else:
                self.get_logger().warn(f"[A] Pitch DOWN blocked — at maximum ({self.PITCH_MAX}°)")

        # X pressed (rising edge) → Yaw RIGHT (increase yaw angle)
        if cur_X == 1 and self.prev_btn_X == 0:
            new_yaw = min(self.YAW_MAX, self.servo_yaw + self.SERVO_STEP)
            if new_yaw != self.servo_yaw:
                self.servo_yaw = new_yaw
                servo_changed = True
                self.get_logger().info(f"[X] Yaw LEFT → {self.servo_yaw}°")
            else:
                self.get_logger().warn(f"[X] Yaw LEFT blocked — at maximum ({self.YAW_MAX}°)")

        # B pressed (rising edge) → Yaw LEFT (decrease yaw angle)
        if cur_B == 1 and self.prev_btn_B == 0:
            new_yaw = max(self.YAW_MIN, self.servo_yaw - self.SERVO_STEP)
            if new_yaw != self.servo_yaw:
                self.servo_yaw = new_yaw
                servo_changed = True
                self.get_logger().info(f"[X] Yaw RIGHT → {self.servo_yaw}°")
            else:
                self.get_logger().warn(f"[X] Yaw RIGHT blocked — at minimum ({self.YAW_MIN}°)")

        # Publish servo command only on a state change
        if servo_changed:
            self._publish_servo_cmd(self.servo_pitch, self.servo_yaw)

        # Update previous button states
        self.prev_btn_A = cur_A
        self.prev_btn_B = cur_B
        self.prev_btn_X = cur_X
        self.prev_btn_Y = cur_Y

        # -----------------------------
        # SELECT → Park | START → Default Scan
        # -----------------------------
        cur_SELECT = btn(self.btn_SELECT)
        cur_START  = btn(self.btn_START)

        # SELECT pressed (rising edge) → move to PARKED position
        if cur_SELECT == 1 and self.prev_btn_SELECT == 0:
            self.servo_yaw   = self.PARKED_YAW
            self.servo_pitch = self.PARKED_PITCH
            self._publish_servo_cmd(self.servo_pitch, self.servo_yaw)
            self.get_logger().info(
                f"[SELECT] → PARKED position: yaw={self.servo_yaw}°, pitch={self.servo_pitch}°"
            )

        # START pressed (rising edge) → move to DEFAULT SCAN position
        if cur_START == 1 and self.prev_btn_START == 0:
            self.servo_yaw   = self.DEFAULT_YAW
            self.servo_pitch = self.DEFAULT_PITCH
            self._publish_servo_cmd(self.servo_pitch, self.servo_yaw)
            self.get_logger().info(
                f"[START] → DEFAULT SCAN position: yaw={self.servo_yaw}°, pitch={self.servo_pitch}°"
            )

        self.prev_btn_SELECT = cur_SELECT
        self.prev_btn_START  = cur_START

        # -----------------------------
        # Status Shortcut Buttons 6-9
        # Each publishes a preset /bot_status message via ros2 topic pub --once
        # -----------------------------
        cur_6 = btn(self.btn_STATUS_6)
        cur_7 = btn(self.btn_STATUS_7)
        cur_8 = btn(self.btn_STATUS_8)
        cur_9 = btn(self.btn_STATUS_9)

        for cur, prev, btn_idx in [
            (cur_6, self.prev_btn_6, 6),
            (cur_7, self.prev_btn_7, 7),
            (cur_8, self.prev_btn_8, 8),
            (cur_9, self.prev_btn_9, 9),
        ]:
            if cur == 1 and prev == 0:
                status_msg = self.STATUS_MESSAGES[btn_idx]
                self.get_logger().info(
                    f"[BTN {btn_idx}] Publishing /bot_status → '{status_msg}'"
                )
                subprocess.Popen([
                    'ros2', 'topic', 'pub', '--once',
                    '/bot_status',
                    'std_msgs/msg/String',
                    f"{{data: '{status_msg}'}}"
                ])

        self.prev_btn_6 = cur_6
        self.prev_btn_7 = cur_7
        self.prev_btn_8 = cur_8
        self.prev_btn_9 = cur_9

        # -----------------------------
        # Publish Twist
        # -----------------------------
        twist = Twist()

        # Forward joystick gives negative value
        twist.linear.x = msg.axes[self.linear_axis] * self.linear_scale

        # Right joystick may also need inversion
        twist.angular.z = msg.axes[self.angular_axis] * self.angular_scale

        self.publisher.publish(twist)

    def _publish_servo_cmd(self, pitch: int, yaw: int):
        """Publish a MOVE command to /camera_servo_cmd (JSON, same format as rack scanner)."""
        cmd = {
            "cmd": "MOVE",
            "pitch": pitch,
            "yaw": yaw,
            "speed": self.SERVO_SPEED,
        }
        msg = String()
        msg.data = json.dumps(cmd)
        self.servo_cmd_pub.publish(msg)
        self.get_logger().info(
            f"Servo CMD → pitch={pitch}°, yaw={yaw}°, speed={self.SERVO_SPEED}"
        )


def main(args=None):

    rclpy.init(args=args)

    node = PS2Teleop()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()