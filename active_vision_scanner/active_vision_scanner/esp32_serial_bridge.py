#!/usr/bin/env python3
"""
ESP32 WiFi Bridge — ROS2 Node
===============================

Bridges ROS2 topics to/from the ESP32 servo controller over WiFi (HTTP).

All devices (laptop, ESP32-CAM, ESP32 servo controller) connect to the
same mobile hotspot. This node sends JSON commands via HTTP POST to the
ESP32's /cmd endpoint and publishes responses.

Topics:
    Subscribes:  /camera_servo_cmd  (std_msgs/String) — JSON commands to forward
    Subscribes:  /camera_system     (std_msgs/String) — "stop" → forward STOP
    Publishes:   /esp32_response    (std_msgs/String) — ESP32 HTTP responses

Usage:
    ros2 run active_vision_scanner esp32_bridge \\
        --ros-args -p esp32_ip:=192.168.X.X
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import json
import requests
import threading
import time


class ESP32WiFiBridge(Node):
    """Bridge between ROS2 topics and ESP32 WiFi HTTP server."""

    def __init__(self):
        super().__init__('esp32_wifi_bridge')

        # Parameters
        self.declare_parameter('esp32_ip', '10.225.34.175')
        self.declare_parameter('esp32_port', 80)
        self.declare_parameter('timeout', 5.0)
        self.declare_parameter('status_poll_rate', 2.0)  # Hz for status polling

        self.esp32_ip = self.get_parameter('esp32_ip').value
        self.esp32_port = self.get_parameter('esp32_port').value
        self.timeout = self.get_parameter('timeout').value
        self.status_poll_rate = self.get_parameter('status_poll_rate').value

        self.base_url = f"http://{self.esp32_ip}:{self.esp32_port}"
        self.cmd_url = f"{self.base_url}/cmd"
        self.status_url = f"{self.base_url}/status"

        # Subscriber: forward servo commands to ESP32 via HTTP
        self.servo_cmd_sub = self.create_subscription(
            String, '/camera_servo_cmd', self.servo_cmd_callback, 10
        )

        # Subscriber: forward camera system commands
        self.camera_system_sub = self.create_subscription(
            String, '/camera_system', self.camera_system_callback, 10
        )

        # Publisher: ESP32 responses
        self.esp32_response_pub = self.create_publisher(String, '/esp32_response', 10)

        # HTTP session for connection reuse
        self.session = requests.Session()

        # Lock for thread-safe HTTP requests
        self.http_lock = threading.Lock()

        # Check initial connection
        self._check_connection()

        self.get_logger().info(
            f"ESP32 WiFi Bridge ready.\n"
            f"  ESP32 URL : {self.base_url}\n"
            f"  Cmd URL   : {self.cmd_url}\n"
            f"  Status URL: {self.status_url}"
        )

    def _check_connection(self):
        """Verify ESP32 is reachable on the network."""
        try:
            resp = self.session.get(self.status_url, timeout=self.timeout)
            if resp.status_code == 200:
                data = resp.json()
                self.get_logger().info(
                    f"ESP32 connected! Yaw={data.get('yaw')}°, "
                    f"Pitch={data.get('pitch')}°, IP={data.get('ip')}"
                )
            else:
                self.get_logger().warn(
                    f"ESP32 responded with status {resp.status_code}"
                )
        except requests.exceptions.ConnectionError:
            self.get_logger().warn(
                f"Cannot reach ESP32 at {self.base_url} — "
                f"make sure it's connected to the same hotspot. "
                f"Will retry when commands are sent."
            )
        except Exception as e:
            self.get_logger().warn(f"Connection check failed: {e}")

    def _send_http_command(self, json_data: str) -> str:
        """
        Send a JSON command to ESP32 via HTTP POST.
        Returns the response body or an error string.
        Thread-safe.
        """
        with self.http_lock:
            try:
                resp = self.session.post(
                    self.cmd_url,
                    data=json_data,
                    headers={'Content-Type': 'application/json'},
                    timeout=self.timeout,
                )
                self.get_logger().info(
                    f"→ ESP32 [{resp.status_code}]: {json_data.strip()}"
                )

                # Publish response
                resp_msg = String()
                resp_msg.data = resp.text
                self.esp32_response_pub.publish(resp_msg)

                self.get_logger().info(f"← ESP32: {resp.text}")
                return resp.text

            except requests.exceptions.ConnectionError:
                err = f"Cannot reach ESP32 at {self.cmd_url}"
                self.get_logger().error(err)
                return json.dumps({"status": "error", "msg": err})

            except requests.exceptions.Timeout:
                err = f"Timeout sending to ESP32 ({self.timeout}s)"
                self.get_logger().error(err)
                return json.dumps({"status": "error", "msg": err})

            except Exception as e:
                err = f"HTTP error: {e}"
                self.get_logger().error(err)
                return json.dumps({"status": "error", "msg": err})

    def servo_cmd_callback(self, msg: String):
        """Forward servo command JSON to ESP32 via HTTP POST."""
        # Run in a separate thread to avoid blocking the ROS2 executor
        threading.Thread(
            target=self._send_http_command,
            args=(msg.data,),
            daemon=True,
        ).start()

    def camera_system_callback(self, msg: String):
        """Forward camera system commands to ESP32."""
        data = msg.data.strip().lower()
        if data == "stop":
            stop_cmd = json.dumps({"cmd": "STOP"})
            self.get_logger().info("Forwarding STOP command to ESP32...")
            threading.Thread(
                target=self._send_http_command,
                args=(stop_cmd,),
                daemon=True,
            ).start()

    def destroy_node(self):
        self.session.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ESP32WiFiBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down ESP32 WiFi bridge.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
