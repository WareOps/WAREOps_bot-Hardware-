#!/usr/bin/env python3
"""
Active Vision Rack Scanning System - ROS2 OpenCV Node
=====================================================

Network Setup:
    All devices connect to the SAME mobile hotspot:
    - Laptop (runs this ROS2 node + esp32_wifi_bridge)
    - ESP32-CAM (streams video over WiFi)
    - ESP32 Dev Kit V1 (servo controller, receives HTTP commands)

This node subscribes to a robot navigation topic to know when the bot has
reached a specific aisle/rack position. Upon receiving a "Reached" message,
it commands the ESP32 servo controller (via the esp32_wifi_bridge node) to
move the camera from its parked position (yaw=100, pitch=90) to the default
scanning position (yaw=10, pitch=90), automatically adjusts pitch so the full rack is in frame,
runs QR-code detection on each shelf region, then publishes "Succeeded" and
parks the camera again.

Physical Parameters (user-specified):
    Camera height from ground : 13.35 cm
    Rack height (default)     : 49 cm  (Aisle 2 Rack 1&2: 42 cm)
    Rack horizontal (default) : 60 cm  (Aisle 2 Rack 1&2: 52 cm)
    Camera distance from rack : 45 cm
    Num shelves (default)     : 4      (Aisle 2 Rack 1&2: 3)
    Parked orientation        : yaw=100°, pitch=90°
    Detection orientation     : yaw=10°, pitch=90°
    Pitch range               : min=20°, max=160°

ROS2 Topics:
    /bot_status        (std_msgs/String)  - Subscribe & Publish
        Robot publishes  : "Reached Aisle X, RackY"
        This node publishes: "Succeeded Aisle X, RackY"
    /camera_servo_cmd  (std_msgs/String)  - Publish servo commands as JSON
        → esp32_wifi_bridge forwards to ESP32 via HTTP POST
    /camera_system     (std_msgs/String)  - Subscribe; "stop" → shutdown
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import cv2
import numpy as np
import json
import time
import threading
import os
import csv
import re
import openpyxl
from enum import Enum, auto


# ---------------------------------------------------------------------------
# Constants – Physical Setup (defaults, can be overridden per-rack)
# ---------------------------------------------------------------------------
CAMERA_HEIGHT_CM = 13.35          # Camera height from ground (cm)
RACK_HEIGHT_CM = 24.5             # Total rack height (cm) — halved from original
RACK_HORIZONTAL_CM = 60.0         # Rack horizontal width (cm)
CAMERA_DISTANCE_CM = 45.0         # Distance from camera to rack (cm, approximately 45cm)
NUM_SHELVES = 2                   # Number of shelves (Shelf 1 = top, Shelf 2 = bottom)

# Camera FOV (OV3660 approximate)
HFOV_DEG = 66.0                   # Horizontal FOV (degrees)
VFOV_DEG = 52.0                   # Vertical FOV (degrees)

# Frame resolution
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# ---------------------------------------------------------------------------
# Rack-specific overrides  (aisle_code, rack_code) → {param: value}
# Only racks that differ from the defaults need entries here.
# ---------------------------------------------------------------------------
RACK_OVERRIDES = {
    # All racks are now uniform: 2 shelves, 24.5 cm tall.
    # Add per-rack overrides here if any rack differs in the future.
}


def compute_shelf_regions(num_shelves: int, frame_height: int = FRAME_HEIGHT) -> dict:
    """Compute equal horizontal pixel bands for the given number of shelves."""
    band = frame_height // num_shelves
    regions = {}
    for i in range(1, num_shelves + 1):
        y_min = (i - 1) * band
        y_max = i * band if i < num_shelves else frame_height
        regions[i] = (y_min, y_max)
    return regions


# Default shelf regions (2 shelves: top = Shelf 1, bottom = Shelf 2)
SHELF_REGIONS = compute_shelf_regions(NUM_SHELVES)

# Servo angles
PARKED_YAW = 100      # Parked yaw (stowed position)
PARKED_PITCH = 90     # Parked pitch (stowed position)
DEFAULT_YAW = 10      # Default scanning / detection yaw (camera faces straight towards rack)
DEFAULT_PITCH = 90    # Default scanning pitch (will be auto-adjusted)

# Pitch adjustment limits
PITCH_STEP_DEG = 2    # Degrees per adjustment step
PITCH_MIN = 20        # Minimum allowed pitch (tilt UP limit)
PITCH_MAX = 160       # Maximum allowed pitch (tilt DOWN limit)

# ESP32-CAM stream URL — all devices on same mobile hotspot
# Update this IP to match your ESP32-CAM's IP on the hotspot network
ESP32_CAM_STREAM_URL = "http://10.225.34.222:81/stream"

# Servo movement speed (degrees per command for ESP32 trajectory)
SERVO_SPEED = 20


# ---------------------------------------------------------------------------
# State machine
# ---------------------------------------------------------------------------
class ScannerState(Enum):
    IDLE = auto()              # Waiting for a "Reached" message
    MOVING_TO_SCAN = auto()    # Moving servos from parked → default scan pos
    ADJUSTING_PITCH = auto()   # Auto-adjusting pitch to frame the rack
    SCANNING = auto()          # QR detection in progress
    MOVING_TO_PARK = auto()    # Moving servos back to parked position
    SHUTDOWN = auto()          # Shutting down


class RackScannerNode(Node):
    """ROS2 node that orchestrates rack scanning with OpenCV + servo control."""

    def __init__(self):
        super().__init__('rack_scanner_node')

        # ------- Parameters (overridable via ROS2 params) -------
        self.declare_parameter('esp32_cam_url', ESP32_CAM_STREAM_URL)
        self.declare_parameter('camera_height_cm', CAMERA_HEIGHT_CM)
        self.declare_parameter('rack_height_cm', RACK_HEIGHT_CM)
        self.declare_parameter('rack_horizontal_cm', RACK_HORIZONTAL_CM)
        self.declare_parameter('camera_distance_cm', CAMERA_DISTANCE_CM)
        self.declare_parameter('num_shelves', NUM_SHELVES)
        self.declare_parameter('database_path', '/home/abhinav/warehouse_bot_simulation_ws/src/warehouse_database.xlsx')
        self.declare_parameter('scanned_inventory_path', '/home/abhinav/warehouse_bot_simulation_ws/src/scanned_inventory.csv')
        self.declare_parameter('mismatch_log_path', '/home/abhinav/warehouse_bot_simulation_ws/src/mismatch_log.csv')

        self.cam_url = self.get_parameter('esp32_cam_url').value
        self.camera_height = self.get_parameter('camera_height_cm').value
        self.rack_height = self.get_parameter('rack_height_cm').value
        self.rack_horizontal = self.get_parameter('rack_horizontal_cm').value
        self.camera_distance = self.get_parameter('camera_distance_cm').value
        self.num_shelves = self.get_parameter('num_shelves').value
        self.db_path = self.get_parameter('database_path').value
        self.scanned_inv_path = self.get_parameter('scanned_inventory_path').value
        self.mismatch_log_path = self.get_parameter('mismatch_log_path').value

        # ------- Publishers -------
        self.bot_status_pub = self.create_publisher(String, '/bot_status', 10)
        self.servo_cmd_pub = self.create_publisher(String, '/camera_servo_cmd', 10)

        # ------- Subscribers -------
        self.bot_status_sub = self.create_subscription(
            String, '/bot_status', self.bot_status_callback, 10
        )
        self.camera_system_sub = self.create_subscription(
            String, '/camera_system', self.camera_system_callback, 10
        )

        # ------- Internal state -------
        self.state = ScannerState.IDLE
        self.current_yaw = PARKED_YAW
        self.current_pitch = PARKED_PITCH
        self.current_location = ""        # e.g. "Aisle_1/Row_1/Rack_1"
        self.scanned_aisle = ""
        self.scanned_row = ""
        self.scanned_rack = ""
        self.scan_results = []            # Detected products
        self.cap = None                   # OpenCV VideoCapture
        self.lock = threading.Lock()
        self.shutdown_event = threading.Event()
        self.stream = None
        self.stream_bytes = b''
        self.latest_frame = None

        # ------- Image enhancement objects (cached to avoid per-frame allocation) -------
        self.clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        self.sharpen_kernel = np.array([[ 0, -1,  0],
                                        [-1,  5, -1],
                                        [ 0, -1,  0]], dtype=np.float32)

        # ------- Active scan parameters (set per-rack before each scan) -------
        self.active_num_shelves = self.num_shelves
        self.active_rack_height = self.rack_height
        self.active_rack_horizontal = self.rack_horizontal
        self.active_shelf_regions = SHELF_REGIONS

        # QR detector — WeChatQRCode is primary, standard QRCodeDetector as fallback
        self.fallback_qr_detector = cv2.QRCodeDetector()
        try:
            self.qr_detector = cv2.wechat_qrcode_WeChatQRCode()
            self.get_logger().info("Using WeChatQRCode detector (with QRCodeDetector fallback).")
        except Exception:
            self.qr_detector = self.fallback_qr_detector
            self.get_logger().warn("WeChatQRCode unavailable, using QRCodeDetector.")

        # Load QR database
        self.database = {}
        self.load_database()

        # Timer for the main processing loop (10 Hz)
        self.create_timer(0.1, self.process_loop)

        self.get_logger().info(
            f"Rack Scanner Node started.\n"
            f"  Camera height : {self.camera_height} cm\n"
            f"  Rack height   : {self.rack_height} cm\n"
            f"  Rack width    : {self.rack_horizontal} cm\n"
            f"  Shelves       : {self.num_shelves}\n"
            f"  Parked pos    : yaw={PARKED_YAW}°, pitch={PARKED_PITCH}°\n"
            f"  Default scan  : yaw={DEFAULT_YAW}°, pitch={DEFAULT_PITCH}°\n"
            f"  Database file : {self.db_path}\n"
            f"  ESP32 stream  : {self.cam_url}"
        )

    def load_database(self):
        self.get_logger().info(f"Loading QR database from {self.db_path}...")
        if not os.path.exists(self.db_path):
            self.get_logger().error(f"Excel database file not found at {self.db_path}!")
            return
        
        try:
            wb = openpyxl.load_workbook(self.db_path)
            sheet = wb.active
            count = 0
            for r in range(2, sheet.max_row + 1):
                qr = sheet.cell(r, 1).value
                if qr:
                    qr = str(qr).strip()
                    self.database[qr] = {
                        'product_code': sheet.cell(r, 2).value,
                        'serial_number': sheet.cell(r, 3).value,
                        'category_number': sheet.cell(r, 4).value,
                        'aisle': sheet.cell(r, 5).value,
                        'row': sheet.cell(r, 6).value,
                        'rack': sheet.cell(r, 7).value,
                        'shelf': sheet.cell(r, 8).value
                    }
                    count += 1
            self.get_logger().info(f"Successfully loaded {count} entries from database.")
        except Exception as e:
            self.get_logger().error(f"Error loading Excel database: {e}")

    # ===========================================================================
    # ROS2 Callbacks
    # ===========================================================================
    def bot_status_callback(self, msg: String):
        """Handle messages on /bot_status."""
        data = msg.data.strip()
        self.get_logger().info(f"[/bot_status] Received: '{data}'")

        # Only react to "Reached ..." messages while idle
        if data.lower().startswith("reached") and self.state == ScannerState.IDLE:
            loc_str = data.replace("Reached", "").strip()
            parsed_aisle, parsed_row, parsed_rack = self.parse_location(loc_str)
            if parsed_aisle and parsed_row and parsed_rack:
                self.current_location = loc_str
                self.scanned_aisle = parsed_aisle
                self.scanned_row = parsed_row
                self.scanned_rack = parsed_rack
                # Apply rack-specific overrides (or fall back to defaults)
                self._apply_rack_overrides(parsed_aisle, parsed_rack)
                self.get_logger().info(
                    f"Bot reached '{self.current_location}' -> Aisle: {self.scanned_aisle}, Row: {self.scanned_row}, Rack: {self.scanned_rack}  "
                    f"[shelves={self.active_num_shelves}, height={self.active_rack_height}cm, width={self.active_rack_horizontal}cm]"
                )
                self.state = ScannerState.MOVING_TO_SCAN
            else:
                self.get_logger().error(f"Failed to parse Aisle/Row/Rack from location: '{loc_str}'")

    def parse_location(self, loc_str):
        # Match e.g., "Aisle_1/Row_1/Rack_1" or "Aisle1/Row1/Rack1"
        match = re.search(r'Aisle_?(\d+)/Row_?(\d+)/Rack_?(\d+)', loc_str, re.IGNORECASE)
        if match:
            return f"A{match.group(1)}", f"R{match.group(2)}", f"RK{match.group(3)}"
        return None, None, None

    def _apply_rack_overrides(self, aisle_code: str, rack_code: str):
        """Set active scan parameters based on rack-specific overrides."""
        key = (aisle_code, rack_code)
        overrides = RACK_OVERRIDES.get(key, {})
        self.active_num_shelves = overrides.get('num_shelves', self.num_shelves)
        self.active_rack_height = overrides.get('rack_height_cm', self.rack_height)
        self.active_rack_horizontal = overrides.get('rack_horizontal_cm', self.rack_horizontal)
        self.active_shelf_regions = compute_shelf_regions(self.active_num_shelves)
        if overrides:
            self.get_logger().info(
                f"Rack override applied for {key}: shelves={self.active_num_shelves}, "
                f"height={self.active_rack_height}cm, width={self.active_rack_horizontal}cm"
            )

    def camera_system_callback(self, msg: String):
        """Handle messages on /camera_system — 'stop' triggers shutdown."""
        data = msg.data.strip().lower()
        self.get_logger().info(f"[/camera_system] Received: '{data}'")

        if data == "stop":
            self.get_logger().warn("Received STOP command — shutting down.")
            self.state = ScannerState.SHUTDOWN
            self.shutdown_event.set()

    # ===========================================================================
    # Servo Command Helpers
    # ===========================================================================
    def send_servo_command(self, pitch: float, yaw: float, speed: int = SERVO_SPEED):
        """Publish a JSON servo command on /camera_servo_cmd. Pitch is hard-clamped <= 90°."""
        pitch = max(PITCH_MIN, min(90, int(pitch)))
        yaw = max(0, min(180, int(yaw)))
        cmd = {
            "cmd": "MOVE",
            "pitch": pitch,
            "yaw": yaw,
            "speed": speed,
        }
        msg = String()
        msg.data = json.dumps(cmd)
        self.servo_cmd_pub.publish(msg)
        self.current_pitch = pitch
        self.current_yaw = yaw
        self.get_logger().info(f"Servo CMD → pitch={pitch}°, yaw={yaw}°, speed={speed}")

    def send_home_command(self):
        """Send HOME command to ESP32."""
        cmd = {"cmd": "HOME"}
        msg = String()
        msg.data = json.dumps(cmd)
        self.servo_cmd_pub.publish(msg)
        self.get_logger().info("Servo CMD → HOME")

    def send_stop_command(self):
        """Send STOP command to ESP32."""
        cmd = {"cmd": "STOP"}
        msg = String()
        msg.data = json.dumps(cmd)
        self.servo_cmd_pub.publish(msg)
        self.get_logger().info("Servo CMD → STOP")

    # ===========================================================================
    # Camera Helpers
    # ===========================================================================
    def open_camera(self) -> bool:
        """Open the ESP32-CAM video stream using urllib."""
        import urllib.request
        import socket
        if hasattr(self, 'stream') and self.stream is not None:
            return True
        self.get_logger().info(f"Opening camera stream: {self.cam_url}")
        try:
            self.stream = urllib.request.urlopen(self.cam_url, timeout=5)
            # Set a short socket-level read timeout to prevent blocking on WiFi drops
            self.stream.fp.raw._sock.settimeout(3.0)
            self.stream_bytes = b''
            self.get_logger().info("Camera stream opened successfully.")
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to open camera stream! {e}")
            self.stream = None
            return False

    def close_camera(self):
        """Release the camera stream."""
        if hasattr(self, 'stream') and self.stream is not None:
            self.stream.close()
            self.stream = None
            self.get_logger().info("Camera stream closed.")

    def grab_frame(self):
        """
        Grab a single enhanced grayscale frame from the camera.
        Returns (ok, gray_frame).

        Robust against partial/corrupt JPEG data from the MJPEG stream.
        Skips bad frames instead of crashing, and limits buffer size to
        prevent stale data accumulation.
        """
        if not hasattr(self, 'stream') or self.stream is None:
            if not self.open_camera():
                return False, None

        try:
            # Read in larger chunks for fewer syscalls and fewer partial-frame issues
            max_attempts = 30
            for _ in range(max_attempts):
                chunk = self.stream.read(4096)
                if not chunk:
                    break
                self.stream_bytes += chunk

                # Prevent unbounded buffer growth (keep last 200KB)
                if len(self.stream_bytes) > 200000:
                    self.stream_bytes = self.stream_bytes[-100000:]

                a = self.stream_bytes.find(b'\xff\xd8')
                b = self.stream_bytes.find(b'\xff\xd9', a + 2 if a != -1 else 0)

                if a != -1 and b != -1 and b > a:
                    jpg = self.stream_bytes[a:b+2]
                    self.stream_bytes = self.stream_bytes[b+2:]

                    # Skip tiny/corrupt JPEG fragments
                    if len(jpg) < 1000:
                        continue

                    frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                    if frame is None:
                        continue  # Bad decode — skip this frame, try next

                    frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))
                    self.latest_frame = frame
                    return True, frame

            # If we exhausted attempts without a valid frame
            return False, None

        except Exception as e:
            self.get_logger().warn(f"Frame grab failed ({e}). Reconnecting...")
            self.close_camera()
            if self.open_camera():
                return self.grab_frame()
            return False, None

    def enhance_frame(self, gray):
        """
        Grayscale-native image enhancement for clear QR detection.
        Uses cached CLAHE and sharpening kernel (no per-frame allocation).
        """
        enhanced = self.clahe.apply(gray)
        sharpened = cv2.filter2D(enhanced, -1, self.sharpen_kernel)
        return sharpened

    # ===========================================================================
    # Rack Framing / Pitch Adjustment
    # ===========================================================================
    def is_rack_fully_visible(self, frame) -> tuple:
        """
        Determine whether the full rack is visible in the frame.

        Strategy: Convert to grayscale, apply edge detection, and check if
        significant edges exist in the top and bottom regions of the frame.
        The top ~10% and bottom ~10% strips are checked.

        Returns (top_visible: bool, bottom_visible: bool).
        """
        gray = frame  # Frame is already grayscale from grab_frame
        edges = cv2.Canny(gray, 50, 150)

        top_strip = edges[0:FRAME_HEIGHT // 10, :]
        bottom_strip = edges[9 * FRAME_HEIGHT // 10:, :]

        top_edge_density = np.count_nonzero(top_strip) / top_strip.size
        bottom_edge_density = np.count_nonzero(bottom_strip) / bottom_strip.size

        # Thresholds — if there are very few edges, the shelf edge is likely
        # outside the frame.
        EDGE_THRESHOLD = 0.02
        top_visible = top_edge_density > EDGE_THRESHOLD
        bottom_visible = bottom_edge_density > EDGE_THRESHOLD

        return top_visible, bottom_visible

    def adjust_pitch_for_rack(self, max_steps: int = 20) -> bool:
        """
        Automatically adjust pitch so the entire rack is in frame.

        Returns True if adjustment succeeded, False on failure/timeout.
        """
        self.get_logger().info("Starting automatic pitch adjustment...")
        for step in range(max_steps):
            ok, frame = self.grab_frame()
            if not ok:
                self.get_logger().warn("Cannot grab frame during pitch adjustment.")
                time.sleep(0.3)
                continue

            top_vis, bottom_vis = self.is_rack_fully_visible(frame)
            self.get_logger().info(
                f"  Step {step}: top_visible={top_vis}, bottom_visible={bottom_vis}, "
                f"pitch={self.current_pitch}°"
            )

            if top_vis and bottom_vis:
                self.get_logger().info(
                    f"Rack fully in frame at pitch={self.current_pitch}°."
                )
                return True

            if not top_vis and bottom_vis:
                # Top shelf is out of frame → decrease pitch (tilt up)
                new_pitch = max(PITCH_MIN, self.current_pitch - PITCH_STEP_DEG)
                self.send_servo_command(new_pitch, self.current_yaw)
            elif top_vis and not bottom_vis:
                # Bottom shelf is out of frame → increase pitch (tilt down)
                new_pitch = min(PITCH_MAX, self.current_pitch + PITCH_STEP_DEG)
                self.send_servo_command(new_pitch, self.current_yaw)
            else:
                # Neither visible — try decreasing pitch (camera too close or wrong angle)
                new_pitch = max(PITCH_MIN, self.current_pitch - PITCH_STEP_DEG)
                self.send_servo_command(new_pitch, self.current_yaw)

            # Wait for servo to settle
            time.sleep(0.5)

        self.get_logger().warn("Pitch adjustment did not converge within max steps.")
        return False

    # ===========================================================================
    # QR Code Detection & Debug Visualizer
    # ===========================================================================
    # ===========================================================================
    # Black Rack Boundary & QR Detection Visualizer
    # ===========================================================================
    def detect_black_rack_boundary(self, frame_gray) -> tuple:
        """
        Detect black rack structure boundary using grayscale thresholding.
        Returns (y_min, y_max, x_min, x_max, found_valid_box).
        """
        if frame_gray is None:
            return 0, FRAME_HEIGHT, 0, FRAME_WIDTH, False

        # Threshold dark pixels (black rack structure has low intensity)
        _, mask = cv2.threshold(frame_gray, 60, 255, cv2.THRESH_BINARY_INV)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        valid = [c for c in contours if cv2.contourArea(c) > 2000]
        if valid:
            x_min = min(cv2.boundingRect(c)[0] for c in valid)
            y_min = min(cv2.boundingRect(c)[1] for c in valid)
            x_max = max(cv2.boundingRect(c)[0] + cv2.boundingRect(c)[2] for c in valid)
            y_max = max(cv2.boundingRect(c)[1] + cv2.boundingRect(c)[3] for c in valid)

            y_min = max(0, y_min)
            y_max = min(FRAME_HEIGHT, y_max)
            x_min = max(0, x_min)
            x_max = min(FRAME_WIDTH, x_max)
            return y_min, y_max, x_min, x_max, True

        return 0, FRAME_HEIGHT, 0, FRAME_WIDTH, False

    def determine_shelf_by_rack_boundary(self, center_y: float, y_min: int, y_max: int) -> int:
        """
        Divide the detected black rack vertical region [y_min, y_max] into N equal shelf bands.
        Align QR center_y into Shelf 1 to N.
        """
        n = self.active_num_shelves
        h_rack = max(80, y_max - y_min)
        shelf_band = h_rack / float(n)
        
        rel_y = center_y - y_min
        shelf = int(rel_y // shelf_band) + 1
        return max(1, min(n, shelf))

    def update_debug_view(self, detections=None, status_text: str = ""):
        """Render live original camera feed window with detected QR bounding boxes."""
        if not hasattr(self, 'latest_frame') or self.latest_frame is None:
            return

        debug_img = self.latest_frame.copy()

        # Draw detected QR code bounding boxes and text (green on original color feed)
        if detections:
            for det in detections:
                cx, cy = int(det['center_x']), int(det['center_y'])
                data = det['raw_data']
                cv2.circle(debug_img, (cx, cy), 6, (0, 255, 0), -1)
                cv2.putText(
                    debug_img, f"QR: {data[:20]}", (cx - 40, max(20, cy - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2
                )
                if 'pts' in det and det['pts'] is not None:
                    pts = np.int32(det['pts'])
                    cv2.polylines(debug_img, [pts], True, (0, 255, 0), 2)

        # Overlay status info on frame
        overlay = f"Pitch: {self.current_pitch}deg | Yaw: {self.current_yaw}deg"
        cv2.putText(debug_img, overlay, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 2)
        if status_text:
            cv2.putText(debug_img, status_text, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

        cv2.imshow("Rack Scanner Debug Feed", debug_img)
        cv2.waitKey(1)

    def detect_qr_codes_in_frame(self, frame) -> list:
        """
        Detect and decode all QR codes in a single frame.
        Uses a robust multi-candidate + multi-engine pipeline:
        1. Prepares 5 image candidates:
           - Enhanced Grayscale
           - Adaptive Gaussian Threshold
           - Adaptive Mean Threshold
           - Otsu Binarization
           - 1.5x Upscaled Zoom (for distant/small QR codes)
        2. Applies WeChatQRCode as primary detector across all candidates.
        3. Applies OpenCV QRCodeDetector as a secondary engine fallback.
        Returns a list of dicts: {'raw_data', 'product_info', 'center_x', 'center_y', 'pts'}
        """
        detections = []
        seen_in_frame = set()

        # Prepare robust image candidates (Candidate 1 is original raw camera feed)
        candidates = [{'img': frame, 'scale': 1.0}]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if len(frame.shape) == 3 else frame
        candidates.append({'img': gray, 'scale': 1.0})

        # Candidate 2: Adaptive Gaussian Binarization (Great for low contrast / blurry QRs)
        try:
            thresh_gauss = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 15, 3)
            candidates.append({'img': thresh_gauss, 'scale': 1.0})
        except Exception:
            pass

        # Candidate 3: Adaptive Mean Binarization (Great for uneven lighting / shadows)
        try:
            thresh_mean = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 21, 5)
            candidates.append({'img': thresh_mean, 'scale': 1.0})
        except Exception:
            pass

        # Candidate 4: Otsu Global Binarization
        try:
            _, otsu = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
            candidates.append({'img': otsu, 'scale': 1.0})
        except Exception:
            pass

        # Candidate 5: 1.5x Upscaled Zoom (Enables detection of tiny/faraway QR modules)
        try:
            upscaled = cv2.resize(gray, (0, 0), fx=1.5, fy=1.5, interpolation=cv2.INTER_CUBIC)
            candidates.append({'img': upscaled, 'scale': 1.5})
        except Exception:
            pass

        # Engine 1: WeChatQRCode Primary
        if isinstance(self.qr_detector, cv2.wechat_qrcode_WeChatQRCode):
            for cand in candidates:
                img = cand['img']
                scale = cand['scale']
                try:
                    decoded_list, points_list = self.qr_detector.detectAndDecode(img)
                    if decoded_list:
                        for i, data in enumerate(decoded_list):
                            data = str(data).strip()
                            if data and data not in seen_in_frame:
                                seen_in_frame.add(data)
                                cx, cy = FRAME_WIDTH / 2.0, FRAME_HEIGHT / 2.0
                                pts = None
                                if points_list and i < len(points_list) and points_list[i] is not None:
                                    pts = points_list[i] / scale
                                    cx = float(np.mean(pts[:, 0]))
                                    cy = float(np.mean(pts[:, 1]))
                                detections.append({
                                    'raw_data': data,
                                    'product_info': self._parse_qr_data(data),
                                    'center_x': cx,
                                    'center_y': cy,
                                    'pts': pts,
                                })
                except Exception:
                    pass

        # Engine 2: OpenCV QRCodeDetector Secondary Engine (catches edge cases)
        for cand in candidates:
            img = cand['img']
            scale = cand['scale']
            try:
                retval, decoded_info, points, _ = self.fallback_qr_detector.detectAndDecodeMulti(img)
                if retval and decoded_info is not None:
                    for i, data in enumerate(decoded_info):
                        data = str(data).strip()
                        if data and data not in seen_in_frame:
                            seen_in_frame.add(data)
                            bbox = points[i] / scale
                            cx = float(np.mean(bbox[:, 0]))
                            cy = float(np.mean(bbox[:, 1]))
                            detections.append({
                                'raw_data': data,
                                'product_info': self._parse_qr_data(data),
                                'center_x': cx,
                                'center_y': cy,
                                'pts': bbox,
                            })
            except Exception:
                pass

        return detections

    def _parse_qr_data(self, data: str) -> dict:
        """Try to parse QR data as JSON; fall back to raw string."""
        try:
            return json.loads(data)
        except (json.JSONDecodeError, TypeError):
            return {'raw': data}

    def _wait_for_servo(self, old_pitch, old_yaw, new_pitch, new_yaw, status_text=""):
        """Wait the appropriate time for servos to reach their target while keeping debug feed updated."""
        travel = max(abs(new_pitch - old_pitch), abs(new_yaw - old_yaw))
        wait_time = travel * 0.05 + 0.4  # ~50ms per degree + 400ms settle buffer
        start_t = time.time()
        while time.time() - start_t < wait_time:
            self.grab_frame()
            self.update_debug_view([], f"Moving... {status_text}")
            time.sleep(0.05)

    def _scan_at_position(self, num_frames: int = 10, status_text: str = "") -> list:
        """
        Grab num_frames and collect all unique QR codes detected.
        """
        found = {}
        for _ in range(num_frames):
            ok, frame = self.grab_frame()
            if ok and frame is not None:
                dets = self.detect_qr_codes_in_frame(frame)
                for d in dets:
                    key = d['raw_data']
                    if key not in found:
                        found[key] = d
                self.update_debug_view(dets, status_text)
            time.sleep(0.08)
        return list(found.values())

    def run_scan(self) -> list:
        """
        Scan rack by sweeping camera across 2 pitch levels (one per shelf)
        and multiple yaw offsets to detect all QR codes present.
        """
        center_yaw = self.current_yaw  # Default 10°
        globally_seen_qrs = set()
        all_detections = []

        self.get_logger().info(f"Starting 2-Shelf QR Detection Sweep (center_yaw={center_yaw}°)...")

        # 2 pitch levels for 2 shelves on a 24.5cm rack:
        #   Shelf 1 (top)  → pitch ~60° (camera tilts slightly up)
        #   Shelf 2 (bottom) → pitch ~80° (camera tilts slightly down)
        pitch_steps = [60.0, 80.0]
        yaw_offsets = [0, -25, 25]

        for step_idx, target_pitch in enumerate(pitch_steps):
            target_pitch = min(90.0, target_pitch)

            for offset in yaw_offsets:
                pan_yaw = max(0, min(180, center_yaw + offset))
                status = f"Shelf {step_idx+1}/2 | Pitch {target_pitch}° | Yaw {pan_yaw}°"

                old_pitch, old_yaw = self.current_pitch, self.current_yaw
                self.send_servo_command(target_pitch, pan_yaw)
                self._wait_for_servo(old_pitch, old_yaw, target_pitch, pan_yaw, status_text=status)

                # Grab frames and detect all QRs in view
                detections = self._scan_at_position(num_frames=8, status_text=status)
                new_dets = [d for d in detections if d['raw_data'] not in globally_seen_qrs]

                for d in new_dets:
                    globally_seen_qrs.add(d['raw_data'])
                    all_detections.append(d)
                    self.get_logger().info(
                        f"  [QR #{len(all_detections)}] Detected: '{d['raw_data'][:30]}' (pitch={target_pitch}°, yaw={pan_yaw}°)"
                    )

            # Return yaw to center before next pitch step
            if self.current_yaw != center_yaw:
                self.send_servo_command(target_pitch, center_yaw)
                self._wait_for_servo(target_pitch, pan_yaw, target_pitch, center_yaw, status_text="Resetting Yaw...")

        self.get_logger().info("=" * 50)
        self.get_logger().info(f"RACK SCAN COMPLETED: Total {len(all_detections)} unique QR code(s) detected.")
        for idx, det in enumerate(all_detections, 1):
            det['count'] = 1
            det['shelf_detected'] = 1
            self.get_logger().info(f"  {idx}. QR='{det['raw_data']}'")
        self.get_logger().info("=" * 50)

        return all_detections



    # ===========================================================================
    # Main Processing Loop (called by ROS2 timer)
    # ===========================================================================
    def process_loop(self):
        """State machine — runs at 10 Hz."""

        # ---- SHUTDOWN ----
        if self.state == ScannerState.SHUTDOWN:
            self.get_logger().info("Shutdown state — cleaning up...")
            self.send_stop_command()
            self.close_camera()
            rclpy.shutdown()
            return

        # ---- IDLE ----
        if self.state == ScannerState.IDLE:
            # Nothing to do; waiting for /bot_status callback
            return

        # ---- MOVING TO SCAN POSITION ----
        if self.state == ScannerState.MOVING_TO_SCAN:
            self.get_logger().info("Moving camera from parked to default scan position...")

            # 1. Command servos to default scan orientation FIRST
            self.send_servo_command(DEFAULT_PITCH, DEFAULT_YAW)

            # 2. Wait for servos to reach position
            travel_yaw = abs(PARKED_YAW - DEFAULT_YAW)
            travel_pitch = abs(PARKED_PITCH - DEFAULT_PITCH)
            max_travel = max(travel_yaw, travel_pitch)
            wait_time = max_travel * 0.05 + 0.5  # ~50ms per degree + buffer
            self.get_logger().info(f"Waiting {wait_time:.1f}s for servos to settle...")
            time.sleep(wait_time)

            # 3. Open camera AFTER servos have started moving/settled
            if not self.open_camera():
                self.get_logger().error("Cannot open camera — aborting scan.")
                self.state = ScannerState.IDLE
                return

            self.state = ScannerState.SCANNING

        # ---- SCANNING ----
        elif self.state == ScannerState.SCANNING or self.state == ScannerState.ADJUSTING_PITCH:
            self.scan_results = self.run_scan()

            # Match scanned QRs against the database and log to CSV
            self.process_scan_results(self.scan_results)

            # Publish success
            result_msg = String()
            result_msg.data = f"Succeeded {self.current_location}"
            self.bot_status_pub.publish(result_msg)
            self.get_logger().info(f"Published: '{result_msg.data}'")

            self.state = ScannerState.MOVING_TO_PARK

        # ---- MOVING TO PARK ----
        elif self.state == ScannerState.MOVING_TO_PARK:
            self.get_logger().info("Moving camera back to parked position...")
            self.send_servo_command(PARKED_PITCH, PARKED_YAW)

            # Wait for servos
            travel_yaw = abs(self.current_yaw - PARKED_YAW)
            travel_pitch = abs(self.current_pitch - PARKED_PITCH)
            max_travel = max(travel_yaw, travel_pitch)
            wait_time = max_travel * 0.05 + 0.5
            time.sleep(wait_time)

            self.close_camera()
            self.get_logger().info("Scan cycle complete — returning to IDLE.")
            self.state = ScannerState.IDLE

    def process_scan_results(self, results):
        self.get_logger().info("Processing scan results against database...")
        
        # Prepare list for this position's correct scans
        position_correct_scans = []

        for r in results:
            qr_data = r['raw_data'].strip()
            
            if qr_data in self.database:
                db_entry = self.database[qr_data]
                expected_aisle = db_entry['aisle']
                expected_row = db_entry['row']
                expected_rack = db_entry['rack']
                shelf_code = db_entry.get('shelf', 'N/A')
                
                is_correct = (
                    expected_aisle == self.scanned_aisle and
                    expected_row == self.scanned_row and
                    expected_rack == self.scanned_rack
                )
                
                if is_correct:
                    self.get_logger().info(f"Match: QR code '{qr_data}' ({db_entry['product_code']}) correctly placed at {self.current_location}")
                    self.log_correct_scan(qr_data, db_entry, shelf_code)
                    position_correct_scans.append({
                        'QR_Code': qr_data,
                        'Product_Code': db_entry['product_code'],
                        'Product_Serial_Number': db_entry['serial_number'],
                        'Category_Number': db_entry['category_number'],
                        'Shelf': shelf_code
                    })
                else:
                    self.get_logger().warn(
                        f"Mismatch! At position {self.scanned_aisle}/{self.scanned_row}/{self.scanned_rack}, "
                        f"QR: '{qr_data}' ({db_entry['product_code']}) is present but expected in "
                        f"Aisle: {expected_aisle}, Row: {expected_row}, Rack: {expected_rack}"
                    )
                    self.log_mismatch_scan(qr_data, db_entry, shelf_code, expected_aisle, expected_row, expected_rack, shelf_code)
            else:
                self.get_logger().error(f"Unknown QR code scanned: '{qr_data}' at {self.current_location}!")
                self.log_unknown_scan(qr_data, "N/A")

        # Compile that position's all data in a position-specific CSV file
        if position_correct_scans:
            self.compile_position_csv(position_correct_scans)

    def log_correct_scan(self, qr, db_entry, shelf):
        file_exists = os.path.exists(self.scanned_inv_path)
        try:
            with open(self.scanned_inv_path, 'a', newline='') as f:
                writer = csv.writer(f)
                if not file_exists:
                    writer.writerow([
                        'Timestamp', 'Scanned_Location', 'Aisle', 'Row', 'Rack', 'Shelf',
                        'QR_Code', 'Product_Code', 'Product_Serial_Number', 'Category_Number'
                    ])
                writer.writerow([
                    time.strftime('%Y-%m-%d %H:%M:%S'),
                    self.current_location,
                    self.scanned_aisle,
                    self.scanned_row,
                    self.scanned_rack,
                    shelf,
                    qr,
                    db_entry['product_code'],
                    db_entry['serial_number'],
                    db_entry['category_number']
                ])
        except Exception as e:
            self.get_logger().error(f"Failed to log correct scan to CSV: {e}")

    def log_mismatch_scan(self, qr, db_entry, shelf, exp_a, exp_row, exp_rk, exp_s):
        file_exists = os.path.exists(self.mismatch_log_path)
        message = (
            f"At position Aisle_{self.scanned_aisle[1:]}/Row_{self.scanned_row[1:]}/Rack_{self.scanned_rack[2:]}/{shelf} "
            f"this QR product ({db_entry['product_code']}) is there but it should be in position "
            f"Aisle_{exp_a[1:]}/Row_{exp_row[1:]}/Rack_{exp_rk[2:]}/{exp_s}"
        )
        try:
            with open(self.mismatch_log_path, 'a', newline='') as f:
                writer = csv.writer(f)
                if not file_exists:
                    writer.writerow([
                        'Timestamp', 'Current_Location', 'Current_Aisle', 'Current_Row', 'Current_Rack', 'Current_Shelf',
                        'QR_Code', 'Product_Code', 'Product_Serial_Number', 'Category_Number',
                        'Expected_Aisle', 'Expected_Row', 'Expected_Rack', 'Expected_Shelf', 'Message'
                    ])
                writer.writerow([
                    time.strftime('%Y-%m-%d %H:%M:%S'),
                    self.current_location,
                    self.scanned_aisle,
                    self.scanned_row,
                    self.scanned_rack,
                    shelf,
                    qr,
                    db_entry['product_code'],
                    db_entry['serial_number'],
                    db_entry['category_number'],
                    exp_a,
                    exp_row,
                    exp_rk,
                    exp_s,
                    message
                ])
        except Exception as e:
            self.get_logger().error(f"Failed to log mismatch scan to CSV: {e}")

    def log_unknown_scan(self, qr, shelf):
        file_exists = os.path.exists(self.mismatch_log_path)
        message = (
            f"At position Aisle_{self.scanned_aisle[1:]}/Row_{self.scanned_row[1:]}/Rack_{self.scanned_rack[2:]}/{shelf} "
            f"unknown QR product ({qr}) is present."
        )
        try:
            with open(self.mismatch_log_path, 'a', newline='') as f:
                writer = csv.writer(f)
                if not file_exists:
                    writer.writerow([
                        'Timestamp', 'Current_Location', 'Current_Aisle', 'Current_Row', 'Current_Rack', 'Current_Shelf',
                        'QR_Code', 'Product_Code', 'Product_Serial_Number', 'Category_Number',
                        'Expected_Aisle', 'Expected_Row', 'Expected_Rack', 'Expected_Shelf', 'Message'
                    ])
                writer.writerow([
                    time.strftime('%Y-%m-%d %H:%M:%S'),
                    self.current_location,
                    self.scanned_aisle,
                    self.scanned_row,
                    self.scanned_rack,
                    shelf,
                    qr,
                    'UNKNOWN',
                    'UNKNOWN',
                    'UNKNOWN',
                    'UNKNOWN',
                    'UNKNOWN',
                    'UNKNOWN',
                    'UNKNOWN',
                    message
                ])
        except Exception as e:
            self.get_logger().error(f"Failed to log unknown scan to CSV: {e}")

    def compile_position_csv(self, correct_scans):
        safe_loc_name = self.current_location.replace('/', '_').replace(' ', '_')
        pos_filename = f"/home/abhinav/warehouse_bot_simulation_ws/src/scanned_{safe_loc_name}.csv"
        
        self.get_logger().info(f"Compiling correct position scans to {pos_filename}...")
        try:
            with open(pos_filename, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['QR_Code', 'Product_Code', 'Product_Serial_Number', 'Category_Number', 'Shelf'])
                for scan in correct_scans:
                    writer.writerow([
                        scan['QR_Code'],
                        scan['Product_Code'],
                        scan['Product_Serial_Number'],
                        scan['Category_Number'],
                        scan['Shelf']
                    ])
        except Exception as e:
            self.get_logger().error(f"Failed to write position specific CSV: {e}")


# ===========================================================================
# Entry point
# ===========================================================================
def main(args=None):
    rclpy.init(args=args)
    node = RackScannerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt — shutting down.")
    finally:
        node.close_camera()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
