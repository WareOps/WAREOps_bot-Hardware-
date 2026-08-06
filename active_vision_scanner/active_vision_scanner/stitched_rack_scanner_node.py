#!/usr/bin/env python3
"""
Stitched Active Vision Rack Scanner Node
========================================

Captures camera frames across pitch and yaw angles during a scanning sweep,
stitches all captured frames into a unified high-resolution panoramic image of the rack,
and establishes a global coordinate system across the entire stitched view.

In this unified panoramic view:
- The full rack boundary [Y_min, Y_max] is identified.
- The height is divided into N equal horizontal shelf bands (Shelf 1 to N).
- All QR codes are detected across the stitched panorama and unambiguously mapped
  to their exact shelf based on their global vertical coordinate (C_y).
- The final stitched panorama with shelf dividers and QR bounding boxes is saved to disk.

Usage:
    ros2 run active_vision_scanner stitched_rack_scanner
"""

import os
import sys
import time
import math
import json
import csv
import urllib.request
import threading
import numpy as np
import cv2
import openpyxl

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# ===============================================================================
# Constants & Defaults
# ===============================================================================
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

DEFAULT_NUM_SHELVES = 4
DEFAULT_RACK_HEIGHT_CM = 56.0
DEFAULT_RACK_HORIZONTAL_CM = 52.0
DEFAULT_CAMERA_HEIGHT_CM = 24.5
CAMERA_DISTANCE_CM = 45.0

PARKED_YAW = 100.0
PARKED_PITCH = 90.0
DEFAULT_YAW = 10.0
DEFAULT_PITCH = 60.0

RACK_OVERRIDES = {
    "Aisle_2/Row_1/Rack_1": {
        "num_shelves": 3,
        "rack_height": 42.0,
        "rack_horizontal": 52.0,
    },
    "Aisle_2/Row_1/Rack_2": {
        "num_shelves": 3,
        "rack_height": 42.0,
        "rack_horizontal": 52.0,
    },
}


class ScannerState:
    IDLE = "IDLE"
    MOVING_TO_SCAN = "MOVING_TO_SCAN"
    SCANNING = "SCANNING"
    STITCHING = "STITCHING"
    MOVING_TO_PARK = "MOVING_TO_PARK"
    SHUTDOWN = "SHUTDOWN"


class StitchedRackScannerNode(Node):
    """
    ROS2 Node that scans warehouse racks by capturing overlapping camera frames,
    stitching them into a single panoramic rack image, and using the global coordinate
    system to map detected QR codes to their respective shelves.
    """

    def __init__(self):
        super().__init__('stitched_rack_scanner_node')

        # Parameters
        self.declare_parameter('cam_url', 'http://10.225.34.222:81/stream')
        self.declare_parameter('camera_height_cm', DEFAULT_CAMERA_HEIGHT_CM)
        self.declare_parameter('rack_height_cm', DEFAULT_RACK_HEIGHT_CM)
        self.declare_parameter('rack_horizontal_cm', DEFAULT_RACK_HORIZONTAL_CM)
        self.declare_parameter('num_shelves', DEFAULT_NUM_SHELVES)
        self.declare_parameter('db_path', '/home/abhinav/warehouse_bot_simulation_ws/inventory_database.xlsx')
        self.declare_parameter('scanned_inv_path', '/home/abhinav/warehouse_bot_simulation_ws/scanned_inventory.csv')

        self.cam_url = self.get_parameter('cam_url').value
        self.camera_height = self.get_parameter('camera_height_cm').value
        self.rack_height = self.get_parameter('rack_height_cm').value
        self.rack_horizontal = self.get_parameter('rack_horizontal_cm').value
        self.num_shelves = self.get_parameter('num_shelves').value
        self.db_path = self.get_parameter('db_path').value
        self.scanned_inv_path = self.get_parameter('scanned_inv_path').value

        # ------- Publishers -------
        self.servo_pub = self.create_publisher(String, '/camera_servo_cmd', 10)
        self.bot_status_pub = self.create_publisher(String, '/bot_status', 10)
        self.camera_system_pub = self.create_publisher(String, '/camera_system', 10)

        # ------- Subscribers -------
        self.bot_status_sub = self.create_subscription(
            String, '/bot_status', self.bot_status_callback, 10
        )
        self.camera_system_sub = self.create_subscription(
            String, '/camera_system', self.camera_system_callback, 10
        )

        # ------- Internal State -------
        self.state = ScannerState.IDLE
        self.current_yaw = PARKED_YAW
        self.current_pitch = PARKED_PITCH
        self.current_location = ""
        self.scanned_aisle = ""
        self.scanned_row = ""
        self.scanned_rack = ""
        self.scan_results = []
        self.stream = None
        self.stream_bytes = b''
        self.latest_color_frame = None

        # Active parameters
        self.active_num_shelves = self.num_shelves
        self.active_rack_height = self.rack_height
        self.active_rack_horizontal = self.rack_horizontal

        # QR Detector
        try:
            self.qr_detector = cv2.wechat_qrcode_WeChatQRCode()
            self.get_logger().info("Using WeChatQRCode detector.")
        except Exception:
            self.qr_detector = cv2.QRCodeDetector()
            self.get_logger().warn("WeChatQRCode unavailable, falling back to QRCodeDetector.")

        # Database
        self.database = {}
        self.load_database()

        # Processing loop timer (10 Hz)
        self.create_timer(0.1, self.process_loop)

        self.get_logger().info("Stitched Rack Scanner Node initialized.")

    # ===========================================================================
    # Callbacks & DB Loader
    # ===========================================================================
    def load_database(self):
        """Load inventory database from Excel file."""
        if not os.path.exists(self.db_path):
            self.get_logger().error(f"Database file not found at: {self.db_path}")
            return

        try:
            wb = openpyxl.load_workbook(self.db_path)
            sheet = wb.active
            for r in range(2, sheet.max_row + 1):
                qr_val = sheet.cell(r, 1).value
                if qr_val:
                    qr_str = str(qr_val).strip()
                    self.database[qr_str] = {
                        'product_code': sheet.cell(r, 2).value,
                        'serial_number': sheet.cell(r, 3).value,
                        'category_number': sheet.cell(r, 4).value,
                        'aisle': str(sheet.cell(r, 5).value).strip() if sheet.cell(r, 5).value else '',
                        'row': str(sheet.cell(r, 6).value).strip() if sheet.cell(r, 6).value else '',
                        'rack': str(sheet.cell(r, 7).value).strip() if sheet.cell(r, 7).value else '',
                        'shelf': str(sheet.cell(r, 8).value).strip() if sheet.cell(r, 8).value else '',
                    }
            self.get_logger().info(f"Loaded {len(self.database)} inventory items from database.")
        except Exception as e:
            self.get_logger().error(f"Error loading database: {e}")

    def bot_status_callback(self, msg: String):
        """Handle status commands from bot navigation."""
        text = msg.data.strip()
        if text.startswith("Reached "):
            location = text.replace("Reached ", "").strip()
            self.current_location = location
            self._parse_location(location)
            self._apply_rack_overrides(location)
            self.get_logger().info(f"Received position trigger: '{location}'. Initiating panoramic scan.")
            self.state = ScannerState.MOVING_TO_SCAN

    def camera_system_callback(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd == "stop":
            self.get_logger().info("Received STOP command. Moving to SHUTDOWN state.")
            self.state = ScannerState.SHUTDOWN

    def _parse_location(self, location: str):
        parts = location.split('/')
        if len(parts) >= 3:
            self.scanned_aisle = parts[0]
            self.scanned_row = parts[1]
            self.scanned_rack = parts[2]

    def _apply_rack_overrides(self, location: str):
        if location in RACK_OVERRIDES:
            ovr = RACK_OVERRIDES[location]
            self.active_num_shelves = ovr["num_shelves"]
            self.active_rack_height = ovr["rack_height"]
            self.active_rack_horizontal = ovr["rack_horizontal"]
            self.get_logger().info(f"Applied rack overrides for {location}: {self.active_num_shelves} shelves")
        else:
            self.active_num_shelves = self.num_shelves
            self.active_rack_height = self.rack_height
            self.active_rack_horizontal = self.rack_horizontal

    def send_servo_command(self, pitch: float, yaw: float):
        pitch = max(0.0, min(90.0, float(pitch)))
        yaw = max(0.0, min(180.0, float(yaw)))
        cmd_dict = {"pitch": round(pitch, 1), "yaw": round(yaw, 1)}
        msg = String()
        msg.data = json.dumps(cmd_dict)
        self.servo_pub.publish(msg)
        self.current_pitch = pitch
        self.current_yaw = yaw

    def open_camera(self) -> bool:
        try:
            self.stream = urllib.request.urlopen(self.cam_url, timeout=10)
            self.stream_bytes = b''
            self.get_logger().info("Camera stream opened.")
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to open camera stream: {e}")
            self.stream = None
            return False

    def close_camera(self):
        if hasattr(self, 'stream') and self.stream is not None:
            self.stream.close()
            self.stream = None

    def grab_frame(self):
        """Grab and sharpen a single frame from the camera stream."""
        if not hasattr(self, 'stream') or self.stream is None:
            if not self.open_camera():
                return False, None

        try:
            while True:
                self.stream_bytes += self.stream.read(2048)
                a = self.stream_bytes.find(b'\xff\xd8')
                b = self.stream_bytes.find(b'\xff\xd9')
                if a != -1 and b != -1:
                    jpg = self.stream_bytes[a:b+2]
                    self.stream_bytes = self.stream_bytes[b+2:]
                    frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                    if frame is not None:
                        frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))
                        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                        enhanced_gray = self.enhance_frame(gray)
                        self.latest_color_frame = cv2.cvtColor(enhanced_gray, cv2.COLOR_GRAY2BGR)
                        return True, enhanced_gray
        except Exception as e:
            self.get_logger().warn(f"Failed to grab frame ({e}). Reconnecting...")
            self.close_camera()
            if self.open_camera():
                return self.grab_frame()
            return False, None
        return False, None

    def enhance_frame(self, frame):
        """Apply CLAHE & Unsharp Masking for image sharpening."""
        if frame is None:
            return None
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if len(frame.shape) == 3 else frame.copy()
        clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8))
        equalized = clahe.apply(gray)
        blurred = cv2.GaussianBlur(equalized, (0, 0), sigmaX=2.0)
        sharpened = cv2.addWeighted(equalized, 1.8, blurred, -0.8, 0)
        return sharpened

    # ===========================================================================
    # Frame Stitching & Panoramic QR Detection Engine
    # ===========================================================================
    def stitch_captured_frames(self, captured_frames: list) -> np.ndarray:
        """
        Stitch a list of overlapping captured frames into a single panorama image.
        Uses OpenCV's Stitcher class with fallback options.
        """
        if not captured_frames:
            return None

        self.get_logger().info(f"Stitching {len(captured_frames)} overlapping frames into panorama...")

        # Convert grayscale frames to 3-channel BGR for OpenCV Stitcher
        bgr_frames = []
        for img in captured_frames:
            if len(img.shape) == 2:
                bgr_frames.append(cv2.cvtColor(img, cv2.COLOR_GRAY2BGR))
            else:
                bgr_frames.append(img)

        # Attempt 1: OpenCV Stitcher PANORAMA
        stitcher = cv2.Stitcher_create(cv2.Stitcher_PANORAMA)
        status, stitched = stitcher.stitch(bgr_frames)

        if status == cv2.Stitcher_OK:
            self.get_logger().info("Panorama stitching succeeded via OpenCV Stitcher!")
            return stitched

        self.get_logger().warn(f"OpenCV Stitcher returned code {status}. Using Grid Homography Fallback...")

        # Fallback 2: Simple Grid Montage / Stitched Blend if feature matching fails
        return self._grid_stitch_fallback(bgr_frames)

    def _grid_stitch_fallback(self, bgr_frames: list) -> np.ndarray:
        """
        Fallback grid stitching when feature points are insufficient.
        Blends frames into a structured panorama matrix.
        """
        n = len(bgr_frames)
        cols = min(n, 5)
        rows = math.ceil(n / cols)

        h, w = bgr_frames[0].shape[:2]
        panorama = np.zeros((rows * h, cols * w, 3), dtype=np.uint8)

        for idx, img in enumerate(bgr_frames):
            r = idx // cols
            c = idx % cols
            panorama[r * h:(r + 1) * h, c * w:(c + 1) * w] = img

        return panorama

    def process_panorama(self, panorama: np.ndarray) -> tuple:
        """
        Detect QR codes on the stitched panorama and map them to global shelf coordinates.
        Divides vertical height [Y_min, Y_max] into N equal shelf bands.

        Returns (annotated_panorama, detections_list).
        """
        if panorama is None:
            return None, []

        h, w = panorama.shape[:2]
        n = self.active_num_shelves

        # Define global rack bounds on the panorama (default full vertical height)
        y_min = int(0.05 * h)
        y_max = int(0.95 * h)
        shelf_height = (y_max - y_min) / float(n)

        annotated = panorama.copy()

        # 1. Draw horizontal shelf lines & labels on stitched view
        for i in range(1, n):
            div_y = int(y_min + i * shelf_height)
            cv2.line(annotated, (10, div_y), (w - 10, div_y), (255, 255, 0), 2, cv2.LINE_AA)
            cv2.putText(annotated, f"SHELF {i} / {i+1} BOUNDARY", (20, div_y - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

        # 2. Detect QR codes on full panorama
        detections = []
        gray_pano = cv2.cvtColor(panorama, cv2.COLOR_BGR2GRAY) if len(panorama.shape) == 3 else panorama

        # Multi-pass detection on panorama
        candidates = [gray_pano]
        try:
            thresh = cv2.adaptiveThreshold(gray_pano, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 15, 3)
            candidates.append(thresh)
        except Exception:
            pass

        seen_qrs = set()
        for img in candidates:
            if isinstance(self.qr_detector, cv2.wechat_qrcode_WeChatQRCode):
                decoded_list, points_list = self.qr_detector.detectAndDecode(img)
                if decoded_list:
                    for i, data in enumerate(decoded_list):
                        data = data.strip()
                        if data and data not in seen_qrs:
                            seen_qrs.add(data)
                            pts = points_list[i] if points_list and i < len(points_list) else None
                            cx = float(np.mean(pts[:, 0])) if pts is not None else w / 2.0
                            cy = float(np.mean(pts[:, 1])) if pts is not None else h / 2.0

                            # Determine global shelf from panorama Cy coordinate
                            rel_y = cy - y_min
                            shelf_num = int(rel_y // shelf_height) + 1
                            shelf_num = max(1, min(n, shelf_num))

                            detections.append({
                                'raw_data': data,
                                'product_info': self._parse_qr_data(data),
                                'center_x': cx,
                                'center_y': cy,
                                'global_shelf': shelf_num,
                                'pts': pts
                            })
            else:
                try:
                    retval, decoded_info, points, _ = self.qr_detector.detectAndDecodeMulti(img)
                    if retval and decoded_info is not None:
                        for i, data in enumerate(decoded_info):
                            data = str(data).strip()
                            if data and data not in seen_qrs:
                                seen_qrs.add(data)
                                bbox = points[i]
                                cx = float(np.mean(bbox[:, 0]))
                                cy = float(np.mean(bbox[:, 1]))

                                rel_y = cy - y_min
                                shelf_num = int(rel_y // shelf_height) + 1
                                shelf_num = max(1, min(n, shelf_num))

                                detections.append({
                                    'raw_data': data,
                                    'product_info': self._parse_qr_data(data),
                                    'center_x': cx,
                                    'center_y': cy,
                                    'global_shelf': shelf_num,
                                    'pts': bbox
                                })
                except Exception:
                    pass

        # 3. Draw detected QR codes on annotated panorama
        for det in detections:
            cx, cy = int(det['center_x']), int(det['center_y'])
            shelf_num = det['global_shelf']
            data = det['raw_data']

            cv2.circle(annotated, (cx, cy), 8, (0, 255, 0), -1)
            cv2.putText(annotated, f"S{shelf_num}: {data}", (cx - 50, max(30, cy - 15)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            if det['pts'] is not None:
                pts = np.int32(det['pts'])
                cv2.polylines(annotated, [pts], True, (0, 255, 0), 3)

        return annotated, detections

    def _parse_qr_data(self, data: str) -> dict:
        try:
            return json.loads(data)
        except Exception:
            return {'raw': data}

    # ===========================================================================
    # Execution & Scan Pipeline
    # ===========================================================================
    def run_stitched_scan(self) -> list:
        """
        Execute scanning sweep, capture overlapping frames, stitch into a panorama,
        and perform global coordinate shelf mapping for all detected QR codes.
        """
        center_yaw = self.current_yaw
        captured_frames = []

        self.get_logger().info(f"Starting Panoramic Stitching Scan (center_yaw={center_yaw}°)...")

        pitch_steps = [40.0, 65.0, 90.0]
        yaw_offsets = [0, -25, 25, -40, 40]

        # 1. Sweep camera and collect overlapping frames
        for step_idx, target_pitch in enumerate(pitch_steps):
            for offset in yaw_offsets:
                pan_yaw = max(0, min(180, center_yaw + offset))
                self.send_servo_command(target_pitch, pan_yaw)
                time.sleep(0.4)

                ok, frame = self.grab_frame()
                if ok and frame is not None:
                    captured_frames.append(frame)

            # Reset yaw
            if self.current_yaw != center_yaw:
                self.send_servo_command(target_pitch, center_yaw)
                time.sleep(0.3)

        # 2. Stitch frames into unified panoramic image
        panorama = self.stitch_captured_frames(captured_frames)

        if panorama is None:
            self.get_logger().error("Panorama creation failed.")
            return []

        # 3. Process panorama & map QRs to global shelf coordinates
        annotated_pano, detections = self.process_panorama(panorama)

        # Save stitched panorama image to disk
        out_filename = f"stitched_rack_{self.scanned_aisle}_{self.scanned_row}_{self.scanned_rack}.jpg"
        out_path = os.path.join('/home/abhinav/warehouse_bot_simulation_ws', out_filename)
        cv2.imwrite(out_path, annotated_pano)
        self.get_logger().info(f"Saved stitched panoramic scan result to: {out_path}")

        # Display debug window
        cv2.imshow("Stitched Panoramic Rack Scan", cv2.resize(annotated_pano, (960, 720)))
        cv2.waitKey(1000)

        return detections

    def process_loop(self):
        """ROS2 timer loop state machine."""
        if self.state == ScannerState.SHUTDOWN:
            self.close_camera()
            rclpy.shutdown()
            return

        if self.state == ScannerState.IDLE:
            return

        if self.state == ScannerState.MOVING_TO_SCAN:
            self.send_servo_command(DEFAULT_PITCH, DEFAULT_YAW)
            time.sleep(0.8)
            if not self.open_camera():
                self.state = ScannerState.IDLE
                return
            self.state = ScannerState.SCANNING

        elif self.state == ScannerState.SCANNING:
            self.scan_results = self.run_stitched_scan()
            self.process_scan_results(self.scan_results)

            result_msg = String()
            result_msg.data = f"Succeeded {self.current_location}"
            self.bot_status_pub.publish(result_msg)

            self.state = ScannerState.MOVING_TO_PARK

        elif self.state == ScannerState.MOVING_TO_PARK:
            self.send_servo_command(PARKED_PITCH, PARKED_YAW)
            time.sleep(0.8)
            self.close_camera()
            self.state = ScannerState.IDLE

    def process_scan_results(self, results):
        self.get_logger().info("Processing global panorama scan results against database...")
        for r in results:
            qr_data = r['raw_data']
            shelf_num = r['global_shelf']
            shelf_code = f"S{shelf_num}"

            if qr_data in self.database:
                db_entry = self.database[qr_data]
                is_correct = (
                    db_entry['aisle'] == self.scanned_aisle and
                    db_entry['row'] == self.scanned_row and
                    db_entry['rack'] == self.scanned_rack and
                    db_entry['shelf'] == shelf_code
                )

                if is_correct:
                    self.get_logger().info(f"MATCH: '{qr_data}' correctly located on {self.current_location}/{shelf_code}")
                else:
                    self.get_logger().warn(f"MISMATCH: '{qr_data}' detected on {shelf_code}, expected {db_entry['shelf']}")
            else:
                self.get_logger().error(f"UNKNOWN QR: '{qr_data}' detected on {shelf_code}")


def main(args=None):
    rclpy.init(args=args)
    node = StitchedRackScannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
