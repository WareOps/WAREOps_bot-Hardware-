# Active Vision Rack Scanning System — Overview

## Network Architecture

All devices connect to the **same mobile hotspot**:

```
+-----------------------------------------------------------------------+
|                         📱 MOBILE HOTSPOT                             |
|                        (WiFi: 192.168.43.x)                           |
+-----------------------------------+-----------------------------------+
                                    |
          +-------------------------+-------------------------+
          |                                                   |
          v                                                   v
+-------------------+                               +-------------------+
|    📷 ESP32-CAM   |                               | 🎮 ESP32 DEV KIT  |
| Video Stream      |                               | HTTP Server (:80) |
| :81/stream        |                               | PCA9685 Servos    |
+---------^---------+                               +---------^---------+
          |                                                   |
          | HTTP GET (MJPEG)                                  | HTTP POST (/cmd)
          |                                                   |
+---------+---------------------------------------------------+---------+
|                                💻 LAPTOP                              |
|                                                                       |
|   [rack_scanner_node.py]                       [esp32_wifi_bridge.py] |
|   (OpenCV + State Machine)                      (ROS2 <-> HTTP Bridge)|
+-----------------------------------------------------------------------+
```

---

## Files & Data Architecture

### Core Files

| File | Purpose |
|------|---------|
| [scanner_system.launch.py](active_vision_scanner/launch/scanner_system.launch.py) | ROS2 Launch file — launches both bridge & scanner with custom parameters |
| [rack_scanner_node.py](active_vision_scanner/active_vision_scanner/rack_scanner_node.py) | ROS2 OpenCV node — state machine, edge/pitch adjustment, QR detection, database lookup, and CSV logging |
| [esp32_serial_bridge.py](active_vision_scanner/active_vision_scanner/esp32_serial_bridge.py) | ROS2 → ESP32 WiFi HTTP bridge |
| [esp32_servo_controller.ino](active_vision_scanner/esp32_servo_controller/esp32_servo_controller.ino) | ESP32 Arduino firmware — WiFi HTTP server + PCA9685 |
| [package.xml](active_vision_scanner/package.xml) | ROS2 package manifest |
| [setup.py](active_vision_scanner/setup.py) | Python package setup |

### Database & Outputs (Workspace `src/`)

| File / Path | Type | Purpose |
|-------------|------|---------|
| [warehouse_database.xlsx](warehouse_database.xlsx) | Excel (xlsx) Input | Master database containing QR codes, product codes, serial numbers, and expected shelf positions (`Aisle`, `Row`, `Rack`, `Shelf`). |
| [scanned_inventory.csv](scanned_inventory.csv) | CSV Output | Appends all successfully verified, correctly placed product scans with timestamps. |
| `scanned_Aisle_<A>_Row_<R>_Rack_<RK>.csv` | CSV Output | A location-specific CSV compiled after each scan cycle containing only correctly placed items at that location. |
| [mismatch_log.csv](mismatch_log.csv) | CSV Output | Logs all misplaced products (incorrect shelf/rack/aisle) and completely unknown QR codes with detailed mismatch warning messages. |

---

## Physical Parameters

| Parameter | Value |
|-----------|-------|
| Camera height from ground | **13.35 cm** |
| Rack height | **49 cm** |
| Rack horizontal width | **60 cm** |
| Camera distance from rack | **40 cm** |
| Number of shelves | **4** |
| Parked position | **Yaw = 100°, Pitch = 90°** |
| Scanning / Detection position | **Yaw = 10°, Pitch = 90°** |
| Pitch tilt range | **20° (Min Tilt UP) to 160° (Max Tilt DOWN)** |

---

## ROS2 Topics

| Topic | Type | Publisher | Subscriber | Purpose |
|-------|------|-----------|------------|---------|
| `/bot_status` | `String` | Robot nav / Scanner | Scanner / Robot nav | `"Reached Aisle_1/Row_1/Rack_1"` triggers scan; `"Succeeded Aisle_1/Row_1/Rack_1"` = done |
| `/camera_servo_cmd` | `String` | Scanner | WiFi Bridge | JSON servo commands forwarded via HTTP POST |
| `/camera_system` | `String` | External | Scanner + Bridge | `"stop"` → shutdown everything |
| `/esp32_response` | `String` | WiFi Bridge | (debug) | ESP32 HTTP responses |

---

## State Machine Flow

```
 [START] ---> (IDLE) ─────────► Receive "Reached Aisle_X/Row_Y/Rack_Z" on /bot_status
                 │
                 ▼
          (MOVING_TO_SCAN) ───► Move servos to Yaw=0°, Pitch=90°
                 │
                 ▼
         (ADJUSTING_PITCH) ───► OpenCV Canny edge pitch alignment (~74.4°)
                 │
                 ▼
            (SCANNING) ───────► Phase 1: Centered Scan (10 frames)
                 │              Phase 2: Active Pitch Tilt Sweep (S1-S4)
                 │              Database Validation & CSV Export
                 ▼
          (MOVING_TO_PARK) ───► Move servos to Yaw=90°, Pitch=90°
                 │
                 ▼
               (IDLE) ────────► Publish "Succeeded..." on /bot_status
```

**Sequence per rack:**
1. Receive `"Reached Aisle_1/Row_1/Rack_1"` on `/bot_status`.
2. Parse location into Aisle `A1`, Row `R1`, Rack `RK1`.
3. Command servos to default scan orientation (yaw 90° → 0°, pitch stays 90°).
4. Auto-adjust pitch until all 4 shelves are visible (~74.4°).
5. Capture frames in 2 phases (Centered View + Active Pitch Tilt Sweep for top/bottom shelves).
6. Query each detected QR against `warehouse_database.xlsx` using `openpyxl`.
   - **Correct Placement**: Logged to `scanned_inventory.csv` and compiled into `scanned_Aisle_1_Row_1_Rack_1.csv`.
   - **Incorrect Placement / Unknown**: Logged to `mismatch_log.csv` with a descriptive message.
7. Publish `"Succeeded Reached Aisle_1/Row_1/Rack_1"` on `/bot_status`.
8. Return servos back to yaw 90°, pitch 90° (parked).

---

## CSV Logging Format

### 1. Correct Inventory Log (`scanned_inventory.csv`)
```csv
Timestamp,Scanned_Location,Aisle,Row,Rack,Shelf,QR_Code,Product_Code,Product_Serial_Number,Category_Number
2026-07-25 16:11:52,Aisle_1/Row_1/Rack_1,A1,R1,RK1,S1,%U0fd,WH-A1-R1-RK1-S1-P1,SN-00001,CAT-01
```

### 2. Mismatch Log (`mismatch_log.csv`)
```csv
Timestamp,Current_Location,Current_Aisle,Current_Row,Current_Rack,Current_Shelf,QR_Code,Product_Code,Product_Serial_Number,Category_Number,Expected_Aisle,Expected_Row,Expected_Rack,Expected_Shelf,Message
2026-07-25 16:11:52,Aisle_1/Row_1/Rack_1,A1,R1,RK1,S1,i#S20,WH-A1-R1-RK1-S2-P1,SN-00004,CAT-02,A1,R1,RK1,S2,At position Aisle_1/Row_1/Rack_1/S1 this QR product (WH-A1-R1-RK1-S2-P1) is there but it should be in position Aisle_1/Row_1/Rack_1/S2
```

---

## ESP32 Servo Trajectory Planning

The ESP32 **never jumps** to a target angle. Both servos move simultaneously:

```
Yaw:   90° → 89° → 88° → ... → 1° → 0°    (+1/-1° per step)
Pitch: 90° → 90° (no change needed)          (stays put)

Each step: 15ms delay (configurable via "speed" in JSON)
```

---

## How to Run

### 1. Launch Everything via Single Launch File (Recommended)

```bash
# Build and source workspace
cd ~/warehouse_bot_simulation_ws
colcon build --packages-select active_vision_scanner
source install/setup.bash

# Launch both bridge and scanner with your target IP address & stream URL:
ros2 launch active_vision_scanner scanner_system.launch.py \
    esp32_ip:=192.168.43.XXX \
    esp32_cam_url:=http://192.168.43.YYY:81/stream
```

### 2. Manual Terminal Launch (Alternative)

```bash
# Terminal 1: Run the WiFi bridge
ros2 run active_vision_scanner esp32_bridge \
    --ros-args -p esp32_ip:=192.168.43.XXX

# Terminal 2: Run the scanner node
ros2 run active_vision_scanner rack_scanner \
    --ros-args -p esp32_cam_url:=http://192.168.43.YYY:81/stream

# Terminal 3: Test — simulate bot reaching a rack
ros2 topic pub --once /bot_status std_msgs/msg/String \
    "data: 'Reached Aisle_1/Row_1/Rack_1'"
```

> [!NOTE]
> Python environment NumPy mismatch has been resolved by pinning `numpy<2` (installed 1.26.4). This ensures full compatibility with OpenCV and Pandas.
