# `joy_teleop` — ROS2 Joystick Teleoperation Package

**Deployment Target**: 🔵 **Local Computer**

`joy_teleop` is a ROS2 package running on the operator's Local Computer that converts joystick / gamepad controller inputs (such as USB gamepads, PlayStation, or Xbox controllers) into:
- `geometry_msgs/msg/Twist` velocity commands for robot drive control
- `std_msgs/msg/String` JSON servo commands for camera yaw/pitch control (forwarded to the ESP32 via `esp32_wifi_bridge`)

---

## 📁 Package Structure

```text
joy_teleop/
├── joy_teleop/
│   ├── __init__.py
│   └── teleop_node.py  # Maps joystick axes/buttons → drive velocity + camera servo commands
├── launch/
│   └── teleop.launch.py # Launches joy_node (ros-humble-joy) and ps2_teleop
├── resource/
├── setup.cfg
├── setup.py
└── package.xml
```

---

## 🛠️ Prerequisites & Installation

On your **Local Computer**, install the ROS2 joystick driver package:
```bash
sudo apt update
sudo apt install -y ros-humble-joy
```

Connect your joystick via USB. Verify system recognition:
```bash
ls -l /dev/input/js*
```

---

## 🚀 Usage & Launch Instructions

### 1. Build Package
```bash
cd ~/warehouse_bot_simulation_ws
colcon build --packages-select joy_teleop
source install/setup.bash
```

### 2. Run Nodes (3 terminals)

**Terminal 1 — Joy driver** (reads physical controller):
```bash
source ~/warehouse_bot_simulation_ws/install/setup.bash
ros2 run joy joy_node
```

**Terminal 2 — Teleop node** (this package):
```bash
source ~/warehouse_bot_simulation_ws/install/setup.bash
ros2 run joy_teleop teleop_node
```

**Terminal 3 — ESP32 WiFi Bridge** (forwards servo commands to hardware):
```bash
source ~/warehouse_bot_simulation_ws/install/setup.bash
ros2 run active_vision_scanner esp32_bridge --ros-args -p esp32_ip:=<YOUR_ESP32_IP>
```

> The ESP32 bridge is required for camera servo commands to reach the hardware. The drive commands work independently of it.

---

## 🎮 Controls & Button Mapping

### Drive Control

| Input | Action | Notes |
|---|---|---|
| **Left Stick (Vertical)** | Linear velocity (`+x` forward) | Axis index `1` |
| **Right Stick (Horizontal)** | Angular velocity (`z` rotation) | Axis index `2` |
| **D-Pad Up / Down** | Increase / Decrease linear speed | 10% per press, range: 0.1–5.0 m/s |
| **D-Pad Right / Left** | Decrease / Increase angular speed | 10% per press, range: 0.2–5.0 rad/s |

### Camera Servo Control (Right-Side Face Buttons)

Each button press moves the servo by **5°**. Limits are hardware-enforced.

| Button | Index | Action | Servo Axis | Limit |
|---|---|---|---|---|
| **Y** | `4` | Pitch **UP** (tilt camera up) | Pitch − 5° | Min **20°** |
| **A** | `0` | Pitch **DOWN** (tilt camera down) | Pitch + 5° | Max **160°** |
| **B** | `1` | Yaw **RIGHT** | Yaw + 5° | Max **180°** |
| **X** | `3` | Yaw **LEFT** | Yaw − 5° | Min **0°** |

### Camera Preset Positions

| Button | Index | Action | Yaw | Pitch |
|---|---|---|---|---|
| **SELECT** | `10` | → **Parked** position (stowed) | 100° | 90° |
| **START** | `11` | → **Default Scan** position (faces rack) | 10° | 90° |

> Preset positions match the `PARKED_YAW/PITCH` and `DEFAULT_YAW/PITCH` constants defined in `active_vision_scanner/rack_scanner_node.py`.

---

## 📡 Published & Subscribed Topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/joy` | `sensor_msgs/msg/Joy` | **Subscribed** | Raw controller axes and button states |
| `/diff_cont/cmd_vel_unstamped` | `geometry_msgs/msg/Twist` | **Published** | Robot drive velocity commands |
| `/camera_servo_cmd` | `std_msgs/msg/String` | **Published** | JSON servo commands → `esp32_wifi_bridge` |

### Servo Command JSON Format

```json
{"cmd": "MOVE", "pitch": 90, "yaw": 100, "speed": 20}
```

---

## 🔧 Adjusting Button Indices

Controller button indices vary by manufacturer. To find your controller's indices, run:

```bash
ros2 topic echo /joy
```

Press each button and observe which index in the `buttons: [...]` array changes from `0` to `1`. Then update the corresponding variables in `teleop_node.py`:

```python
self.btn_A      = 0    # Pitch DOWN
self.btn_B      = 1    # Yaw RIGHT
self.btn_X      = 3    # Yaw LEFT
self.btn_Y      = 4    # Pitch UP
self.btn_SELECT = 10   # Go to PARKED position
self.btn_START  = 11   # Go to DEFAULT SCAN position
```
