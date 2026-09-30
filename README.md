# Warehouse Bot & Active Vision System (ROS2 Humble)

An end-to-end Autonomous Mobile Robot (AMR) system for warehouse navigation, mapping, and active vision rack inventory scanning developed using **ROS2 Humble**.

---

##  Video Demonstrations & Project Demos

[![YouTube Playlist](https://img.shields.io/badge/YouTube-Watch%20Full%20Project%20Playlist-FF0000?style=for-the-badge&logo=youtube&logoColor=white)](https://youtube.com/playlist?list=PLdkPHi9wHG-M&si=vZzbjLmxS52RyPB2)

*  **[Watch Complete Project Demonstration Playlist on YouTube](https://youtube.com/playlist?list=PLdkPHi9wHG-M&si=vZzbjLmxS52RyPB2)** — Includes full autonomous navigation tours, SLAM mapping, active vision pan-tilt QR scanning, and hardware testing demonstrations.

---

##  System Architecture & Package Deployment Split

The repository is structured for a multi-device deployment between the **onboard Raspberry Pi 4** (robot hardware control & navigation) and the **Local Computer** (operator station, active vision processing, teleoperation, and visualization).

```text
               +---------------------------------------------------+
               |             Wi-Fi / Network Subnet                |
               +-------------------------+-------------------------+
                                         |
            +----------------------------+----------------------------+
            |                                                         |
            v                                                         v
+-----------------------+                                 +-----------------------+
|   RASPBERRY PI 4    |                                 |   LOCAL COMPUTER    |
| (Robot Controller)    |                                 | (Operator Station)    |
+-----------------------+                                 +-----------------------+
| • pi_bot              |                                 | • active_vision_scanner|
| • serial              |                                 | • joy_teleop          |
| • diffdrive_arduino   |                                 | • rviz                |
+-----------+-----------+                                 | • Motors              |
            |                                             +-----------------------+
            v                                                         |
+-----------------------+                                             |
|  Hardware           |                                             v
| • Arduino Nano        |<============================================+
| • RPLidar A1M8        |   (ESP32-CAM MJPEG Stream & WiFi Servo HTTP)
| • Motor Drivers       |
+-----------------------+
```

---

##  Package Distribution Guide

###  Packages to clone on **Raspberry Pi** (Robot Hardware & Navigation)

| Package | Description |
|---|---|
| **`pi_bot`** | Core robot ROS2 bringup, URDF/Xacro models, RPLidar launch, SLAM Toolbox configurations, and Navigation2 launch stack. |
| **`serial`** | C++ serial library required by `diffdrive_arduino` to communicate over USB with the Arduino motor controller. |
| **`diffdrive_arduino`** | `ros2_control` hardware interface C++ plugin interfacing with Arduino Nano for differential drive wheel odometry and motor commands. |

###  Packages to clone on **Local Computer** (Workstation & Vision Processing)

| Package | Description |
|---|---|
| **`active_vision_scanner`** | Python ROS2 node for OpenCV active rack alignment, pan-tilt servo tracking, QR code scanner, database lookup (`warehouse_database.xlsx`), and CSV inventory logging. |
| **`joy_teleop`** | ROS2 node mapping USB joystick / gamepad input to `/cmd_vel` velocity commands for manual robot control. |
| **`rviz`** | Pre-configured RViz2 display profiles (`rviz_config.rviz`, `rviz_config_nav2.rviz`) for live map, costmap, TF, and LiDAR visualization. |
| **`Motors`** | Arduino Nano firmware (`ROSArduinoBridge`) sketch and driver headers for motor control, PID tuning, and encoder processing. |

---

##  Quick Start Guide

### 1. Prerequisites & Dependencies

Both machines must run **Ubuntu 22.04 LTS** with **ROS2 Humble** installed.

On **Raspberry Pi**:
```bash
sudo apt update
sudo apt install -y \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-slam-toolbox \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-rplidar-ros
```

On **Local Computer**:
```bash
sudo apt update
sudo apt install -y \
  ros-humble-rviz2 \
  ros-humble-joy \
  python3-opencv \
  python3-pandas \
  python3-openpyxl
pip install pyzbar "numpy<2"
```

---

### 2. Setting Up on Raspberry Pi

1. Create a ROS2 workspace and clone the Pi packages:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   # Copy or git clone pi_bot, serial, diffdrive_arduino into src/
   ```

2. Build and source:
   ```bash
   cd ~/ros2_ws
   colcon build --symlink-install
   source install/setup.bash
   ```

3. Export ROS Domain ID (must match Local PC):
   ```bash
   echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
   source ~/.bashrc
   ```

---

### 3. Setting Up on Local Computer

1. Create a ROS2 workspace and clone the Local PC packages:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   # Copy or git clone active_vision_scanner, joy_teleop, rviz, Motors into src/
   ```

2. Build and source:
   ```bash
   cd ~/ros2_ws
   colcon build --symlink-install
   source install/setup.bash
   ```

3. Export ROS Domain ID (must match Raspberry Pi):
   ```bash
   echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
   source ~/.bashrc
   ```

---

##  Running the System

### On Raspberry Pi (Robot Hardware Bringup):

```bash
# 1. Bring up robot state publisher and diffdrive hardware interface
ros2 launch pi_bot launch_robot.launch.py 

# 2. Launch SLAM Toolbox for mapping (or navigation for autonomous drive)
ros2 launch pi_bot slam_map.launch.py

# 3. Launch Nav2_bringup (navigation for autonomous drive with delayed initial pose setup)
ros2 launch pi_bot navigation.launch.py map:=/path/to/my_map.yaml

# 4. Optional: Run autonomous rack navigation script or full tour manager
python3 scripts/go_to_rack_r4_a3.py
python3 scripts/full_tour.py
```

### On Local Computer (Control & Inspection):

```bash
# 1. Run Joystick Teleop for manual navigation
ros2 launch joy_teleop teleop.launch.py

# 2. Launch RViz2 for visualization
rviz2 -d src/rviz/rviz_config_nav2.rviz

# 3. Launch Active Vision Scanner System
ros2 launch active_vision_scanner scanner_system.launch.py \
    esp32_ip:=192.168.43.XXX \
    esp32_cam_url:=http://192.168.43.YYY:81/stream
```

---

##  Repository Directory Tree

```text
src/
├── active_vision_scanner/  # [Local PC] ESP32 WiFi bridge & OpenCV QR scanner state machine
├── Chassis_step_file/      # 3D CAD step model file of the WareOps robot chassis
├── diffdrive_arduino/      # [Raspberry Pi] ros2_control C++ hardware interface plugin
├── Images/                 # High-resolution hardware and assembly gallery with README.md
├── joy_teleop/             # [Local PC] Joystick teleoperation node
├── Motors/                 # [Local PC] Arduino Nano motor firmware (ROSArduinoBridge)
├── pi_bot/                 # [Raspberry Pi] URDF, RPLidar driver, SLAM, & Nav2 launch configs
├── rviz/                   # [Local PC] RViz2 display configuration files
├── scripts/                # Autonomous navigation waypoint & full tour sequence scripts
│   ├── initialise.py       # Delayed initial pose setup script for Nav2
│   ├── full_tour.py        # Multi-rack automated inspection sequence manager
│   ├── go_to_rack_r3_b3.py # Waypoint navigation script for Rack R3_B3
│   ├── go_to_rack_r4_a2.py # Waypoint navigation script for Rack R4_A2
│   └── go_to_rack_r4_a3.py # Waypoint navigation script for Rack R4_A3
├── serial/                 # [Raspberry Pi] Serial communication C++ library
├── HARDWARE.md             # Complete hardware specification sheet
├── system_overview.md       # Detailed active vision system flow documentation
├── warehouse_database.xlsx # Master warehouse inventory Excel database
└── warehouse_database.ods  # Excel/ODS master inventory database file
```

---

##  Documentation Links
* [HARDWARE.md](HARDWARE.md): Detailed hardware connections, pinouts, and power distribution specs.
* [system_overview.md](system_overview.md): Complete breakdown of network architecture, ESP32 pan-tilt trajectory, and vision state machine flow.
