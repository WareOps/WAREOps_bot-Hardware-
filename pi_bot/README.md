# `pi_bot` — ROS2 Robot Bringup & Navigation Package

**Deployment Target**: 🔴 **Raspberry Pi**

`pi_bot` is the core ROS2 bringup package installed directly on the Raspberry Pi 4 onboard the robot. It contains the robot's URDF description, `ros2_control` configuration, LiDAR integration, SLAM Toolbox mapping parameters, and Navigation2 launch setup.

---

## 📁 Package Structure

```text
pi_bot/
├── config/
│   ├── my_controllers.yaml  # ros2_control diff_drive_controller & joint_state_broadcaster
│   ├── param_nav2.yaml      # Navigation2 stack parameters (AMCL, Planner, Controller)
│   ├── Slam_param.yaml      # SLAM Toolbox online async mapping parameters
│   └── twist_mux.yaml       # Velocity multiplexer priority setup
├── launch/
│   ├── launch_robot.launch.py # Main hardware bringup (robot_state_publisher + controller_manager)
│   ├── navigation.launch.py   # Nav2 navigation stack launch
│   ├── rplidar.launch.py      # RPLidar A1M8 node launch
│   ├── rsp.launch.py          # Robot State Publisher standalone launch
│   └── slam_map.launch.py     # SLAM Toolbox mapping launch
├── urdf/
│   ├── robot_base.urdf.xacro  # Top-level URDF xacro entry point
│   ├── base_mobile.xacro     # Chassis geometry, wheel links, and caster wheels
│   ├── ros2_control.xacro    # Hardware interface integration (diffdrive_arduino)
│   └── common_properties.xacro # Inertia macros and color materials
├── CMakeLists.txt
└── package.xml
```

---

## 🛠️ Dependencies

Ensure the following packages are installed on your Raspberry Pi:
```bash
sudo apt update
sudo apt install -y \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-robot-state-publisher \
  ros-humble-xacro \
  ros-humble-rplidar-ros \
  ros-humble-slam-toolbox \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup
```

You must also have `serial` and `diffdrive_arduino` cloned and built in the same ROS2 workspace on the Raspberry Pi.

---

## 🚀 Usage & Launch Instructions

### 1. Bring Up Robot Hardware & Controllers
Converts xacro to URDF, starts `robot_state_publisher`, initializes `controller_manager`, and loads `diff_cont` and `joint_broad`:
```bash
ros2 launch pi_bot launch_robot.launch.py
```

### 2. Launch RPLidar A1M8
Publishes 2D laser scan data to `/scan` topic over USB serial (`/dev/ttyUSB0` or `/dev/ttyACM0`):
```bash
ros2 launch pi_bot rplidar.launch.py
```

### 3. Launch SLAM Toolbox (Mapping Mode)
Launches 2D mapping mode to generate a map of the environment:
```bash
ros2 launch pi_bot slam_map.launch.py
```

### 4. Launch Navigation2 Stack
Launches path planning, costmaps, and obstacle avoidance using pre-configured parameters. Automatically includes delayed execution (8s) of `/home/abhinav/warehouse_bot_simulation_ws/src/initialise.py` to set initial pose once Nav2 lifecycle nodes are active:
```bash
ros2 launch pi_bot navigation.launch.py map:=/path/to/my_map.yaml
```

---

## 📡 Published & Subscribed Topics

| Topic | Type | Description |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Input velocity command for wheel motion |
| `/diff_cont/cmd_vel_unstamped` | `geometry_msgs/msg/Twist` | Unstamped velocity command sent to controller |
| `/scan` | `sensor_msgs/msg/LaserScan` | 2D LiDAR range measurements |
| `/tf` / `/tf_static` | `tf2_msgs/msg/TFMessage` | Robot coordinate frame transforms (`base_link` -> `laser_frame`, etc.) |
| `/odom` | `nav_msgs/msg/Odometry` | Wheel odometry computed by `diffdrive_arduino` |
