# `joy_teleop` — ROS2 Joystick Teleoperation Package

**Deployment Target**: 🔵 **Local Computer**

`joy_teleop` is a ROS2 package running on the operator's Local Computer that converts joystick / gamepad controller inputs (such as USB gamepads, PlayStation, or Xbox controllers) into ROS2 velocity commands (`geometry_msgs/msg/Twist`) published on `/cmd_vel`.

---

## 📁 Package Structure

```text
joy_teleop/
├── joy_teleop/
│   ├── __init__.py
│   └── teleop_node.py  # Python node mapping joystick axes/buttons to linear & angular velocities
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
cd ~/ros2_ws
colcon build --packages-select joy_teleop
source install/setup.bash
```

### 2. Launch Joystick Teleoperation
```bash
ros2 launch joy_teleop teleop.launch.py
```

This single launch file starts:
1. `joy_node`: Reads raw input from `/dev/input/js0` and publishes `sensor_msgs/msg/Joy` messages on `/joy`.
2. `ps2_teleop` (`teleop_node.py`): Listens to `/joy` and publishes `geometry_msgs/msg/Twist` velocity commands to `/cmd_vel`.

---

## 🎮 Controls & Button Mapping

| Axis / Button | Action | Description |
|---|---|---|
| **Left Joystick (Vertical)** | Linear Velocity ($x$) | Push Forward for forward speed, Push Back for reverse |
| **Right Joystick (Horizontal)** | Angular Velocity ($z$) | Push Left/Right to turn left/right |
| **Deadman Switch / Enable Button** | Safety Override | Must be held down while moving joystick to transmit velocities |

---

## 📡 Published & Subscribed Topics

| Topic | Type | Node | Description |
|---|---|---|---|
| `/joy` | `sensor_msgs/msg/Joy` | Subscribed by `teleop_node` | Raw controller axes and button state |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Published by `teleop_node` | Robot linear and angular command velocities |
