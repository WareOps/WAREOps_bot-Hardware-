# `diffdrive_arduino` — ROS2 Control Hardware Interface for Arduino

**Deployment Target**: 🔴 **Raspberry Pi**

`diffdrive_arduino` is a C++ `ros2_control` hardware interface plugin designed to bridge ROS2 controllers (`diff_drive_controller`, `joint_state_broadcaster`) with an Arduino Nano running the `ROSArduinoBridge` motor controller firmware.

---

## ⚙️ How It Works

```text
[ROS2 Twist Command]
        │
        ▼
[diff_drive_controller]
        │
        ▼
[diffdrive_arduino (Hardware Interface Plugin)]
        │ (Uses serial library over /dev/ttyACM0 @ 57600 baud)
        ▼
[Arduino Nano (ROSArduinoBridge Firmware)]
        │
        ▼
[Cytron MDD10A Driver + TTN25 Encoder Motors]
```

1. **Read Loop**: Sends `e\r` over serial to query left and right wheel encoder counts from Arduino Nano. Calculates wheel position (rad) and velocity (rad/s) and updates ROS state.
2. **Write Loop**: Converts target wheel angular velocities into motor PWM counts and sends `m <left_speed> <right_speed>\r` to Arduino Nano.

---

## 📁 Package Structure

```text
diffdrive_arduino/
├── include/diffdrive_arduino/
│   ├── arduino_comms.hpp     # Serial communication wrapper
│   ├── diffdrive_robot.hpp     # SystemInterface hardware implementation
│   ├── fake_robot.hpp          # Simulated hardware interface for testing
│   └── wheel.hpp               # Wheel state data structure
├── src/
│   ├── arduino_comms.cpp
│   ├── diffdrive_robot.cpp
│   ├── fake_robot.cpp
│   └── wheel.cpp
├── launch/
│   ├── fake_robot.launch.py    # Launch script using fake hardware
│   └── test_robot.launch.py    # Launch script using physical Arduino hardware
├── robot_hardware.xml          # Pluginlib export manifest for physical robot
├── fake_robot_hardware.xml     # Pluginlib export manifest for fake robot
├── CMakeLists.txt
└── package.xml
```

---

## 🛠️ Requirements & Dependencies

- **Hardware**: Raspberry Pi 4, Arduino Nano (flashed with `ROSArduinoBridge` firmware), USB-A to USB-B Mini cable.
- **Dependencies**:
  - `ros2_control`
  - `hardware_interface`
  - `pluginlib`
  - `serial` (cloned in workspace)

---

## 🚀 Usage Instructions

### 1. Build Package
```bash
cd ~/ros2_ws
colcon build --packages-select diffdrive_arduino
source install/setup.bash
```

### 2. Standalone Testing (Physical Hardware)
To test serial connection to Arduino on `/dev/ttyACM0` at `57600` baud:
```bash
ros2 launch diffdrive_arduino test_robot.launch.py
```

### 3. Standalone Testing (Simulated Hardware)
To verify ros2_control controller loading without physical hardware attached:
```bash
ros2 launch diffdrive_arduino fake_robot.launch.py
```

---

## ⚙️ Integration with URDF (`ros2_control.xacro`)

This package is registered via `pluginlib` and configured inside `pi_bot/urdf/ros2_control.xacro`:

```xml
<ros2_control name="RealRobot" type="system">
    <hardware>
        <plugin>diffdrive_arduino/DiffDriveArduino</plugin>
        <param name="left_wheel_name">left_wheel_joint</param>
        <param name="right_wheel_name">right_wheel_joint</param>
        <param name="loop_rate">30</param>
        <param name="device">/dev/ttyACM0</param>
        <param name="baud_rate">57600</param>
        <param name="timeout">1000</param>
        <param name="enc_counts_per_rev">1920</param>
    </hardware>
    ...
</ros2_control>
```