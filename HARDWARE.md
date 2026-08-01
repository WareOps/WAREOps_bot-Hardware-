# Hardware Overview

This document provides a complete overview of the hardware used in the **Warehouse Stock Navigation Robot**. The robot is designed as an autonomous mobile robot (AMR) capable of warehouse mapping, localization, navigation, live monitoring, and future inventory management.

---

# System Architecture

The robot consists of the following major subsystems:

* Computing Unit
* Navigation Sensors
* Motion System
* Vision System
* Motor Control
* Power Distribution
* Communication Interfaces

---

# Hardware Specifications

| Component                                   | Quantity | Purpose                                   |
| ------------------------------------------- | -------: | ----------------------------------------- |
| Raspberry Pi 4 (4GB RAM)                    |        1 | Main onboard computer running ROS2        |
| RPLidar A1M8 (12m, 360°)                    |        1 | Mapping, localization, obstacle detection |
| Arduino Nano                                |        1 | Motor control, encoder processing         |
| TTN25 Encoder DC Gear Motors (12V, 370 RPM) |        2 | Differential drive locomotion             |
| Cytron MDD10A Dual Channel Motor Driver     |        1 | Drives left and right motors              |
| ESP32-CAM                                   |        1 | Live video streaming                      |
| ESP32 Development Board                     |        1 | Camera gimbal controller                  |
| PCA9685 Servo Driver                        |        1 | Servo PWM generation                      |
| SG90 Servo Motor                            |        2 | Camera yaw and pitch control              |
| Buck Converter                              |        1 | Voltage regulation                        |
| Power Distribution Board                    |        1 | Power distribution                        |
| Battery Pack                                |        1 | Primary power source                      |
| Chassis                                     |        1 | Robot frame                               |

---

# Computing Unit

## Raspberry Pi 4

**Specifications**

* RAM: 4 GB
* Operating System: Ubuntu 22.04 LTS
* ROS2 Distribution: Humble Hawksbill

### Responsibilities

* Runs ROS2
* Navigation2 Stack
* SLAM Toolbox
* AMCL Localization
* RPLidar Driver
* Path Planning
* Obstacle Avoidance
* ESP32 Camera Stream
* Serial Communication with Arduino

---

# Navigation Sensor

## RPLidar A1M8

### Specifications

* 360° Laser Scanner
* Maximum Range: 12 meters
* USB Communication
* 2D LiDAR

### Purpose

* Environment Mapping
* Localization
* Obstacle Detection
* Costmap Generation
* Autonomous Navigation

ROS Topic

```
/scan
```

---

# Motion System

## Drive Configuration

* Differential Drive Robot

### Motors

**TTN25 Encoder Gear Motors**

Quantity

```
2
```

Specifications

* Operating Voltage: 12V
* Speed: 370 RPM
* Integrated Quadrature Encoder

Functions

* Robot locomotion
* Closed-loop velocity control
* Wheel odometry

---

# Motor Driver

## Cytron MDD10A

Specifications

* Dual Channel
* 10A Continuous Current
* PWM + Direction Interface

Purpose

* Controls left motor
* Controls right motor
* Receives PWM signals from Arduino Nano

---

# Embedded Controller

## Arduino Nano

### Responsibilities

* Reads wheel encoders
* Calculates wheel velocity
* PID motor control
* Receives velocity commands
* Sends odometry to Raspberry Pi
* Controls MDD10A

Communication

```
USB Serial
```

---

# Vision System

## ESP32-CAM

Purpose

* Live video streaming
* Warehouse monitoring
* Shelf inspection
* Future barcode detection
* Future QR code detection
* Future object detection

Communication

```
Wi-Fi
```

---

# Camera Gimbal

The camera is mounted on a custom-built two-axis gimbal.

Components

* ESP32
* PCA9685 Servo Driver
* SG90 Servo (Yaw)
* SG90 Servo (Pitch)

Capabilities

* Horizontal rotation
* Vertical rotation
* Shelf inspection
* Remote camera control

---

# Servo Driver

## PCA9685

Purpose

* 16-channel PWM controller
* Drives both SG90 servos
* Controlled using I²C

---

# Power System

## Battery

Provides power to

* Raspberry Pi
* Arduino Nano
* Motor Driver
* Motors
* ESP32
* Servo Driver

---

## Power Distribution Board

Purpose

* Safe power distribution
* Organized wiring
* Multiple voltage outputs

---

## Buck Converter

Purpose

Converts battery voltage into regulated outputs for

* Raspberry Pi
* Arduino Nano
* ESP32
* PCA9685
* Servo System

---

# Communication Interfaces

| Interface       | Devices                     |
| --------------- | --------------------------- |
| USB             | Raspberry Pi ↔ RPLidar      |
| USB Serial      | Raspberry Pi ↔ Arduino Nano |
| Wi-Fi           | Raspberry Pi ↔ ESP32-CAM    |
| I²C             | ESP32 ↔ PCA9685             |
| PWM             | PCA9685 ↔ SG90 Servos       |
| PWM + Direction | Arduino ↔ MDD10A            |

---

# Software-Hardware Mapping

| Hardware           | Software               |
| ------------------ | ---------------------- |
| Raspberry Pi       | ROS2 Humble            |
| RPLidar            | rplidar_ros            |
| Arduino Nano       | Motor Control Firmware |
| Encoders           | Wheel Odometry         |
| Differential Drive | ros2_control           |
| ESP32-CAM          | Live Video Streaming   |
| Navigation         | Navigation2            |
| Mapping            | SLAM Toolbox           |
| Localization       | AMCL                   |

---

# Robot Features

* Autonomous SLAM mapping
* Autonomous localization
* Autonomous navigation
* Obstacle avoidance
* Differential drive motion
* Wheel encoder odometry
* Closed-loop motor control
* Live Wi-Fi camera feed
* Two-axis camera gimbal
* Shelf inspection
* Expandable hardware architecture

---

# Future Hardware Upgrades

* IMU (BNO055/BNO085)
* Battery Management System (BMS)
* Charging Dock
* QR/Barcode Scanner
* RGB-D Camera
* Industrial Emergency Stop
* Status LEDs and Buzzer
* Environmental Sensors
* Higher resolution camera
* AI accelerator (Google Coral TPU or Intel Neural Compute Stick)

---

# Overall Hardware Architecture

```text
                   +-----------------------------+
                   |      Raspberry Pi 4         |
                   |-----------------------------|
                   | ROS2 Humble                 |
                   | Navigation2                 |
                   | SLAM Toolbox                |
                   | AMCL                        |
                   +-------------+---------------+
                                 |
                    USB Serial
                                 |
                          Arduino Nano
                +----------------+----------------+
                |                                 |
          Encoder Feedback                 MDD10A Driver
                |                                 |
        Left / Right Encoders          Left / Right Motors

RPLidar A1M8 -------- USB --------> Raspberry Pi

ESP32-CAM ---------- Wi-Fi -------> Raspberry Pi

ESP32 ---- I²C ----> PCA9685 ----> SG90 (Yaw)

                               └──> SG90 (Pitch)

Battery
   │
   ├── Power Distribution Board
   │
   ├── Buck Converter
   │
   ├── Raspberry Pi
   ├── Arduino Nano
   ├── ESP32
   ├── PCA9685
   └── Motor Driver
```

---

# Hardware Summary

The Warehouse Stock Navigation Robot is built around a modular ROS2 architecture using a Raspberry Pi 4 as the main computing platform. It combines LiDAR-based autonomous navigation, encoder-based odometry, differential drive control, and a Wi-Fi-enabled pan-tilt camera system. The modular design allows future integration of AI-based inventory management, barcode/QR scanning, autonomous charging, and advanced perception sensors, making it suitable for research, education, and warehouse automation applications.
