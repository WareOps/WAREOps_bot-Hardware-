# `serial` — Cross-Platform C++ Serial Library

**Deployment Target**: 🔴 **Raspberry Pi**

The `serial` package is a cross-platform C++ library for interfacing with RS-232 serial ports. In this project workspace, it serves as a critical dependency for the `diffdrive_arduino` ROS2 hardware interface package running on the **Raspberry Pi 4**.

---

## 🛠️ Role in Warehouse Bot System

`serial` enables `diffdrive_arduino` to establish raw serial communication over USB (e.g., `/dev/ttyACM0` or `/dev/ttyUSB0`) with the Arduino Nano motor controller:
* Transmitting formatted velocity motor commands (`m <left_speed> <right_speed>\r`).
* Requesting and receiving encoder tick counts (`e\r`).
* Configuring baudrates (default: 57600 baud) and timeout handling.

---

## 📦 Build & Installation

This package is automatically compiled as part of the ROS2 workspace build process on the Raspberry Pi:

```bash
cd ~/ros2_ws
colcon build --packages-select serial
source install/setup.bash
```

---

## 📄 API & Features

* Standard PySerial-like modern C++ API (`open()`, `close()`, `read()`, `write()`, `readline()`).
* Tight control over byte timeouts and control lines (RTS, CTS, DTR, DSR).
* Zero external third-party dependencies required.

---

## 📜 License & Original Authors
* **Author**: William Woodall (<william@osrfoundation.org>), John Harrison
* **License**: MIT License
