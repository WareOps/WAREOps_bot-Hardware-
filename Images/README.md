# WareOps Bot - Hardware & Assembly Gallery

This directory contains high-resolution photographs detailing the physical hardware, internal electronics, sensor integration, wiring layout, and 3D-printing manufacturing process of the **WareOps Autonomous Warehouse Inspection Robot**.

---

## 🤖 System Overview & Feature Architecture

![WareOps Bot System Overview](Bot.png)
*WareOps Autonomous Mobile Robot hardware architecture, active vision pan-tilt system, sensor suite, and system capabilities overview.*

---

## 📸 Assembled Robot Views

### Robot Perspective Overview
![WareOps Bot Side Perspective](image_1.jpeg)
*Side perspective view of the fully assembled robot showing the top-mounted RPLiDAR A1, front pan/tilt camera scanner, and differential drive wheels.*

### Front View & Sensor Alignment
![WareOps Bot Front View](image_2.jpeg)
*Front view highlighting the active vision camera module mounted on a pan/tilt servo assembly and the top LiDAR.*

### Front-Isometric View
![WareOps Bot Isometric View](image_3.jpeg)
*Isometric view showing compact chassis design and wheel mounting.*

### Front Sensor Array
![WareOps Bot Sensor Array](image_4.jpeg)
*Close-up front view detailing sensor positioning for autonomous navigation and barcode/QR rack scanning.*

### Floor Navigation Testing
![WareOps Bot Floor Test](image_7.jpeg)
*Side profile of assembled robot undergoing mobility and ground-truth transform testing.*

### Navigation Test Grid
![WareOps Bot Navigation Grid](image_11.jpeg)
*Robot positioned on floor layout grid during AMCL localization calibration.*

---

## ⚡ Internal Electronics & Control Hardware

### Electronics Bay Overview
![Internal Electronics Layout](image_5.jpeg)
*Open chassis compartment displaying the LiPo battery placement, Cytron motor driver, and main controller board.*

### Control Board & Power Layout
![Top Down Component View](image_6.jpeg)
*Top-down layout showing Cytron motor driver, microcontroller control board, DC-DC buck converters, and 2200mAh 3S LiPo battery.*

### Component Placement & Power Distribution
![Component Layout](image_8.jpeg)
*Overhead component mapping with top mounting plate detached.*

### High-Angle Hardware Layout
![High Angle Assembly Layout](image_9.jpeg)
*Component layout reference prior to top plate installation.*

### Electronics Board Close-Up
![Electronics Boards Close-Up](image_10.jpeg)
*Detailed close-up of motor drive electronics, power distribution circuitry, and microcontroller interconnects.*

### Internal Wiring & Harness
![Internal Wiring Harness](image_12.jpeg)
*Detailed view of internal cabling, USB interconnects, DC power wiring, and sensor communication lines.*

---

## 🖨️ 3D Printing & Manufacturing

### Chassis Printing Process
![3D Printing Chassis Body](image_13.jpeg)
*Fabrication of the main blue robot chassis enclosure using an Anycubic FDM 3D printer.*

### Print Bed Fabrication Detail
![3D Printer Bed Close-Up](image_14.jpeg)
*Close-up view of the custom 3D printed body shell during printing process.*

---

## 🛠️ Hardware Specification Summary

- **Main Controller**: Raspberry Pi 4 (ROS 2 Humble, Nav2, AMCL, SLAM Toolbox)
- **Motor Control**: Arduino Nano & Cytron Dual-Channel DC Motor Driver
- **LiDAR Sensor**: RPLidar A1M8 360° 2D Laser Scanner
- **Active Vision System**: ESP32-CAM Pan/Tilt 2-DOF Camera Module for multi-tier rack QR/barcode auditing
- **Power System**: 11.1V 3S High Performance LiPo Battery with DC-DC Buck Converters
- **Chassis**: Custom 3D-Printed High-Durability Modular Enclosure
