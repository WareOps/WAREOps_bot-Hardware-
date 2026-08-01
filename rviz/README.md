# `rviz` — RViz2 Display Profiles & Visualization Package

**Deployment Target**: 🔵 **Local Computer**

The `rviz` directory contains pre-configured RViz2 display profile files (`.rviz`) designed for visualizing the Warehouse Bot on the operator's **Local Computer**.

---

## 📁 Files Included

```text
rviz/
├── rviz_config.rviz       # General visualization profile (Robot model, TF, LiDAR scan, Odometry)
└── rviz_config_nav2.rviz  # Navigation visualization profile (Global/Local Costmaps, Path Plan, AMCL pose)
```

---

## 🛠️ Configuration Details

### 1. `rviz_config.rviz` (General / Teleop Display)
Configured displays:
- **RobotModel**: Renders 3D URDF visualization using `/robot_description` topic.
- **TF Tree**: Visualizes coordinate frames (`map` -> `odom` -> `base_link` -> `laser_frame`).
- **LaserScan**: Renders 2D LiDAR range points from `/scan` topic in red.
- **Odometry**: Plots real-time wheel odometry path from `/odom`.

### 2. `rviz_config_nav2.rviz` (Autonomous Navigation Display)
Configured displays:
- **Global Costmap**: Visualizes global obstacle inflation layer on map frame.
- **Local Costmap**: Visualizes real-time local rolling costmap for dynamic obstacle avoidance.
- **Planner Path**: Renders calculated path trajectory (`/plan` / `/transformed_global_plan`).
- **Particle Cloud**: Displays AMCL particle array for localization estimation uncertainty.
- **2D Pose Estimate**: Interactive tool for setting initial robot pose in AMCL.
- **2D Nav Goal**: Interactive tool for dispatching navigation goals to Navigation2.

---

## 🚀 Usage Instructions

Ensure RViz2 is installed on your **Local Computer**:
```bash
sudo apt update
sudo apt install -y ros-humble-rviz2
```

Launch RViz with your target profile:

### Launch General Visualizer:
```bash
rviz2 -d src/rviz/rviz_config.rviz
```

### Launch Navigation Visualizer:
```bash
rviz2 -d src/rviz/rviz_config_nav2.rviz
```
