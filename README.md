# 🚤 KABOAT-2026

> **A ROS 2–based autonomous navigation and control framework for an Unmanned Surface Vehicle (USV), developed for the KABOAT 2026 competition.**

---

## 🛠 System Configuration

### 🔍 Sensors
- **LiDAR** – Obstacle detection and environment mapping  
- **IMU (Inertial Measurement Unit)** – Attitude and heading estimation  
- **GPS (Global Positioning System)** – Global positioning and waypoint navigation  
- **Vision Camera** – Object detection and visual perception  

### ⚙️ Actuators
- **Servo Motor** – Steering control  
- **Thruster** – Propulsion control  

---

## 🧭 Coordinate Frame Definition

To ensure consistent spatial representation across heterogeneous sensors, a unified coordinate framework is established.

### 🔄 Coordinate Frame Alignment & Fusion
- **LiDAR data** are represented in the vessel-relative coordinate frame.  
- **GPS and IMU data** are initially expressed in the global (absolute) reference frame.  
- Global measurements are transformed into a **vessel-fixed coordinate system**, aligned with the **bow-referenced X-axis**.  
- All sensor data are integrated into this unified frame, enabling robust spatial consistency and reliable sensor fusion.
