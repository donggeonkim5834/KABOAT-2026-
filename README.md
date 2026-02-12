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

# 🏁 Competition Field Overview

## 🌊 Overall Field Layout
- Description of the complete competition environment  
- Includes waypoint zones, buoy obstacles, hopping section, and docking area  

<img width="712" height="336" alt="image" src="https://github.com/user-attachments/assets/5e95a6dc-3ed7-46eb-a2af-65c2fd03a18e" />

---

# 📍 Course-Specific Algorithms

---

# 🥇 Course 1 – GPS-Based Navigation with LiDAR Obstacle Avoidance

## 📡 Sensor Initialization
All sensors (LiDAR, IMU, GPS, Camera) are activated and continuously subscribed via ROS 2 callbacks from system startup.

At every control frame, the **servo target angle** is determined according to the following priority:

---

## 🎯 Steering Decision Priority

### 1️⃣ Primary: GPS Heading Tracking
The vessel follows the heading angle toward the target GPS waypoint:
- First waypoint  
- After passing all buoy obstacles  
- Entry point before the purple buoy (hopping section)

---

### 2️⃣ Secondary: LiDAR-Based Obstacle Avoidance

If LiDAR detects an obstacle within a predefined safety distance:

1. Select front **180° LiDAR scan data**
2. Divide into **180 angular bins (1° resolution)**
3. Average values inside each bin → produce a 180-element distance array (unit: meters)

Each distance value is mapped to a risk score using an exponential function

- Closer distance → Higher risk (0–100 scale)
- Risk above threshold → Marked as **Danger (0)**
- Otherwise → **Safe (1)**

### 🚧 Vessel Width Compensation
For every detected danger index:
- Extend danger marking to ±5 neighboring indices
- Ensures collision-free clearance considering vessel width

### ✅ Final Steering Selection
Among all remaining safe angles:
- Select the angle closest to the GPS target heading.

---

## 🔄 Transition to Hopping Mode

Upon reaching the first waypoint:

- IMU heading is adjusted to approximately **+45° starboard turn**
- Purpose: Bring purple buoy into camera’s left field of view
- If buoy detected **or 3 seconds elapsed**, transition to Course 2

---

# 🥈 Course 2 – Vision-LiDAR Fusion & Hopping Algorithm

## 🎥 Sensor Calibration

- Camera faces **-90° (port side)**
- LiDAR scans **front 180°**
- Overlapping usable range: **0° to -90°**

Pixel offset from camera center is converted into angular displacement.
This is aligned with LiDAR angular measurements.

---

## 🔁 Hopping Control Logic (Line-Tracing Principle)

1. Follow IMU heading ≈ **135°**
2. When buoy detected:
   - Adjust steering so buoy center aligns with **-90° vessel heading**
3. If buoy center = -90° → Move straight
4. If buoy center = -110° (example threshold):
   - Command 30° port turn (thruster 60%)

Since vessel circles buoy from right side:
- Exit loop after IMU heading crosses 0° twice

---

## 🎯 Target Object Recognition

After loop exit:

- Follow IMU -90°
- Move straight until camera detects target object
- Align object center with camera center (-90° vessel heading)
- Activate LED corresponding to object color
- Continue to second waypoint (junction center)

---

# 🥉 Course 3 – LiDAR-Based Free-Space Navigation (Docking)

Camera is disabled.
Navigation relies on **LiDAR + IMU only**.

Unlike Course 1:
- No GPS heading tracking
- Follow the direction of **maximum LiDAR distance**

---

## 📐 Steering Decision Logic

At each LiDAR callback frame:

1. Compute intersection between:
   - IMU heading range (0° to -180°)
   - LiDAR front 180° data

2. Identify angle with **maximum distance**

3. If maximum distance ≤ 0.3 m:
   - Stop immediately

---

## 📊 LiDAR Processing

- Front 180° divided into 180 bins
- Averaged per 1°
- Produces 180-element distance list

Selecting the maximum-distance angle naturally:
- Guides vessel toward open space
- Avoids walls
- Handles corner navigation implicitly

Turning radius is controlled via thruster output modulation.

The stopping threshold (0.3 m) remains active throughout Course 3.

---

## 🔧 Algorithm Extensibility

Although the farthest-distance strategy is the simplest implementation, the following can be incorporated:

- Course 1–style obstacle risk evaluation
- Adaptive safety angle generation
- Turning radius–aware safe region selection

---

