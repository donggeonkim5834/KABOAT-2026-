# KABOAT-2026-
A ROS 2–based autonomous navigation and control algorithm for an unmanned surface vehicle (USV), developed for the KABOAT 2026 competition.

System Configuration
The system is equipped with LiDAR, IMU, GPS, and a vision camera for perception and localization.
Actuation is achieved through a servo motor for steering and a thruster for propulsion control.

Coordinate Frame Definition
Coordinate Frame Alignment and Fusion: Sensor measurements from heterogeneous reference frames—LIDAR in the vessel-relative frame and GPS/IMU in the global frame—are transformed into a unified vessel-fixed coordinate system referenced to the bow-aligned X-axis, enabling consistent spatial integration and sensor fusion.
