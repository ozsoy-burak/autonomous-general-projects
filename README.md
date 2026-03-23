# Teknofest 2025 — Autonomous Vehicle

A ROS-based autonomous electric vehicle developed for Teknofest 2025. The system operates in a structured urban-like environment (~100×80 m) and is capable of following lanes, obeying traffic lights and signs, navigating to target waypoints, and autonomously parking in a designated zone.


## Hardware

| Component | Role |
|---|---|
| ZED2 Stereo Camera | Visual odometry, depth estimation, sign/lane detection |
| Velodyne VLP-16 LiDAR | 3D mapping, scan matching, localization |
| IMU | Inertial measurements for sensor fusion |
| Wheel RPM Encoders | Ackermann wheel odometry |
| Vehicle Computer | Central compute for all ROS nodes |


## Modules

### Lane Following
A dedicated lane detection module processes ZED2 camera frames to identify lane markings and generate steering commands. The vehicle maintains its lane throughout the course using visual feedback.

### Traffic Light & Sign Detection
A YOLOv11m model, custom-trained on a Teknofest-specific dataset, handles detection of traffic lights and road signs in real time. Detected classes are published as ROS topics and consumed by the decision-making layer to trigger appropriate vehicle behaviors (stop, new path planning etc.).

![20250805_095602_165783](https://github.com/user-attachments/assets/5b12781f-96cf-4c63-98fd-c667c767a78d)
![20250805_095404_071022](https://github.com/user-attachments/assets/9248d0bf-55db-4f97-9caa-12abf35fe2cc)



### Sensor Fusion & Localization
Localization is achieved in two stages:

**Stage 1 — Odometry fusion:**
Four odometry sources are fused using the `robot_localization` package (Extended Kalman Filter) with individually tuned covariance matrices:
- Wheel odometry (Ackermann RPM model)
- IMU
- Visual odometry (RTAB-Map / ZED2)
- Laser scan odometry (rf2o)

This fusion produces a stable `odom → base_link` transform.


**Stage 2 — Map-based correction:**
HDL SLAM is used to build a 3D point cloud map of the environment offline. At runtime, HDL Localization performs NDT (Normal Distributions Transform) scan matching between live VLP-16 data and the pre-built map, continuously correcting accumulated drift and yielding an accurate `map → odom` transform.
<img width="1920" height="1080" alt="Screenshot from 2025-09-17 01-37-56" src="https://github.com/user-attachments/assets/bcd1e41d-fab4-48db-8f2f-508f88f31bfb" />


### 3D Mapping
The environment map is built using HDL Graph SLAM on VLP-16 point cloud data. The resulting map serves as the reference for NDT-based localization during the run.
<img width="1920" height="1080" alt="Screenshot from 2025-09-17 01-34-43" src="https://github.com/user-attachments/assets/5ac32c47-c0e9-4984-9895-ad9ccc8587fb" />


### Navigation
The ROS Navigation Stack is used for global and local path planning. The **TEB (Timed Elastic Band)** local planner is configured for the vehicle's Ackermann kinematics, enabling smooth and kinematically feasible trajectory execution in dynamic environments. Several optimizations were applied to the costmap and planner parameters for reliable urban-course navigation.
<img width="623" height="784" alt="Screenshot from 2026-03-23 14-41-02" src="https://github.com/user-attachments/assets/f86b96af-4d7c-48ae-a68e-60ae476a0057" />

### Parking
The parking module operates as a two-stage pipeline:

1. **3D localization of the parking sign** — When the parking sign is detected by YOLOv11m, its 2D bounding box is back-projected into 3D space using ZED2 depth data. The resulting 3D pose is anchored to the map frame via a TF transform, giving the sign a fixed coordinate in the map.

2. **Zone matching and path planning** — A second node reads the sign's map coordinate and matches it against a set of predefined candidate parking zones. The best matching zone is selected, and the navigation stack plans and executes a path to that zone, completing the autonomous parking maneuver.

---

## Key Technologies

| Area | Technology |
|---|---|
| Middleware | ROS Noetic |
| Detection models | YOLOv11m (custom trained) |
| Sensor fusion | robot_localization (EKF) |
| Mapping | HDL Graph SLAM |
| Localization | HDL Localization (NDT) |
| Local planner | TEB (Timed Elastic Band) |
| Depth & visual odom | ZED2 + RTAB-Map |
| Language | C++, Python |
