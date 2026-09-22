---
layout: page
title: "SLAM & Autonomous Driving System Architecture"
permalink: /architecture/
---

## 1. Hardware & Sensor Specs
- **Computing Platform:** NVIDIA Jetson Orin Nano / x86 Onboard PC (Ubuntu 22.04 LTS)
- **Primary Sensor:** RPLiDAR A2 / Livox Mid-360 LiDAR
- **IMU:** 9-DOF Bosch BNO055 / MicroStrain 3DM-GX5
- **Mobile Base:** Clearpath Jackal UGV / 4WD Differential Drive Robot

---

## 2. Software Architecture

```
+-------------------------------------------------------------+
|                      User / Mission UI                      |
|                  (2D Goal Pose / Waypoints)                 |
+------------------------------+------------------------------+
                               | Action Goal
                               v
+-------------------------------------------------------------+
|                       Nav2 Controller                       |
|  +-----------------------+     +--------------------------+ |
|  | Global Planner (Smac) |     | Local Planner (DWB/TEB)  | |
|  +-----------------------+     +--------------------------+ |
|  +--------------------------------------------------------+ |
|  | Costmap 2D (Static, Inflation, Obstacle Layer)         | |
|  +--------------------------------------------------------+ |
+--------------+-------------------------------+--------------+
               |                               | /cmd_vel
               v                               v
+-------------------------------+  +--------------------------+
|       SLAM / Localization     |  |     Motor Controller     |
| (Slam Toolbox / Cartographer) |  |   (Wheel Base Driver)    |
+---------------+---------------+  +--------------------------+
                ^
     /scan, /tf |
+---------------+---------------+
|       Sensors (LiDAR / IMU)   |
+-------------------------------+
```

---

## 3. Localization & Path Planning Pipeline
1. **Perception Layer:** LiDAR 데이터를 포인트클라우드 또는 레이저 스캔 토픽(`/scan`)으로 수신.
2. **State Estimation:** 휠 오도메트리와 IMU 데이터를 EKF(`robot_localization`)로 융합하여 `/odometry/filtered` 생성.
3. **Mapping & SLAM:** `slam_toolbox` 노드가 포즈 그래프 최적화를 거쳐 실시간 맵(`/map`) 발행.
4. **Trajectory Execution:** 목적지 도달 시까지 Costmap 장애물을 회피하며 `/cmd_vel` 제어 명령을 모터 드라이버로 송신.
