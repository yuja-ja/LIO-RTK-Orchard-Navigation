# A LiDAR-Inertial-RTK Fusion Localization and Fruit-Tree Instance Mapping System for Tracked Inspection Robots in Non-Standardized Orchards

[![ROS](https://img.shields.io/badge/ROS-Noetic%20%2F%20Melodic-blue.svg)](http://wiki.ros.org/)
[![C++](https://img.shields.io/badge/Language-C%2B%2B14-brightgreen.svg)](https://isocpp.org/)
[![Python](https://img.shields.io/badge/Python-3.8%2B-yellow.svg)](https://www.python.org/)
[![Status](https://img.shields.io/badge/Paper_Status-Under_Review-orange.svg)](#videos--demonstrations)
[![Video](https://img.shields.io/badge/Video_Demo-Bilibili%20%7C%20YouTube-red.svg)](#videos--demonstrations)

This repository contains the software modules for a LiDAR-inertial-RTK fusion localization and fruit-tree instance mapping system tailored for tracked mobile inspection platforms in non-standardized orchards.

---

## 1. System Overview

```text
======================= System Pipeline & Data Flow =======================

[ Sensor Hardware ]  32-channel LiDAR + 10-axis IMU + Dual-antenna RTK + Tracked Chassis
                              │                    │                    │
                              ▼                    ▼                    ▼
[ State Estimation ] ───────► Asynchronous Sequential LIO-RTK Estimator
                              │
                              ▼ Outputs 3D point cloud (.pcd)
[ Instance Mapping ] ───────► Individual-Tree Semantic Mapping
                              │
                              ▼
[ Global Relocalization ] ──► Fixed-Map Relocalization
                              │
                              ▼
[ Motion Control ] ─────────► Hierarchical Navigation & Chassis Control
```

---

## 2. Hardware & Software Specifications

* **Operating System**: Ubuntu 18.04 (ROS Melodic) / Ubuntu 20.04 (ROS Noetic)
* **Core Libraries**: PCL (>= 1.8), Eigen (>= 3.3.4), OpenMP, Python 3
* **Onboard Computer**: Intel NUC 12 Pro (Intel Core i7-1260P, 16 GB RAM)
* **LiDAR Sensor**: LSLiDAR C32 (32-channel mechanical LiDAR, 10 Hz)
* **IMU**: 10-axis inertial measurement unit (200 Hz)
* **Dual-Antenna RTK**: Beitian BT-982K2 receivers (10 Hz, dual-antenna heading)
* **Tracked Platform**: Differential skid-steering tracked agricultural chassis (RS-232 serial control at 115,200 bps)

---

## 3. LIO-RTK State Estimation (`/LIO-RTK`)

### Asynchronous Sequential LIO-RTK Fusion Estimator
* Direct point-to-plane residual formulation based on incremental kd-tree (ikd-Tree)
* Asynchronous soft-anchoring global transformation estimation (translation and yaw)
* Outlier rejection and temporal consistency checks for satellite signal anomalies

---

## 4. Semantic Mapping, Relocalization & Navigation (`/导航`)

### 4.1. Fruit-Tree Instance Semantic Mapping (`orchard_semantic_map`)
* Elevation-span analysis and local ground plane estimation
* Local non-maximum suppression (NMS) for trunk centroid extraction
* Generation of georeferenced fruit-tree coordinate catalogs (`tree_inventory.csv`)

### 4.2. 3D Point-Cloud to 2D Costmap Conversion (`pcd2gridmap`)
* Elevation pass-band filtering to remove low ground weeds (0.00 – 0.15 m) and high canopies (> 1.20 m)
* Generation of 2D static occupancy grid maps (`orchard_map.pgm`, `orchard_map.yaml`)

### 4.3. Fixed-Map Global Relocalization (`FAST_LIO-RTK`)
* Read-only static ikd-Tree point-to-plane ICP registration
* Smooth coordinate frame decoupling between map and odometry frames

### 4.4. Hierarchical Autonomous Navigation (`fast_lio_navigation`)
* Potential field reshaped global path planning for tree-row centerline guidance
* Timed Elastic Band (TEB) local trajectory optimization with nonholonomic kinematic constraints
* Chassis motor serial communication interface (`velocity_controller_node`)
* Mobile terminal teleoperation bridge via WebSocket (`app_ros_bridge_node`)

---

## 5. Dual-RTK Reference Protocol & Controlled Degradation Benchmark

### 5.1. Independent Dual-RTK Reference Setup
To perform an objective, self-reference-free evaluation, two physically independent dual-antenna RTK systems were deployed on the vehicle:
* **Fusion RTK**: Connected to the estimator, receiving RTCM32 differential corrections.
* **Reference Ground-Truth RTK**: Logged independently for offline evaluation, receiving RTCM33 differential corrections.
* **Quality Screening**: The reference trajectory was extracted through strict offline gating (carrier-phase fixed status, minimum satellite count, low dilution of precision, and differential age limits). Repeated stationary field trials confirmed sub-centimeter horizontal repeatability (CEP95 within 0.77–0.97 cm).

### 5.2. Controlled Signal Degradation Scenarios
To systematically evaluate performance under canopy-induced signal anomalies, four synthetic degradation conditions were applied to the input stream across three random seeds:
* **Dropout**: Complete omission of RTK observations (20 s windows) to evaluate dead-reckoning performance.
* **Bias**: Gradual horizontal multipath drift (0.50 m nominal offset with 0.03 Hz ripple).
* **Jump**: Sudden step-like horizontal position displacements (1.00 m).
* **False-Fixed**: Deceptive position offsets (0.80 m) injected while keeping fixed status and unscaled covariance to simulate covert integer ambiguity errors.

---

## 6. Videos & Demonstrations

* **Field Navigation Demonstration (YouTube)**: [https://www.youtube.com/watch?v=52Jc8SSe3pU](https://www.youtube.com/watch?v=52Jc8SSe3pU)
* **Field Navigation Demonstration (Bilibili)**: [https://www.bilibili.com/video/BV1WUEQ6TEqa](https://www.bilibili.com/video/BV1WUEQ6TEqa)
