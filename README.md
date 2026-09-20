# A LiDAR-Inertial-RTK Fusion Localization and Fruit-Tree Instance Mapping System for Tracked Inspection Robots in Non-Standardized Orchards

[![ROS](https://img.shields.io/badge/ROS-Noetic%20%2F%20Melodic-blue.svg)](http://wiki.ros.org/)
[![C++](https://img.shields.io/badge/Language-C%2B%2B14-brightgreen.svg)](https://isocpp.org/)
[![Python](https://img.shields.io/badge/Python-3.8%2B-yellow.svg)](https://www.python.org/)
[![Status](https://img.shields.io/badge/Paper_Status-Under_Review-orange.svg)](#10-citation--acknowledgments)
[![Video](https://img.shields.io/badge/Video_Demo-Bilibili%20%7C%20YouTube-red.svg)](#11-demonstration-videos--data-availability)

This repository hosts the official implementation of the paper:  
**"A LiDAR-Inertial-RTK Fusion Localization and Fruit-Tree Instance Mapping System for Tracked Inspection Robots in Non-Standardized Orchards"** *(Under Review at Computers and Electronics in Agriculture)*.

Developed by the **College of Artificial Intelligence and Low-Altitude Technology, South China Agricultural University (SCAU)** and **Pazhou Lab, Guangzhou, China**.

---

## Table of Contents

- [1. System Overview & Architecture](#1-system-overview--architecture)
- [2. Repository Structure](#2-repository-structure)
- [3. Hardware & Software Prerequisites](#3-hardware--software-prerequisites)
- [4. Build & Installation](#4-build--installation)
- [5. Module 1: Robust LIO-RTK Localization (`/LIO-RTK`)](#5-module-1-robust-lio-rtk-localization-lio-rtk)
- [6. Module 2: Semantic Mapping, Relocalization & Navigation (`/导航`)](#6-module-2-semantic-mapping-relocalization--navigation-导航)
  - [6.1 Fruit-Tree Instance Semantic Mapping (`orchard_semantic_map`)](#61-fruit-tree-instance-semantic-mapping-orchard_semantic_map)
  - [6.2 3D Point-Cloud to 2D Safety Costmap (`pcd2gridmap`)](#62-3d-point-cloud-to-2d-safety-costmap-pcd2gridmap)
  - [6.3 Map-Based Global Relocalization (`FAST_LIO-RTK`)](#63-map-based-global-relocalization-fast_lio-rtk)
  - [6.4 Hierarchical Autonomous Navigation (`fast_lio_navigation`)](#64-hierarchical-autonomous-navigation-fast_lio_navigation)
- [7. Coordinate Frames & ROS Interfaces](#7-coordinate-frames--ros-interfaces)
- [8. Dual-RTK Ground-Truth Protocol & Degradation Benchmark](#8-dual-rtk-ground-truth-protocol--degradation-benchmark)
- [9. Quick Start Guide](#9-quick-start-guide)
- [10. Citation & Acknowledgments](#10-citation--acknowledgments)
- [11. Demonstration Videos & Data Availability](#11-demonstration-videos--data-availability)

---

## 1. System Overview & Architecture

Autonomous inspection in non-standardized orchards faces severe operational challenges arising from dense canopy occlusions, repetitive trunk corridors, and track slippage on uneven soils. This study presents an end-to-end autonomy pipeline comprising **two primary scientific contributions** supported by **two deployment modules**:

```text
======================= System Pipeline & Data Flow =======================

[ Sensor Hardware ]  32-channel LiDAR (C32) + 10-axis IMU + Dual-antenna RTK + Tracked Chassis
                              │                    │                    │
                              ▼                    ▼                    ▼
[ Core Contribution 1 ] ──► Asynchronous Sequential LIO-RTK Estimator
                             ├── FAST-LIO2 front-end (ikd-Tree + direct point-to-plane IEKF)
                             ├── 4-DoF soft-anchoring global transformation (translation + yaw)
                             └── Multi-source temporal fault gating (rejecting dropout, bias, jump, false-fixed)
                              │
                              ▼ Outputs high-fidelity, georeferenced 3D point cloud (.pcd)
[ Core Contribution 2 ] ──► Individual-Tree Instance Semantic Mapping
                             ├── Elevation-span decoupling (filters out orchard floor clutter/weeds)
                             ├── Local non-maximum suppression (NMS, inter-tree distance >= 1.8 m)
                             └── Outputs georeferenced tree catalog (tree_inventory.csv with WGS-84 coordinates)
                              │
                              ▼
[ Supporting Module A ] ──► Fixed-Map Global Relocalization
                             ├── Read-only static ikd-Tree point-to-plane ICP
                             └── Decoupled Map-to-Odom frame updates (ensuring smooth continuous odometry)
                              │
                              ▼
[ Supporting Module B ] ──► Hierarchical Autonomous Navigation & Chassis Control
                             ├── Costmap-reshaped Dijkstra global planner (corridor centerline guidance)
                             ├── Nonholonomic TEB local trajectory optimization (dynamic obstacle avoidance)
                             ├── Low-level RS-232 serial velocity controller with deadband compensation
                             └── Android supervisory terminal (WebSocket via rosbridge_server)
```

---

## 2. Repository Structure

The workspace is organized into two primary project folders:

```bash
.
├── LIO-RTK/                      # Folder 1: Robust LIO-RTK state estimation and mapping
│   └── src/
│       ├── LIO-RTK/              # Core state-estimation package (extended from FAST-LIO2)
│       │   ├── config/           # Sensor extrinsic calibration & fusion parameters (lslidar_rtk.yaml)
│       │   ├── launch/           # Mapping entry points (rtk_mapping.launch)
│       │   ├── include/          # ikd-Tree, IKFoM toolkit, and asynchronous RTK fusion headers
│       │   └── src/              # laserMapping.cpp, rtk_fusion.cpp implementations
│       ├── nmea_rtk_driver/      # NMEA dual-antenna RTK driver with UTC hardware synchronization
│       ├── lslidar_driver/       # Driver for LSLiDAR C32 32-channel mechanical LiDAR
│       ├── livox_ros_driver/     # Compatible driver for Livox solid-state LiDARs
│       ├── imu_ros_driver/       # 10-axis IMU driver and temporal alignment node
│       └── rtk_truth_tum/        # Dual-RTK independent ground-truth recorder and TUM exporter
│
└── 导航/                         # Folder 2: Semantic mapping, relocalization, and navigation
    ├── orchard_semantic_map/     # Fruit-tree instance extraction node (generates tree_inventory.csv)
    ├── pcd2gridmap/              # 3D PCD point-cloud elevation slicing to 2D costmap converter
    ├── FAST_LIO-RTK/             # Global relocalization package on static ikd-Tree maps
    ├── fast_lio_navigation/      # move_base navigation stack, TEB/DWA planners, serial & APP bridge
    ├── ls2velo/                  # Point-cloud format adapter (LSLiDAR to standard Velodyne PointXYZIRT)
    ├── lslidar_driver/           # Navigation deployment LiDAR driver
    ├── rtk_driver/               # Navigation-side RTK serial communication interface
    └── wit_imu_driver/           # 10-axis IMU interface driver
```

---

## 3. Hardware & Software Prerequisites

### 3.1 Software Requirements
* **Operating System**: Ubuntu 18.04 LTS (ROS Melodic) or Ubuntu 20.04 LTS (ROS Noetic)
* **Compiler**: CMake >= 3.0.2, GCC/G++ >= 7.5.0 (C++14 standard)
* **Dependencies**: PCL >= 1.8, Eigen >= 3.3.4, OpenMP, Python 3.8+
* **ROS Packages**:
  ```bash
  sudo apt-get update
  sudo apt-get install ros-$ROS_DISTRO-navigation \
                       ros-$ROS_DISTRO-teb-local-planner \
                       ros-$ROS_DISTRO-rosbridge-server \
                       ros-$ROS_DISTRO-map-server \
                       ros-$ROS_DISTRO-costmap-2d
  ```

### 3.2 Tracked Robot Hardware Platform
* **Onboard Computer**: Intel NUC 12 Pro (Intel Core i7-1260P, 12-core/16-thread, 16 GB RAM)
* **3D LiDAR**: LSLiDAR C32 (32 channels, 10 Hz, 150 m range, ±3 cm ranging accuracy)
* **IMU**: Yahboom 10-axis IMU (200 Hz, roll/pitch accuracy 0.5°, yaw accuracy 0.5°–1.0°)
* **Dual-Redundant RTK Receivers**:
  * **Fusion RTK**: Beitian BT-982K2 dual-antenna receiver (10 Hz, Qianxun CORS `RTCM32GRCpro` mountpoint)
  * **Reference Ground-Truth RTK**: Beitian BT-982K2 dual-antenna receiver (10 Hz, Qianxun CORS `RTCM33GRCEJpro` mountpoint, physically independent baseline)
* **Tracked Chassis**: Dual-channel DC servo-driven tracked mobile robot (1350 × 820 × 520 mm, 300 kg payload, 30° gradeability, 48 V 120 Ah lithium battery providing 6–8 h continuous endurance, RS-232 interface at 115,200 bps)

---

## 4. Build & Installation

Clone and compile the repository within your ROS catkin workspace:

```bash
# 1. Create catkin workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# 2. Clone the repository
git clone https://github.com/yuja-ja/LIO-RTK-Orchard-Navigation.git .

# 3. Resolve dependencies
cd ~/catkin_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 4. Compile in Release mode
catkin_make -DCMAKE_BUILD_TYPE=Release

# 5. Source environment
source devel/setup.bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

---

## 5. Module 1: Robust LIO-RTK Localization (`/LIO-RTK`)

### 5.1 Methodology Highlights
1. **Direct Point-to-Plane ESIKF Update (Stage 1)**: Directly minimizes raw point-to-plane geometric residuals without fragile line/plane feature extraction, preserving disordered canopy topologies while IMU pre-integration ensures accurate scan deskewing.
2. **Asynchronous Sequential Soft-Anchoring (Stage 2)**: Estimates a smooth 4-DoF transformation ($T_{world}^{enu}$) between the local odometry frame and the global ENU frame. Unlike naive loose-coupling that directly overrides poses, this mechanism prevents violent chassis oscillations induced by discrete RTK jumps.
3. **Temporal Fault-Tolerant Gating**:
   * **Signal Dropout**: Gracefully degrades to local dead-reckoning, ensuring uncorrupted trajectory continuity.
   * **Multipath Bias**: A recursive Cumulative Sum (CUSUM) change-point detector dynamically inflates observation covariance to resist gradual pulling.
   * **Step Jumps & False-Fixed Solutions**: Joint physical Euclidean distance and Mahalanobis statistical distance gates isolate spurious fixed solutions with deceptively small reported covariances.

### 5.2 Mapping Execution
```bash
# Launch LIO-RTK online estimator with RViz visualization
roslaunch fast_lio_rtk rtk_mapping.launch rviz:=true

# In a separate terminal, play field dataset
rosbag play your_orchard_dataset.bag --clock
```
*Upon shutdown, the georeferenced 3D point cloud map is automatically exported to `PCD/scans.pcd`.*

---

## 6. Module 2: Semantic Mapping, Relocalization & Navigation (`/导航`)

### 6.1 Fruit-Tree Instance Semantic Mapping (`orchard_semantic_map`)
Extracts individual tree trunk instances and catalogs their georeferenced positions (validating Section 2.3.2):
```bash
roslaunch orchard_semantic_map generate_map.launch
```
* **Inputs**: Downsampled point cloud `scans.subsampled.pcd` and datum file `map_origin.txt`.
* **Pipeline**: 0.12 m grid elevation slicing $\to$ 15×15 sliding-window peak detection $\to$ local NMS with minimum inter-tree distance of 1.8 m $\to$ weighted centroid refinement.
* **Outputs**: `tree_inventory.csv` containing Tree IDs, local metric coordinates, and absolute WGS-84 coordinates (field validation: **96.59% precision**, **85.00% F1-score**, absolute coordinate RMSE **0.441 m** across 112 ground-truth trees).

### 6.2 3D Point-Cloud to 2D Safety Costmap (`pcd2gridmap`)
Converts raw 3D point clouds into navigation-ready 2D occupancy costmaps while filtering agricultural noise (validating Section 2.3.1):
* **0.00 – 0.15 m**: Ground terrain and low weeds (classified as traversable **Free** space).
* **0.15 – 1.20 m**: Physical tree trunks and rigid obstacles (classified as **Occupied** with 0.3 m safety dilation).
* **> 1.20 m**: Overhanging leaves and branches (completely cleared to prevent corridor false-closure).

```bash
roslaunch pcd2gridmap generate_map.launch pcd_file:=/path/to/scans.pcd
```
* **Outputs**: `orchard_map.yaml` and `orchard_map.pgm` saved to `fast_lio_navigation/maps/`.

### 6.3 Map-Based Global Relocalization (`FAST_LIO-RTK`)
Performs global pose recovery against the pre-built 3D point cloud map (validating Section 2.4):
```bash
roslaunch fast_lio_rtk relocation.launch map_file:=/path/to/site_map.pcd rviz:=true
```
* **Read-Only Static ikd-Tree**: Prevents transient registration errors from polluting the global map.
* **Point-to-Plane ICP + RTK Seeding**: Coarse initialization via RTK fix followed by ICP convergence verification (>= 50 correspondences and RMSE < 0.30 m for 3 consecutive frames).
* **Decoupled Slerp Interpolation**: Rate-limits global pose corrections onto `map -> odom`, keeping high-frequency local odometry strictly uninterrupted.

### 6.4 Hierarchical Autonomous Navigation (`fast_lio_navigation`)
Executes full-domain autonomous corridor inspection tailored for tracked UGVs (validating Section 2.5):
```bash
roslaunch fast_lio_navigation navigation.launch
```
* **Corridor Centerline Global Guidance (Reshaped Dijkstra)**: Reshapes costmap gradients via a repulsive potential field, forcing paths down the center of tree rows (increasing minimum clearance from 0.07 m to 0.38 m, preventing trunk scraping).
* **Tracked Kinematic TEB Local Optimization**: Incorporates nonholonomic differential skid-steering constraints with lightweight tuning (iteration count = 3, look-ahead = 3 m) to dynamically circumnavigate sudden obstacles (e.g., free-range poultry, irrigation hoses).
* **Low-Level Motor Control (`velocity_controller_node`)**: Transmits linear and angular velocity commands via RS-232 serial protocol with 0.005 deadband compensation.
* **Android Supervisory Teleoperation (`app_ros_bridge_node`)**: Bridges ROS telemetry to Android mobile terminals via WebSocket (port 9090) for real-time electronic geofencing, waypoint dispatch, and emergency braking.

---

## 7. Coordinate Frames & ROS Interfaces

### 7.1 Coordinate Frame Transformations (TF Tree)
The system strictly enforces a decoupled three-layer TF tree architecture:
```text
map (Global absolute geodetic datum / ENU)
 └── odom (Local smooth continuous odometry, driven by LIO, zero jumps)
      └── base_link (Tracked chassis geometric center)
           ├── body (IMU coordinate origin)
           ├── camera_init / lidar (LiDAR optical center)
           └── rtk_antenna (Main RTK antenna phase center with lever arm)
```

### 7.2 Key ROS Topics

| Topic | Message Type | Description |
| :--- | :--- | :--- |
| `/Odometry` | `nav_msgs/Odometry` | Local continuous high-frequency odometry (`odom -> base_link`) for motor control |
| `/Odometry_global` | `nav_msgs/Odometry` | Georeferenced absolute position (`map -> base_link`) for global navigation and mapping |
| `/localization/status`| `std_msgs/String` | System operational state (`WAITING_RTK`, `RELOCALIZING`, `TRACKING`, `DEGRADED`) |
| `/localization/valid` | `std_msgs/Bool` | Relocalization validity flag (`true` when locked) |
| `/cloud_registered` | `sensor_msgs/PointCloud2` | Deskewed point cloud registered in the global frame |
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity command generated by the navigation stack |
| `/move_base_simple/goal`| `geometry_msgs/PoseStamped`| Navigation goal input (dispatched via RViz or Android APP) |

---

## 8. Dual-RTK Ground-Truth Protocol & Degradation Benchmark

To prevent the methodological tautology of evaluating an RTK-fused trajectory against its own input data, this study deployed a **physically independent dual-RTK protocol** (validating Section 2.6 and Section 3.1):

1. **Independent Reference Benchmark**:
   * Two identical Beitian BT-982K2 receivers were mounted with calibrated 3D lever arms.
   * The fusion receiver used `RTCM32GRCpro`, while the reference receiver independently subscribed to `RTCM33GRCEJpro`.
   * Reference epochs were quality-gated offline (carrier-phase fixed solutions, $N_{sat} \ge 10$, $\text{PDOP} \le 3.5$, age $\le 5.0$ s, horizontal error ellipse semi-major axis $\sigma_{horiz} \le 0.05$ m, 5-frame streak). Stationary trials confirmed sub-centimeter repeatability (CEP95 of 0.77–0.97 cm).
2. **Controlled Degradation Suite**:
   To systematically evaluate fault tolerance under reproducible conditions, the input stream (`/rtk/*`) was injected with four synthetic failure modes (15% dataset duration across 3 random seeds), while `/rtk_truth/*` remained pristine:
   * **Dropout (20 s)**: Simulates complete satellite loss-of-lock beneath dense canopies.
   * **Bias (0.50 m)**: Simulates slow-varying multipath drift with a 0.03 Hz low-frequency ripple.
   * **Jump (1.00 m)**: Simulates sudden spatial steps caused by multipath reflections.
   * **False-Fixed (0.80 m)**: Injects horizontal displacement while deceptively maintaining fixed status and unscaled covariance, replicating integer ambiguity errors.

---

## 9. Quick Start Guide

### Step 1: Execute LIO-RTK Mapping
```bash
roslaunch fast_lio_rtk rtk_mapping.launch
rosbag play orchard_dataset.bag --clock
```

### Step 2: Generate Tree Inventory & 2D Costmap
```bash
# Extract individual-tree WGS-84 coordinates
roslaunch orchard_semantic_map generate_map.launch

# Slice point cloud into 2D navigation costmap
roslaunch pcd2gridmap generate_map.launch pcd_file:=/path/to/scans.pcd
```

### Step 3: Launch Prior-Map Relocalization
```bash
roslaunch fast_lio_rtk relocation.launch map_file:=/path/to/scans.pcd
```

### Step 4: Run Autonomous Corridor Navigation
```bash
roslaunch fast_lio_navigation navigation.launch
```

---

## 10. Citation & Acknowledgments

This framework and its underlying algorithms correspond to our submitted research manuscript, **currently under peer review**:

### BibTeX Citation
```bibtex
@article{lyu2025liortk,
  title={A LiDAR-Inertial-RTK Fusion Localization and Fruit-Tree Instance Mapping System for Tracked Inspection Robots in Non-Standardized Orchards},
  author={Lyu, Shilei and Gao, Songmao and Zhang, Guoning and He, Junxing and Chen, Leyuan and Gao, Peng and Li, Zhen},
  journal={Computers and Electronics in Agriculture},
  year={2025},
  note={Under Review}
}
```

*Note: Once formally accepted, publication metadata (volume, issue, and DOI) will be promptly updated.*

### Funding Acknowledgments
This work was supported in part by:
* **National Natural Science Foundation of China (NSFC)**: Grant No. `32271997`
* **Key Technologies R&D Program of Guangdong Province**: Grant No. `2023B0202100001`
* **Guangzhou Key Research and Development Program**: Grant No. `2024B03J1309`
* **Earmarked Fund for China Agriculture Research System of MARA (CARS)**: Grant No. `CARS-26`

### Open-Source Acknowledgments
We express our sincere gratitude to the developers of the following open-source frameworks:
* [FAST-LIO2](https://github.com/hku-mars/FAST_LIO) & [ikd-Tree](https://github.com/hku-mars/ikd-Tree) (HKU MARS Lab)
* [TEB Local Planner](http://wiki.ros.org/teb_local_planner) (TU Dortmund)
* [rosbridge_suite](http://wiki.ros.org/rosbridge_suite) (Robot Web Tools)

---

## 11. Demonstration Videos & Data Availability

* 📺 **Field Demonstration & Navigation Videos**:
  * **YouTube**: [https://www.youtube.com/watch?v=52Jc8SSe3pU](https://www.youtube.com/watch?v=52Jc8SSe3pU)
  * **Bilibili**: [https://www.bilibili.com/video/BV1WUEQ6TEqa](https://www.bilibili.com/video/BV1WUEQ6TEqa)
* 💾 **Dataset Availability**:
  * All 8 field evaluation sequences (including normal open-sky trajectories and controlled degradation suites) will be made publicly available upon formal paper acceptance.
  * During the peer-review process, sample datasets are accessible upon reasonable request to the corresponding authors.
