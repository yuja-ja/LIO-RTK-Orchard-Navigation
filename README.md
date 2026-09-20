# LIO-RTK: Soft-Anchored LiDAR-Inertial-RTK Fusion Localization for Tracked Orchard Robots

[![Ubuntu](https://img.shields.io/badge/OS-Ubuntu%2020.04-orange.svg)](https://releases.ubuntu.com/20.04/)
[![ROS](https://img.shields.io/badge/ROS-Noetic-blue.svg)](http://wiki.ros.org/noetic)
[![Paper](https://img.shields.io/badge/Status-Under%20Review%20(COMPAG)-green.svg)]()
[![Demo-Video](https://img.shields.io/badge/Video-YouTube%20%7C%20Bilibili-red.svg)](#video-demonstrations)

This repository hosts the **LIO-RTK** state estimation and localization framework developed for tracked unmanned ground vehicles (UGVs) operating in complex, non-standardized orchard environments with dense canopy cover.

---

## 1. Overview & Framework

In modern dense orchards, standard GNSS/RTK signals suffer frequent multipath degradation, lock loss, and signal outages beneath tree crowns, while pure LiDAR-Inertial Odometry (LIO) inevitably accumulates long-term drift. 

**LIO-RTK** resolves this dilemma through a **soft-anchored asynchronous sequential error-state Kalman filter (ESIKF)**:
* **High-Frequency LIO State Propagation**: Real-time high-rate continuous state estimation built upon iterative Kalman filtering with `ikd-Tree`.
* **Soft-Anchored RTK Correction**: Asynchronous, sequential absolute coordinate updates that mitigate discrete position jumps even during sudden RTK re-convergence or covariance transitions.
* **Canopy Drift Suppression**: Maintains consistent global state estimation and suppresses long-term drift throughout extended multi-row inspection routes.

---

## 2. Reviewer Notice & Open-Source Release Policy

> **To Academic Reviewers:**  
> To protect academic priority prior to formal publication, the underlying proprietary C++ implementation files of the sequential ESIKF update node have been temporarily withheld.  
> 
> However, to guarantee **100% transparency and reproducibility**, this repository provides:
> 1. Complete ROS package structures, launch configurations, and parameter definitions.
> 2. Full sensor transformation setups and calibration templates.
> 3. Real-world benchmark ROS bag datasets (under dense canopy conditions).
> 4. Verification and evaluation scripts replicating **Table 3**, **Table 4**, and **Figure 8** in the manuscript.
> 
> **The complete, uncompressed C++ source code will be made publicly available immediately upon final acceptance of the manuscript.**

---

## 3. Video Demonstrations

Real-world field trials and continuous localization under dense canopies can be viewed at:
* **YouTube Demo**: [Field Inspection & Localization Video](https://www.youtube.com/watch?v=52Jc8SSe3pU)
* **Bilibili (Mainland Mirror)**: [果园履带机器人自主导航与定位验证](https://www.bilibili.com/video/BV1WUEQ6TEqa?t=22.6)

---

## 4. Prerequisites & Dependencies

The code is developed and verified on **Ubuntu 20.04 (LTS)** with **ROS Noetic**.

* **ROS Dependencies**:
  ```bash
  sudo apt-get install -y ros-noetic-cv-bridge ros-noetic-tf ros-noetic-message-filters \
                          ros-noetic-image-transport ros-noetic-geographic-msgs
  ```
* **Eigen** >= 3.3.4
* **PCL (Point Cloud Library)** >= 1.8
* **Sophus & Ceres Solver**:
  ```bash
  sudo apt-get install -y libceres-dev
  ```
* **EVO Evaluation Tool** (for reproducing trajectory metrics):
  ```bash
  pip install evo --upgrade --no-binary evo
  ```

---

## 5. Build & Installation

Clone this repository into your ROS catkin workspace:

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/YourUsername/LIO-RTK.git
cd ..
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

---

## 6. Quick Start & Dataset Evaluation

### 6.1 Sample Dataset Download
We provide a representative field ROS bag recorded in an actual non-standardized orchard with severe canopy occlusion:
* **Google Drive Link**: [Download Sample Orchard Bag (sample_orchard.bag)](https://drive.google.com/file/d/1xAxKd2RzwZJSzw48yX8yVTadO8iQPJ0n/view?usp=sharing)

### 6.2 Running LIO-RTK
Launch the localization and visualization interface:
```bash
# Terminal 1: Launch LIO-RTK node and RViz visualization
roslaunch lio_rtk run_orchard.launch

# Terminal 2: Play the experimental ROS bag
rosbag play sample_orchard.bag --clock
```

### 6.3 Reproducing Paper Results

To assist reviewers in verifying our quantitative evaluations, automated reproduction scripts are provided in `scripts/`:

```bash
# 1. Reproduce Trajectory Accuracy against Baselines (Manuscript Table 3)
python3 scripts/eval_trajectory_accuracy.py --est_path data/trajectories/lio_rtk.txt --gt_path data/ground_truth/gt_trajectory.txt

# 2. Reproduce Synthetic GNSS Degradation & Outage Experiments (Manuscript Table 4 & Figure 8)
python3 scripts/eval_degraded_gnss_simulation.py --bag sample_orchard.bag --outage_duration 10.0
```

---

## 7. Configuration Guide

All sensor topics, coordinate transforms, and fusion thresholds can be adjusted in `config/lio_rtk_params.yaml`:

```yaml
lio_rtk:
  # Sensor Topics
  lidar_topic: "/velodyne_points"
  imu_topic: "/imu/data"
  rtk_topic: "/fix"
  
  # Soft-Anchoring ESIKF Settings
  rtk_position_cov_threshold: 0.15     # Minimum accuracy threshold to accept RTK update (m)
  adaptive_noise_scaling: true         # Enable CUSUM-based measurement noise scaling
  gravity_align: true
```

---

## 8. Citation

If you find this work or the dataset helpful in your research, please consider citing our paper:

```bibtex
@article{li2025liortk,
  title={LiDAR-Inertial-RTK Fusion Localization and Fruit-Tree Semantic Mapping for Autonomous Inspection Robots in Non-Standardized Orchards},
  author={Li, Author and Colleagues},
  journal={Computers and Electronics in Agriculture},
  year={2025},
  note={Under Review}
}
```

---

## 9. Contact & Acknowledgements

For technical inquiries or data sharing regarding this paper, please open an issue in this repository or contact the corresponding author via email.
