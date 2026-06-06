# LIO-RTK-Orchard-Navigation

This repository contains the source code, datasets, and demonstration videos for the paper focusing on robust LIO-RTK localization, semantic mapping, and autonomous navigation for tracked UGVs in non-standardized orchards. 

> **Note:** This repository has been carefully sanitized and anonymized to strictly comply with the **double-blind peer review** policy. 

## 1. Demonstration Videos
You can watch the actual vehicle autonomous navigation and obstacle avoidance tests in the non-standardized orchard here:
* [Link to Video 1 (e.g., Google Drive / YouTube anonymous link) ](把您的视频分享链接贴在这里)

## 2. Dataset
Due to GitHub's file size limits, the raw LiDAR-IMU-RTK datasets (ROS bag files) and the semantic mapping point clouds are hosted on an external drive.
* [Download the Dataset here](把您的网盘分享链接贴在这里)

## 3. Source Code Architecture
The code is currently organized into the following main modules:
* `Mapping_Module`: Contains the asynchronous sequential ESIKF fusion based on FAST-LIO2.
* `Navigation_Module`: Contains the hierarchical navigation framework with the reshaped costmap Dijkstra and TEB local planner.
* `Semantic_Extraction`: Algorithm for instance-level sub-pixel semantic mapping of orchard trees.

*(Detailed installation and running instructions will be fully updated upon publication.)*
