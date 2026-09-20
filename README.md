# A LiDAR-Inertial-RTK Fusion Algorithm for Robust Localization, Semantic Mapping, and Autonomous Navigation of Tracked Vehicles in Non-Standardized Orchards

> **面向非标准化果园履带车的激光惯导-RTK融合鲁棒定位、语义建图与自主导航系统**  
> *Developed by South China Agricultural University (华南农业大学) & Pazhou Lab (琶洲实验室)*

[![ROS](https://img.shields.io/badge/ROS-Noetic%20%2F%20Melodic-blue.svg)](http://wiki.ros.org/)
[![C++](https://img.shields.io/badge/Language-C%2B%2B14-brightgreen.svg)](https://isocpp.org/)
[![Python](https://img.shields.io/badge/Python-3.8%2B-yellow.svg)](https://www.python.org/)
[![Status](https://img.shields.io/badge/Paper_Status-Under_Review-orange.svg)](#10-论文引用与致谢)
[![Video](https://img.shields.io/badge/Video_Demo-Bilibili%20%7C%20YouTube-red.svg)](#11-演示视频与数据开源)

本开源项目专为**非标准化丘陵山地及自然杂乱果园环境**下的履带式无人移动作业底盘（UGV）设计，提供了一套涵盖 **“高鲁棒激光惯导-RTK紧耦合定位建图”**、**“果树单株语义实例资产提取”**、**“高程滤波点云转2D导航代价栅格”**、**“全局先验点云地图重定位”** 到 **“分层自主避障导航与移动端控制”** 的全流程开源软硬件闭环系统。

---

## 目录（Table of Contents）

- [1. 系统核心功能与架构](#1-系统核心功能与架构)
- [2. 仓库目录结构](#2-仓库目录结构)
- [3. 软硬件环境要求](#3-软硬件环境要求)
- [4. 编译与安装指南](#4-编译与安装指南)
- [5. 模块一：LIO-RTK 紧耦合定位建图算法](#5-模块一lio-rtk-紧耦合定位建图算法)
- [6. 模块二：果树语义实例建图与高程栅格切片](#6-模块二果树语义实例建图与高程栅格切片)
- [7. 模块三：全局先验地图重定位算法](#7-模块三全局先验地图重定位算法)
- [8. 模块四：分层自主避障导航与底盘控制](#8-模块四分层自主避障导航与底盘控制)
- [9. 坐标系架构与关键话题接口](#9-坐标系架构与关键话题接口)
- [10. 论文引用与致谢](#10-论文引用与致谢)
- [11. 演示视频与数据开源](#11-演示视频与数据开源)

---

## 1. 系统核心功能与架构

系统整体数据流与控制闭环如下图所示，主要分为两大核心功能模块：

```text
======================= 系统顶层数据流与控制闭环 =======================

[ 传感器硬件 ]  LSLiDAR C32 (32线) + 10轴 IMU + 双天线差分 RTK + 履带底盘
                      │                 │                │
                      ▼                 ▼                ▼
[ LIO-RTK 模块 ] ──► FAST-LIO2 前端 (ikd-Tree + IEKF 局部高频连续里程计解算)
                      │
                      ├──► 异步软锚定更新 (4-DoF 外参卡尔曼滤波：平移 + 偏航角)
                      ├──► 时序一致性容错门控 (抗 Dropout 丢星 / Jump 跳变 / Bias 多径偏差 / False-fixed 假固定解)
                      │
                      ▼
[ 地图生成流水线 ] ──► 1. 高精度稠密 3D 点云地图 (PCD)
                      │
                      ├──► orchard_semantic_map: 地面自适应分割 + 局部 NMS 提取果树质心 -> tree_inventory.csv
                      └──► pcd2gridmap: 树干高程切片 (0.5m ~ 1.2m) 滤除杂草树冠 -> 2D 占用栅格地图 (PGM/YAML)
                      │
                      ▼
[ 全局重定位 ] ─────► FAST-LIO-RTK Relocalization: 静态 ikd-Tree 粗/精 ICP 匹配 + RTK 初始位姿驱动
                      │ (严格维护 map -> odom -> base_link 三层 TF 树，保证全局校正不产生控制跳变)
                      │
                      ▼
[ 自主导航与交互 ] ─► fast_lio_navigation:
                      ├──► 全局路径规划 (Global Planner / 农机行间中线引导)
                      ├──► 局部避障规划 (TEB / DWA Local Planner，针对履带底盘非完整运动约束优化)
                      ├──► 底盘控制驱动 (velocity_controller_node 串口下发 /cmd_vel)
                      └──► 移动端交互桥接 (app_ros_bridge_node + rosbridge_server WebSocket)
```

### 主要特性亮点：
1. **全工况 RTK 时序容错保障**：针对果园树冠遮挡导致的多路径慢变偏差（Multipath Bias）、突发阶跃跳变（Step Jump）、高置信度假固定解（False-Fixed）及长达数十秒的信号失锁断流（Dropout），建立了滑动窗口容错门控、动态协方差前置白化与双向 CUSUM 趋势检测机制，实现全局平滑且无漂移的位姿输出。
2. **免大算力深度学习的果树几何语义实例建图**：提出基于点云高程解耦与局部非极大值抑制（NMS）的果树单株实例提取算法，无需高端 GPU 即可从数亿点云中稳健提取果树主干中心与空间三维质心，自动生成全球绝对 WGS-84 坐标编码的果园树木资产清单。
3. **农机专用高程滤波切片**：针对果园地面杂草丛生、泥地不平整及低垂树冠的特点，通过双高度带切片技术（保留 0.5m~1.2m 物理树干，剔除 0~0.5m 低矮杂草及 >1.2m 低垂树冠），根除虚假避障与道路假性闭塞问题。
4. **边缘端轻量化与移动交互闭环**：全系统在 Intel NUC12Pro 边缘平台上各核心模块内存占用均稳定在 300MB 以内；集成 WebSocket 桥接节点，支持作业人员通过安卓手机/平板 APP 实施电子围栏标定、航点下发与远程接管。

---

## 2. 仓库目录结构

本仓库分为两个核心目录：`LIO-RTK`（紧耦合定位建图）与 `导航`（语义地图、栅格投影、重定位与避障导航）。

```bash
.
├── LIO-RTK/                      # 核心模块 1：激光惯导-RTK紧耦合定位建图系统
│   └── src/
│       ├── LIO-RTK/              # LIO-RTK 算法主包 (基于 FAST-LIO2 深度扩展)
│       │   ├── config/           # 传感器外参及 RTK 融合配置文件 (lslidar_rtk.yaml 等)
│       │   ├── launch/           # 建图启动入口脚本 (rtk_mapping.launch 等)
│       │   ├── include/          # ikd-Tree、IKFoM 几何流形及 RTK 异步融合头文件
│       │   └── src/              # laserMapping.cpp, rtk_fusion.cpp 等核心实现
│       ├── nmea_rtk_driver/      # NMEA 双天线 RTK 接收机驱动与时间对齐同步节点
│       ├── lslidar_driver/       # 镭神 C32 机械式 32 线激光雷达驱动
│       ├── livox_ros_driver/     # Livox 固态激光雷达驱动 (兼容器材)
│       ├── imu_ros_driver/       # 10 轴高频 IMU 驱动与姿态预处理节点
│       └── rtk_truth_tum/        # 双 RTK 独立参考真值记录与 TUM 格式评测导出工具
│
└── 导航/                         # 核心模块 2：语义建图、重定位与自主导航控制系统
    ├── FAST_LIO-RTK/             # 固定地图全局重定位包 (静态 ikd-Tree 点到面/点到点 ICP + RTK)
    ├── orchard_semantic_map/     # 果树语义实例提取包 (输出 tree_inventory.csv)
    ├── pcd2gridmap/              # 3D PCD 点云向 2D 导航占用栅格地图高度切片转换包
    ├── fast_lio_navigation/      # 导航控制核心包 (move_base, TEB/DWA, 底盘串口, APP桥接)
    ├── ls2velo/                  # 镭神雷达点云格式转标准 Velodyne 格式中间件
    ├── lslidar_driver/           # 导航端激光雷达驱动包
    ├── rtk_driver/               # 导航重定位所用 RTK 接口驱动
    └── wit_imu_driver/           # 维特 10 轴惯性测量单元驱动
```

---

## 3. 软硬件环境要求

### 3.1 软件基础环境
* **操作系统**：Ubuntu 18.04 LTS (ROS Melodic) 或 Ubuntu 20.04 LTS (ROS Noetic)
* **核心编译工具**：CMake >= 3.0.2, GCC/G++ >= 7.5.0 (支持 C++14)
* **基础依赖库**：PCL >= 1.8, Eigen >= 3.3.4, OpenMP, Python 3.8+
* **ROS 官方扩展功能包安装**：
  ```bash
  sudo apt-get update
  sudo apt-get install ros-$ROS_DISTRO-navigation \
                       ros-$ROS_DISTRO-teb-local-planner \
                       ros-$ROS_DISTRO-rosbridge-server \
                       ros-$ROS_DISTRO-map-server \
                       ros-$ROS_DISTRO-costmap-2d
  ```

### 3.2 推荐硬件平台配置
* **车载工控机**：Intel Core i7/i5 工业车载计算机 (推荐 16GB RAM，如 Intel NUC12Pro)
* **激光雷达**：LSLiDAR C32 (32通道机械式激光雷达，10 Hz，测距 150m)
* **惯导单元**：10 轴高精度 IMU (>= 200 Hz，俯仰/横滚精度 0.5°)
* **差分定位**：北天 BT-982K2 双天线定向 RTK 接收机 (10 Hz，基于 CORS 网络差分服务)
* **移动作业底盘**：差速履带式果园作业平台 (额定载荷 300kg，爬坡能力 30°，RS-232 串口协议)

---

## 4. 编译与安装指南

建议在 Linux 用户主目录下统一构建工作空间（以 `catkin_ws` 为例）：

```bash
# 1. 创建 ROS 工作空间目录
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# 2. 将本仓库克隆或复制到工作空间 src 目录下
git clone https://github.com/yuja-ja/LIO-RTK-Orchard-Navigation.git .

# 3. 安装项目依赖
cd ~/catkin_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 4. 采用 Release 优化模式进行编译 (充分利用 OpenMP 多线程加速)
catkin_make -DCMAKE_BUILD_TYPE=Release

# 5. 刷新工作空间环境变量
source devel/setup.bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

---

## 5. 模块一：LIO-RTK 紧耦合定位建图算法

### 5.1 核心算法原理
针对复杂树冠遮挡下的 GNSS 信号衰减与多径效应，LIO-RTK 算法采用**异步分步状态更新机制（Asynchronous Sequential ESIKF）** 与 **软锚定（Soft-Anchoring）策略**：
1. **高频连续前端（Stage 1）**：继承 FAST-LIO2 直接法，利用高频 IMU（200Hz）进行状态前向传播和点云去畸变，结合动态增量式 `ikd-Tree` 快速构建点到平面残差并执行 IEKF 更新，维持车体高频平滑位姿；
2. **异步软锚定更新（Stage 2）**：当接收到低频 RTK（10Hz）观测后，在线解析 NMEA GST 语句构造测量噪声协方差；经过欧氏物理距离与马氏统计距离双重门限检验后，在后台异步估计并平滑更新局部里程计坐标系（`camera_init`）到全局 ENU 坐标系的刚体变换，杜绝传统松耦合方法因 RTK 跳变而造成的底盘剧烈失控晃动；
3. **完全退化支撑**：当检测到 RTK 丢星断流或严重多径时，系统平滑退化至纯 LIO 模式，凭借局部几何地图维持亚米级精度的连续外推。

### 5.2 运行建图命令
```bash
# 1. 启动 LIO-RTK 建图节点 (启动 RViz 可视化界面)
roslaunch fast_lio_rtk rtk_mapping.launch rviz:=true

# 2. 另开终端回放果园实测 rosbag (雷达点云、IMU、RTK 原始话题)
rosbag play your_orchard_dataset.bag --clock

# 3. 保存 3D 点云地图
# 建图节点正常退出时，会将全局点云自动写入 PCD 目录下的指定文件 (如 scans.pcd)
```

---

## 6. 模块二：果树语义实例建图与高程栅格切片

### 6.1 果树单株语义实例提取 (`orchard_semantic_map`)
利用离线工具链对稠密 3D 点云进行几何解耦，提取单株果树位置并生成数字农业资产清单：
```bash
# 启动语义实例提取节点
roslaunch orchard_semantic_map generate_map.launch
```
* **核心输入**：建图生成的降采样点云 `scans.subsampled.pcd` 与原点文件 `map_origin.txt`；
* **算法逻辑**：点云网格化高程差计算 $\to$ 局部滑动窗口寻峰 $\to$ 强特征点加权质心提取 $\to$ 距离门限非极大值抑制（NMS）；
* **核心输出**：在 `tree_csv/tree_inventory.csv` 中生成包含果树唯一编号（ID）、局部平面坐标、绝对 WGS-84 经纬度及树冠几何半径的清单文件。

### 6.2 点云向 2D 占用栅格地图投影 (`pcd2gridmap`)
为消除杂草与低垂枝条干扰，通过高度切片生成供导航使用的 2D 代价栅格地图：
* **0 ~ 0.5 m**：判定为近地杂草或平整可通行路面（标记为 Free 空闲）；
* **0.5 ~ 1.2 m**：判定为果树主干及物理障碍物（标记为 Occupied 占用）；
* **> 1.2 m**：高位果树树冠直接过滤舍弃，防止造成作业道路假闭塞。

```bash
# 启动高度切片与栅格地图生成
roslaunch pcd2gridmap generate_map.launch pcd_file:=/path/to/scans.pcd
```
* **输出成果**：自动在 `fast_lio_navigation/maps/` 目录下生成 `orchard_map.yaml` 和 `orchard_map.pgm`。

---

## 7. 模块三：全局先验地图重定位算法

在二次作业或例行巡检时，机器人需在已建好的先验 PCD 地图中快速确定全局坐标。重定位模块结合了**静态 ikd-Tree 粗精 ICP 匹配**与 **RTK 初始位姿驱动**。

### 7.1 启动全局重定位
```bash
roslaunch fast_lio_rtk relocation.launch map_file:=/path/to/site_map.pcd rviz:=true
```

### 7.2 状态流转机制
系统在重定位过程中严格发布定位状态话题 `/localization/status`：
* `WAITING_RTK`：等待 RTK 获取有效卫星固定解或等待外部初始位姿给定；
* `RELOCALIZING`：利用初始位姿在静态 ikd-Tree 中进行点到平面/点到点 ICP 匹配；
* `TRACKING`：当连续通过匹配门限（默认 3 帧）后，发布 `/localization/valid = true`，进入高精度稳定追踪模式；
* `DEGRADED / LOST`：若雷达严重退化且 RTK 失锁，系统保持局部里程计不跳变，并提示导航端降低行驶速度或停车。

---

## 8. 模块四：分层自主避障导航与底盘控制

### 8.1 一键启动自主导航
在确认栅格地图与参数加载无误后，启动导航主节点：
```bash
roslaunch fast_lio_navigation navigation.launch
```

### 8.2 核心导航特性配置
1. **全局路径规划（农机中线引导）**：
   * 基于全局规划器对自由空间基础代价与障碍物膨胀代价进行梯度重塑，强制全局路径沿果树行间中央中线平滑延展，消除传统“最短路径算法”在狭窄果行内贴边擦撞树干的隐患；
2. **局部避障与轨迹优化（TEB Local Planner）**：
   * 建立履带底盘非完整约束与原地转向运动学模型；
   * 针对树间散养禽类、掉落果枝等突发动静态障碍物，实施快速弹性局部绕行，并在越过障碍后迅速回归行间中线；
3. **底盘速度控制节点 (`velocity_controller_node`)**：
   * 订阅导航输出的 `/cmd_vel` 话题；
   * 通过串口（`/dev/ttyUSB1`，波特率 115200）下发带有死区补偿与角速度线性比例校正的驱动报文；
4. **移动端 APP 交互桥接 (`app_ros_bridge_node`)**：
   * 基于 `rosbridge_websocket`（默认监听 9090 端口）；
   * 将当前位姿与全局航点双向转换为高德地图/WGS-84 坐标，作业人员可通过移动手持终端下发巡航路线并实时监控设备运行状态。

---

## 9. 坐标系架构与关键话题接口

### 9.1 TF 变换拓扑树（TF Tree）
系统遵循严格的解耦设计，全局位置修正仅作用于 `map -> odom`，绝不干扰导航所依赖的高频平滑 `odom -> base_link`：
```text
map (全局地理绝对参考系 / ENU)
 └── odom (局部连续平滑里程计坐标系，由 LIO 驱动，无跳跃)
      └── base_link (履带底盘旋转中心)
           ├── body (IMU 安装中心坐标系)
           ├── camera_init / lidar (激光雷达点云坐标系)
           └── rtk_antenna (RTK 主天线相位中心，含杆臂补偿)
```

### 9.2 核心 ROS 话题列表

| 话题名称 (Topic) | 消息类型 (Message Type) | 说明 (Description) |
| :--- | :--- | :--- |
| `/Odometry` | `nav_msgs/Odometry` | 局部高频连续平滑里程计（`odom -> base_link`），供底层控制器高速闭环 |
| `/Odometry_global` | `nav_msgs/Odometry` | 全局地理绝对位姿（`map -> base_link`），供宏观规划与监控显示 |
| `/localization/status` | `std_msgs/String` | 系统当前定位状态（`TRACKING`, `RELOCALIZING`, `DEGRADED`, `LOST`） |
| `/localization/valid` | `std_msgs/Bool` | 全局重定位锁定标志位（锁定后为 `true`） |
| `/cloud_registered` | `sensor_msgs/PointCloud2` | 经运动畸变补偿并对齐至地图坐标系下的稠密实时点云 |
| `/cmd_vel` | `geometry_msgs/Twist` | 导航规划器计算生成的底盘线速度与角速度控制指令 |
| `/move_base_simple/goal`| `geometry_msgs/PoseStamped`| 2D 目标导航航点给定接口（支持 RViz 点击或手机端 APP 远程下发） |

---

## 10. 论文引用与致谢

本系统的核心算法成果已整理撰写为学术论文，**目前正处于审稿修订阶段（Under Review）**。如果您在科研、教学或工程开发中参考或使用了本仓库的算法与代码，请以如下格式客观引用我们的工作：

### 论文引用信息（BibTeX）
```bibtex
@article{lyu2024lidar,
  title={A LiDAR-Inertial-RTK Fusion Algorithm for Robust Localization, Semantic Mapping, and Autonomous Navigation of Tracked Vehicles in Non-Standardized Orchards},
  author={Lyu, Shilei and Gao, Songmao and Zhang, Guoning and He, Junxing and Chen, Leyuan and Gao, Peng and Li, Zhen},
  journal={Computers and Electronics in Agriculture},
  note={Under review},
  year={2024}
}
```

*注：论文正式录用发表后，我们将第一时间在本章节更新正式的 Volume、Issue 及 DOI 索引信息。*

### 项目资助（Funding Acknowledgments）
本研究与开源项目得到了以下科研项目的资助支持，特此致谢：
* **国家自然科学基金 (National Natural Science Foundation of China)**: 项目编号 `32271997`
* **广东省重点领域研发计划 (Key Technologies R&D Program of Guangdong Province)**: 项目编号 `2023B0202100001`
* **广州市重点研发计划 (Guangzhou Key Research and Development Program)**: 项目编号 `2024B03J1309`
* **国家现代农业产业技术体系专项资金 (Earmarked Fund for CARS)**: 项目编号 `CARS-26`

### 开源致谢（Open-source Acknowledgments）
特别感谢以下优秀开源算法与软件框架为本系统提供的底层技术支撑：
* [FAST-LIO2](https://github.com/hku-mars/FAST_LIO) & [ikd-Tree](https://github.com/hku-mars/ikd-Tree) (香港大学 MARS 实验室)
* [TEB Local Planner](http://wiki.ros.org/teb_local_planner) (TU Dortmund)
* [rosbridge_suite](http://wiki.ros.org/rosbridge_suite) (Robot Web Tools)

---

## 11. 演示视频与数据开源

为方便审稿专家及同行学者更直观地考察算法实车运行效果与复现结果，我们提供了以下开放资源：

* 📺 **实车试验与导航演示视频**：
  * **Bilibili**：[https://www.bilibili.com/video/BV1WUEQ6TEqa](https://www.bilibili.com/video/BV1WUEQ6TEqa)
  * **YouTube**：[https://www.youtube.com/watch?v=52Jc8SSe3pU](https://www.youtube.com/watch?v=52Jc8SSe3pU)
* 💾 **数据集获取说明**：
  * 论文评测所涉及的全部 8 个野外实测序列（包含正常工况与人工注入的丢星、多径偏差、跳变、假固定解等受控退化数据集）将在论文正式录用后全量公开。
  * 审稿期间如需获取脱敏抽样验证数据，欢迎通过论文通讯作者邮箱与研究团队联系获取。
