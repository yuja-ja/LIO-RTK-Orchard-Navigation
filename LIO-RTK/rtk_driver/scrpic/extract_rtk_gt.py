#!/usr/bin/env python3

import rosbag
import os
import sys
import numpy as np
from scipy.spatial.transform import Rotation as R

# ==================== 配置区 ====================
BAG_FILE_PATH = "/home/gsm/LIO-RTK/20-8/20-8.bag"
OUTPUT_GT_PATH = "/home/gsm/LIO-RTK/20-8/rtk_truth_lio.txt"
ODOM_TOPIC = "/rtk/odom_truth"

# 【关键新增】：RTK天线相对于IMU中心的外参平移 [X, Y, Z] 单位：米
# 对应你的 FAST-LIO2 配置: extrinsic_T: [0.34, -0.42, 0.20]
EXTRINSIC_T = np.array([0.10, 0.42, -0.15])
# ==============================================

def extract_rtk_groundtruth(bag_path, output_path, topic):
    if not os.path.isfile(bag_path):
        print(f"[ERROR] Bag file not found: {bag_path}")
        sys.exit(1)

    output_dir = os.path.dirname(output_path)
    if output_dir and not os.path.exists(output_dir):
        os.makedirs(output_dir)

    poses_extracted = 0
    try:
        with rosbag.Bag(bag_path, 'r') as bag, open(output_path, 'w') as f_out:
            for topic_name, msg, t in bag.read_messages(topics=[topic]):
                stamp = msg.header.stamp
                timestamp = stamp.to_sec()
                if timestamp <= 0:
                    continue

                # 1. 获取 RTK 天线在世界坐标系下的位置和姿态
                pos = msg.pose.pose.position
                ori = msg.pose.pose.orientation

                t_WR = np.array([pos.x, pos.y, pos.z])            # RTK位置
                quat = [ori.x, ori.y, ori.z, ori.w]               # RTK姿态四元数

                # 2. 【核心补偿】：将 RTK 轨迹投影到 IMU 中心处
                try:
                    # 将四元数转换为旋转矩阵
                    rot_matrix = R.from_quat(quat).as_matrix()
                    
                    # 公式：P_imu = P_rtk - R_rtk * T_extrinsic
                    # 将外参向量转换到世界坐标系下，然后从RTK位置中减去
                    t_WI = t_WR - rot_matrix.dot(EXTRINSIC_T)
                except ValueError:
                    # 防止由于四元数全为0导致的异常情况，遇到异常不做补偿
                    t_WI = t_WR 

                # 3. 写入文件（格式：时间戳 x y z qx qy qz qw），使用的是补偿后的 IMU 位置
                line = f"{timestamp:.9f} {t_WI[0]:.6f} {t_WI[1]:.6f} {t_WI[2]:.6f} " \
                       f"{ori.x:.6f} {ori.y:.6f} {ori.z:.6f} {ori.w:.6f}\n"
                f_out.write(line)
                poses_extracted += 1

    except Exception as e:
        print(f"[ERROR] Failed to process bag: {e}")
        sys.exit(1)

    print(f"[SUCCESS] Extracted {poses_extracted} poses with Extrinsic Compensation.")
    print(f"[SUCCESS] Applied Extrinsic offset: X={EXTRINSIC_T[0]}, Y={EXTRINSIC_T[1]}, Z={EXTRINSIC_T[2]}")
    print(f"[SAVE] Ground truth saved to: {output_path}")

if __name__ == '__main__':
    extract_rtk_groundtruth(BAG_FILE_PATH, OUTPUT_GT_PATH, ODOM_TOPIC)