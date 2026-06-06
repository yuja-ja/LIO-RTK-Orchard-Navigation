#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
from scipy.spatial.transform import Rotation as R
import math

def read_tum(file_path):
    """读取 TUM 格式数据，返回 {时间戳: 4x4变换矩阵} 字典和时间戳列表"""
    poses = {}
    timestamps = []
    with open(file_path, 'r') as f:
        for line in f:
            if line.startswith('#') or not line.strip():
                continue
            parts = list(map(float, line.strip().split()))
            t = parts[0]
            # TUM 格式: tx ty tz qx qy qz qw
            trans = np.array(parts[1:4])
            quat = np.array(parts[4:8])
            
            # 构建 4x4 变换矩阵
            rot_matrix = R.from_quat(quat).as_matrix()
            T = np.eye(4)
            T[:3, :3] = rot_matrix
            T[:3, 3] = trans
            
            poses[t] = T
            timestamps.append(t)
    return poses, np.array(timestamps)

def find_nearest_time(target_t, time_array, threshold=0.15):
    """在时间数组中寻找最近的时间戳，如果时间差大于 threshold，则认为未找到 (模拟信号丢失)"""
    idx = np.searchsorted(time_array, target_t)
    # 检查左边和右边哪个更近
    candidates = []
    if idx < len(time_array):
        candidates.append(time_array[idx])
    if idx > 0:
        candidates.append(time_array[idx-1])
        
    if not candidates:
        return None
        
    nearest_t = min(candidates, key=lambda x: abs(x - target_t))
    if abs(nearest_t - target_t) <= threshold:
        return nearest_t
    return None

def main():
    # ================== 文件路径配置 ==================
    rtk_file = "/home/gsm/LIO-RTK/50-Z/degraded_rtk.txt"      # B: 带有周期性遮蔽的 RTK
    lio_file = "/home/gsm/LIO-RTK/50-Z/FAST-LIO.txt"    # A: 纯 FAST-LIO2 轨迹
    gt_file = "/home/gsm/LIO-RTK/50-Z/rtk_truth.txt"       # C: 真实的高精度 RTK 轨迹
    output_file = "/home/gsm/LIO-RTK/50-Z/naive_fusion.txt"   # 输出的朴素融合轨迹
    # ==================================================

    print("正在加载轨迹数据...")
    rtk_poses, rtk_times = read_tum(rtk_file)
    lio_poses, lio_times = read_tum(lio_file)
    gt_poses, gt_times = read_tum(gt_file)

    fused_trajectory = []
    errors = []

    anchor_rtk_T = None
    anchor_lio_T = None

    print("正在执行朴素松耦合融合...")
    # 以 ground_truth 的时间轴为主轴进行评估
    for t_gt in gt_times:
        gt_T = gt_poses[t_gt]
        
        # 1. 尝试寻找当前时刻有效的 RTK 观测 (阈值0.15秒代表允许的数据延迟/不同步)
        t_rtk = find_nearest_time(t_gt, rtk_times, threshold=0.15)
        t_lio = find_nearest_time(t_gt, lio_times, threshold=0.1) # LIO频率高，阈值给小点

        current_pose_T = np.eye(4)

        if t_rtk is not None:
            # 状态 1: RTK 有效，直接使用 RTK 位姿 (硬重置)
            current_pose_T = rtk_poses[t_rtk]
            
            # 更新锚点，供后续丢失信号时进行推算
            anchor_rtk_T = current_pose_T
            if t_lio is not None:
                anchor_lio_T = lio_poses[t_lio]
                
        else:
            # 状态 2: RTK 丢失 (即处于那 10 秒的断锁期)
            if anchor_rtk_T is not None and anchor_lio_T is not None and t_lio is not None:
                # 使用 LIO 航位推算：计算相对运动
                current_lio_T = lio_poses[t_lio]
                # delta_T = inv(LIO_old) * LIO_new
                delta_T = np.linalg.inv(anchor_lio_T) @ current_lio_T
                # 应用到上一个 RTK 锚点上: Pose = RTK_old * delta_T
                current_pose_T = anchor_rtk_T @ delta_T
            else:
                # 如果一开始就没信号，或者 LIO 也没数据，就跳过不评估
                continue

        # --- 保存生成的轨迹 ---
        trans = current_pose_T[:3, 3]
        quat = R.from_matrix(current_pose_T[:3, :3]).as_quat()
        fused_trajectory.append([t_gt, trans[0], trans[1], trans[2], quat[0], quat[1], quat[2], quat[3]])

        # --- 计算与 Ground Truth 的误差 (Absolute Pose Error) ---
        gt_trans = gt_T[:3, 3]
        # 计算 3D 欧氏距离误差
        error = np.linalg.norm(trans - gt_trans)
        errors.append(error)

    # 保存文件
    with open(output_file, 'w') as f:
        for pt in fused_trajectory:
            f.write(f"{pt[0]:.6f} {pt[1]:.6f} {pt[2]:.6f} {pt[3]:.6f} {pt[4]:.6f} {pt[5]:.6f} {pt[6]:.6f} {pt[7]:.6f}\n")
    
    print(f"融合完成！结果已保存至: {output_file}")

    # 计算统计数据
    rmse = math.sqrt(sum(e**2 for e in errors) / len(errors))
    mean_error = np.mean(errors)
    std_error = np.std(errors)
    max_error = np.max(errors)

    print("\n" + "="*40)
    print("定量评估结果 (朴素松耦合 RTK+LIO Baseline):")
    print(f"参与评估的有效帧数: {len(errors)}")
    print(f"RMSE (均方根误差): {rmse:.3f} m")
    print(f"Mean (平均误差)  : {mean_error:.3f} m")
    print(f"STD (标准差)     : {std_error:.3f} m")
    print(f"Max (最大误差)   : {max_error:.3f} m")
    print("="*40)
    print("\n你可以将此结果添加到论文的 Table 2 中，作为 'Naive RTK-LIO' 列。")

if __name__ == "__main__":
    main()