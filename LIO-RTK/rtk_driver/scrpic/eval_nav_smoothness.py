#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import math
from scipy.spatial.transform import Rotation as R

def read_trajectory(file_path):
    """读取实际轨迹，提取时间、位置和四元数"""
    times, pts, quats = [], [], []
    with open(file_path, 'r') as f:
        for line in f:
            if line.startswith('#') or not line.strip():
                continue
            parts = line.strip().split()
            # TUM 格式: timestamp x y z qx qy qz qw
            t = float(parts[0])
            x, y = float(parts[1]), float(parts[2])
            qx, qy, qz, qw = float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])
            
            times.append(t)
            pts.append([x, y])
            quats.append([qx, qy, qz, qw])
            
    return np.array(times), np.array(pts), np.array(quats)

def evaluate_dynamic_navigation(times, trajectory, quats):
    if len(trajectory) < 10:
        return
    
    # ================= 1. 路径长度与闭环判断 =================
    start_pt = trajectory[0]
    end_pt = trajectory[-1]
    shortest_straight_dist = np.linalg.norm(end_pt - start_pt)
    
    actual_length = 0.0
    for i in range(1, len(trajectory)):
        actual_length += np.linalg.norm(trajectory[i] - trajectory[i-1])
        
    # 判断是否为闭环 (起点终点距离 < 5米，且总长度 > 20米)
    is_closed_loop = (shortest_straight_dist < 5.0) and (actual_length > 20.0)

    # ================= 2. 轨迹平滑度 (利用真实四元数) =================
    yaws = []
    # 将四元数转换为欧拉角 (roll, pitch, yaw)
    for q in quats:
        # scipy 默认格式为 [x, y, z, w]
        rot = R.from_quat(q)
        # zyx 对应的欧拉角，索引 0 即为 yaw
        yaw = rot.as_euler('zyx', degrees=False)[0] 
        yaws.append(yaw)
        
    yaw_rates = []
    valid_frames = 0
    
    for i in range(1, len(yaws)):
        dt = times[i] - times[i-1]
        # 跳过时间差极小(高频重复)的帧，避免除以0或噪声放大
        if dt < 0.05: 
            continue
            
        dyaw = yaws[i] - yaws[i-1]
        # 归一化到 [-pi, pi] 之间，处理 360度 跳变
        dyaw = (dyaw + math.pi) % (2 * math.pi) - math.pi
        
        yaw_rate = abs(dyaw / dt)
        
        # 过滤掉极端异常的角速度 (比如履带车不可能每秒转超过 2 rad = 114度)
        if yaw_rate < 3.0: 
            yaw_rates.append(yaw_rate)
            valid_frames += 1

    mean_yaw_rate = np.mean(yaw_rates)
    std_yaw_rate = np.std(yaw_rates)

    print("="*50)
    print("基于动态代价地图的自主导航定量评估:")
    if is_closed_loop:
        print(f"轨迹类型         : 闭环/环形轨迹 (Figure-8 / Ring)")
        print(f"实际行驶总长度   : {actual_length:.3f} m")
        print(f"路径效率比       : N/A (闭环轨迹不适用此指标)")
    else:
        print(f"轨迹类型         : 开放轨迹 (如：长直道/单点导航)")
        print(f"理论最短直线距离 : {shortest_straight_dist:.3f} m")
        print(f"实际平滑绕行长度 : {actual_length:.3f} m")
        efficiency_ratio = actual_length / shortest_straight_dist if shortest_straight_dist > 0 else 1.0
        print(f"动态路径效率比   : {efficiency_ratio:.3f} (接近1.05-1.20为极佳)")
        
    print("-" * 50)
    print(f"评估有效帧数     : {valid_frames}")
    print(f"平均航向变化率   : {mean_yaw_rate:.3f} rad/s")
    print(f"航向平滑度 (Std) : {std_yaw_rate:.3f} rad/s (极低表明无控制震荡)")
    print("="*50)

if __name__ == "__main__":
    # 填入你实际行驶的轨迹 txt 文件
    TRAJ_FILE = "/home/gsm/LIO-RTK/50-Z/FAST-LIO-RTK.txt" 
    times, trajectory, quats = read_trajectory(TRAJ_FILE)
    evaluate_dynamic_navigation(times, trajectory, quats)