#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rosbag
import rospy
import sys

def process_bag(input_bag_path, output_bag_path, rtk_topic):
    # 参数设置：周期 30 秒 = 保持 20 秒 + 丢弃 10 秒
    keep_duration = 20.0  # 保留时间 (秒)
    drop_duration = 10.0  # 丢弃时间 (秒)
    cycle_duration = keep_duration + drop_duration

    first_rtk_time = None
    dropped_count = 0
    kept_count = 0

    print("开始处理 Bag 包...")
    print("输入: {}".format(input_bag_path))
    print("输出: {}".format(output_bag_path))
    print("目标 RTK Topic: {}".format(rtk_topic))
    print("规则: 每保留 {} 秒，丢弃 {} 秒".format(keep_duration, drop_duration))

    with rosbag.Bag(output_bag_path, 'w') as outbag:
        with rosbag.Bag(input_bag_path, 'r') as inbag:
            # 遍历原始包中的所有消息
            for topic, msg, t in inbag.read_messages():
                # 如果不是 RTK topic，原封不动直接写入新包 (雷达、IMU等)
                if topic != rtk_topic:
                    outbag.write(topic, msg, t)
                    continue
                
                # 如果是 RTK topic，执行时间周期判断逻辑
                current_time = t.to_sec() # 使用包记录的时间戳
                
                # 记录第一帧 RTK 出现的时间，作为时间轴零点
                if first_rtk_time is None:
                    first_rtk_time = current_time
                
                # 计算相对时间
                relative_time = current_time - first_rtk_time
                
                # 计算当前处于周期的哪个位置 (0 到 30 秒之间)
                cycle_time = relative_time % cycle_duration
                
                # 如果在保留阶段 (0 ~ 20秒)
                if cycle_time < keep_duration:
                    outbag.write(topic, msg, t)
                    kept_count += 1
                else:
                    # 在丢弃阶段 (20 ~ 30秒)，不执行 write 操作，模拟信号丢失
                    dropped_count += 1

    print("\n处理完成！")
    print("RTK 消息统计: 保留了 {} 帧， 丢弃了 {} 帧。".format(kept_count, dropped_count))
    print("新 Bag 包已保存至: {}".format(output_bag_path))

if __name__ == "__main__":
    # ================== 请在这里修改你的配置 ==================
    # 你的原始 bag 路径
    INPUT_BAG = "/home/gsm/LIO-RTK/20-8/20-8.bag" 
    
    # 你想生成的带有断锁模拟的新 bag 路径
    OUTPUT_BAG = "/home/gsm/LIO-RTK/20-8/degraded_rtk_orchard.bag" 
    
    # 你的 RTK 话题名称 (例如: /rtk/fix, /odometry/gps 等，请通过 rostopic list 确认)
    RTK_TOPIC = "/rtk/odom_truth" 
    # ==========================================================

    process_bag(INPUT_BAG, OUTPUT_BAG, RTK_TOPIC)