import rosbag
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.ticker as ticker  # ★ 引入 ticker 模块，用于加密刻度

# ================= 1. 数据读取与准备 =================
bag_path = '/home/gsm/rtk_b.bag'  # 替换为你的 bag 路径
topic_name = '/rtk/odom_truth'

x_list, y_list, std_list, time_list = [], [], [],[]
first_time = None

print(f"正在读取 {bag_path} ...")
with rosbag.Bag(bag_path, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=[topic_name]):
        if first_time is None:
            first_time = t.to_sec()
        x_list.append(msg.pose.pose.position.x)
        y_list.append(msg.pose.pose.position.y)
        var_x = msg.pose.covariance[0]
        var_y = msg.pose.covariance[7]
        std_list.append(np.sqrt(var_x + var_y))
        time_list.append(t.to_sec() - first_time)

x_arr = np.array(x_list)
y_arr = np.array(y_list)
std_arr = np.array(std_list)
t_arr = np.array(time_list)

# 寻找遮挡拐点
degrade_idx = np.where(std_arr > 0.5)[0][0] if np.any(std_arr > 0.5) else len(t_arr)//2
degrade_time = t_arr[degrade_idx]

# 分离数据：开阔地 (低漂移)
mask_low = t_arr < degrade_time

# ================= 2. 全局绘图参数设置 =================
plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = ['Times New Roman']
plt.rcParams['axes.labelsize'] = 14
plt.rcParams['xtick.labelsize'] = 12
plt.rcParams['ytick.labelsize'] = 12

def plot_spatial_scatter(ax, x, y, std, title):
    """通用空间散点图绘制函数"""
    sc = ax.scatter(x, y, c=std, cmap='jet', s=35, alpha=0.9, 
                    edgecolors='white', linewidths=0.3, vmin=0.0, vmax=3.0)
    ax.set_xlabel('Local East (m)')
    ax.set_ylabel('Local North (m)')
    ax.set_title(title, fontsize=15)
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.ticklabel_format(style='plain', axis='both') # 防止出现科学计数法
    return sc

# ================= 图 (a) 完整空间轨迹图 =================
fig_a, ax_a = plt.subplots(figsize=(6, 5.5))
sc_a = plot_spatial_scatter(ax_a, x_arr, y_arr, std_arr, '(a) Full Spatial Trajectory')

x_margin_a = (np.max(x_arr) - np.min(x_arr)) * 0.15
ax_a.set_xlim(np.min(x_arr) - x_margin_a, np.max(x_arr) + x_margin_a)
ax_a.set_ylim(np.min(y_arr) - 0.5, np.max(y_arr) + 0.5)

cbar_a = fig_a.colorbar(sc_a, ax=ax_a, pad=0.02)
cbar_a.set_label('Position Uncertainty (m)', fontsize=12)
fig_a.tight_layout()
fig_a.savefig('plot_a_full_spatial.pdf', dpi=300)


# ================= 图 (b) 完整时间不确定度图 =================
fig_b, ax_b = plt.subplots(figsize=(7, 5.5))
ax_b.plot(t_arr, std_arr, color='#D32F2F', linewidth=2.0)
ax_b.axvline(x=degrade_time, color='black', linestyle='--', linewidth=1.5)

max_std = np.max(std_arr)
ax_b.set_ylim(-0.5, max_std * 1.15)
ax_b.set_xlim(0, t_arr[-1])

ax_b.text(degrade_time/2, max_std * 0.95, 'Open Sky\n(Stationary)', ha='center', va='center', fontsize=13, color='green')
ax_b.text((degrade_time + t_arr[-1])/2, max_std * 0.95, 'Canopy Occlusion\n(Moving)', ha='center', va='center', fontsize=13, color='#D32F2F')

ax_b.set_xlabel('Time (s)')
ax_b.set_ylabel('GNSS Position Uncertainty (m)')
ax_b.set_title('(b) Temporal Uncertainty Surge', fontsize=15)
ax_b.grid(True, linestyle='--', alpha=0.6)
fig_b.tight_layout()
fig_b.savefig('plot_b_full_temporal.pdf', dpi=300)


# ================= 图 (c) 局部空间图：低漂移区 (要求更细的刻度) =================
fig_c, ax_c = plt.subplots(figsize=(5, 5))
sc_c = plot_spatial_scatter(ax_c, x_arr[mask_low], y_arr[mask_low], std_arr[mask_low], '(c) Zoom: Low Drift (Open Sky)')

# 自适应调整极小视野
x_min_c, x_max_c = np.min(x_arr[mask_low]), np.max(x_arr[mask_low])
y_min_c, y_max_c = np.min(y_arr[mask_low]), np.max(y_arr[mask_low])
ax_c.set_xlim(x_min_c - 0.002, x_max_c + 0.002)
ax_c.set_ylim(y_min_c - 0.002, y_max_c + 0.002)

# ★ 核心修改：强制划分更多的刻度线 (nbins=7表示大约划分7个区间)
ax_c.xaxis.set_major_locator(ticker.MaxNLocator(nbins=6))
ax_c.yaxis.set_major_locator(ticker.MaxNLocator(nbins=7))

cbar_c = fig_c.colorbar(sc_c, ax=ax_c, pad=0.02)
cbar_c.set_label('Position Uncertainty (m)', fontsize=12)
fig_c.tight_layout()
fig_c.savefig('plot_c_spatial_low_drift.pdf', dpi=300)


# ================= 图 (d) 局部时间图：低漂移细节 (原图e) =================
fig_d, ax_d = plt.subplots(figsize=(5, 4))
ax_d.plot(t_arr[mask_low], std_arr[mask_low], color='#D32F2F', linewidth=1.5)

ax_d.set_xlim(1, degrade_time - 1)  # 避开起始瞬间
inset_ymax = np.max(std_arr[mask_low]) * 1.5 if len(std_arr[mask_low]) > 0 else 0.1
ax_d.set_ylim(0, inset_ymax)

ax_d.set_xlabel('Time (s)')
ax_d.set_ylabel('Uncertainty (m)')
ax_d.set_title('(d) Zoom: Temporal Open Sky', fontsize=15)
ax_d.grid(True, linestyle=':', alpha=0.6)
fig_d.tight_layout()
fig_d.savefig('plot_d_temporal_zoom.pdf', dpi=300)

plt.show()
print("🎉 4 张子图 (a, b, c, d) 已成功生成并保存！图(c)已加密刻度显示。")
