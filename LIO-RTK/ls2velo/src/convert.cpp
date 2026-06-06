#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// ==========================================
// 1. 输入：镭神 C32 原始结构 (适配你的 rostopic echo 结果)
// ==========================================
struct PointLS {
    PCL_ADD_POINT4D;            // x,y,z (Float32, Offset 0-15)
    float intensity;            // Offset 16 (Float32)
    uint16_t ring;              // Offset 20 (UInt16)
    uint16_t padding;           // 补位 2字节 (Offset 22-23, 凑齐内存对齐)
    double time;                // Offset 24 (Double/Float64) -> 你的 log 显示 datatype 8
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

// 注册输入类型
POINT_CLOUD_REGISTER_POINT_STRUCT(PointLS,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint16_t, ring, ring)
    (double, time, time)        // 注意这里对应 double
)

// ==========================================
// 2. 输出：Velodyne 标准结构 (适配 FAST-LIVO2)
// ==========================================
struct PointVelo {
    PCL_ADD_POINT4D;
    float intensity;
    float time;                 // 目标：Float32，相对时间
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointVelo,
    (float, x, x) (float, y, y) (float, z, z)
    (float, intensity, intensity)
    (float, time, time)
    (std::uint16_t, ring, ring)
)

ros::Publisher pub;

void callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    pcl::PointCloud<PointLS> cloud_ls;
    pcl::fromROSMsg(*msg, cloud_ls);

    pcl::PointCloud<PointVelo> cloud_velo;
    cloud_velo.header = cloud_ls.header;
    cloud_velo.is_dense = cloud_ls.is_dense;
    
    // 我们可以把 FrameID 改成 FAST-LIVO2 默认喜欢的，也可以不改
    // cloud_velo.header.frame_id = "velodyne"; 

    if (cloud_ls.empty()) return;

    // 获取帧头时间 (用于计算相对时间)
    double header_time = msg->header.stamp.toSec();
    
    for (const auto& p_ls : cloud_ls.points) {
        // 剔除无效点 (NaN/Inf)
        if (!pcl::isFinite(p_ls)) continue;

        PointVelo p_velo;
        p_velo.x = p_ls.x;
        p_velo.y = p_ls.y;
        p_velo.z = p_ls.z;
        p_velo.intensity = p_ls.intensity;
        p_velo.ring = p_ls.ring;

        // --- 核心逻辑：时间处理 ---
        // 你的 time 是 double，可能是绝对时间。我们需要相对时间。
        // 逻辑：如果点的时间 > 10亿 (绝对时间)，则减去 header_time
        
        double rel_time = 0.0;
        if (p_ls.time > 1e9) { 
            rel_time = p_ls.time - header_time;
        } else {
            // 如果驱动输出的已经是相对时间 (比如 0.05)
            rel_time = p_ls.time;
        }

        // 安全钳制：FAST-LIVO2 假设点云时间为正，且在扫描周期内 (0.0 ~ 0.1)
        // 有些机械雷达会有微小的负值漂移，这里强制修正
        if (rel_time < 0.0) rel_time = 0.0;
        if (rel_time > 0.2) rel_time = 0.1; // 假设是 10Hz

        p_velo.time = (float)rel_time; // 转为 float32
        
        cloud_velo.points.push_back(p_velo);
    }

    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(cloud_velo, output_msg);
    output_msg.header = msg->header;
    pub.publish(output_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ls2velo_converter");
    ros::NodeHandle nh;

    // 订阅你的原始话题
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/point_cloud_raw", 100, callback);
    
    // 发布转换后的话题
    pub = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 100);

    ROS_INFO("Lslidar C32 (Double Time) -> Velodyne (Float Time) Converter Started.");
    ros::spin();
    return 0;
}