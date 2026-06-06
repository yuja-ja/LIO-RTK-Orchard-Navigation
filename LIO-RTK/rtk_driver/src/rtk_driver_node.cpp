/**
 * rtk_driver_node.cpp  (Enhanced Research Version + TF Broadcast)
 *
 * 功能：
 *  - 解析 GGA / GST / GSA / HDT/THS
 *  - 发布真实 GNSS 协方差
 *  - 广播 TF: world → rtk_link
 *  - 为 LiDAR-IMU-RTK 融合提供 probabilistic measurements
 *
 * 专为：FAST-LIO2 + GNSS 融合设计
 */

#include <ros/ros.h>
#include <serial/serial.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <GeographicLib/UTMUPS.hpp>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include <fstream>
#include <iomanip>

using namespace std;

class RTKDriver {
public:
    RTKDriver(): origin_set_(false)
    {
        ros::NodeHandle nh("~");

        string port;
        int baud;

        nh.param("port", port, string("/dev/ttyUSB0"));
        nh.param("baud", baud, 115200);
        nh.param("frame_id", frame_id_, string("world"));
        nh.param("child_frame_id", child_frame_id_, string("rtk_link"));
        // ==================== 新增：读取模式并加载原点 ====================
        nh.param("localization_mode", localization_mode_, false);

        if (localization_mode_) {
            std::string origin_file = "/home/gsm/LIO-RTK/src/FAST_LIO-RTK/PCD/map_origin.txt";
            std::ifstream in_origin(origin_file);
            if (in_origin.is_open()) {
                std::string line;
                while (std::getline(in_origin, line)) {
                    if (line.find("UTM_X:") != std::string::npos) origin_x_ = std::stod(line.substr(line.find(":") + 1));
                    else if (line.find("UTM_Y:") != std::string::npos) origin_y_ = std::stod(line.substr(line.find(":") + 1));
                    else if (line.find("UTM_Z:") != std::string::npos) origin_z_ = std::stod(line.substr(line.find(":") + 1));
                }
                origin_set_ = true;
                ROS_INFO("\033[1;32m[ RTK Driver ] Localization Mode: Map origin loaded from %s\033[0m", origin_file.c_str());
            } else {
                ROS_ERROR("\033[1;31m[ RTK Driver ] Localization Mode enabled, but failed to open %s. Driver will wait for new RTK origin.\033[0m", origin_file.c_str());
            }
        }
        // ==================================================================

        pub_fix_     = nh.advertise<sensor_msgs::NavSatFix>("/rtk/fix", 10);
        pub_heading_ = nh.advertise<sensor_msgs::Imu>("/rtk/heading_raw", 10);
        pub_odom_    = nh.advertise<nav_msgs::Odometry>("/rtk/odom_truth", 10);

        ser_.setPort(port);
        ser_.setBaudrate(baud);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser_.setTimeout(to);
        ser_.open();

        ROS_INFO("Enhanced RTK Driver with TF broadcast started.");
    }

    void run()
    {
        ros::Rate rate(100);
        string buffer;

        while (ros::ok())
        {
            if (ser_.available())
            {
                buffer += ser_.read(ser_.available());

                size_t pos;
                while ((pos = buffer.find("\r\n")) != string::npos)
                {
                    string line = buffer.substr(0, pos);
                    buffer.erase(0, pos+2);
                    parseNMEA(line);
                }
            }

            ros::spinOnce();
            rate.sleep();
        }
    }

private:

    /* ================= ROS ================= */
    ros::Publisher pub_fix_, pub_heading_, pub_odom_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;  // 👈 新增 TF broadcaster
    serial::Serial ser_;

    string frame_id_, child_frame_id_;
    bool localization_mode_; // 标识是否为定位模式

    /* ================= GNSS state ================= */

    double lat_=0, lon_=0, alt_=0;
    double heading_deg_=0;

    bool has_heading_=false;
    int quality_=0;

    /* ⭐⭐⭐ 新增：真实不确定度 */
    double std_lat_=1.0;
    double std_lon_=1.0;
    double std_alt_=2.0;

    double hdop_=1.0;
    int sat_used_=10;


    /* ================= origin ================= */

    bool origin_set_;
    double origin_x_, origin_y_, origin_z_;


    /* ================= helpers ================= */

    vector<string> split(const string& s, char d)
    {
        vector<string> v;
        string token;
        stringstream ss(s);
        while(getline(ss, token, d)) v.push_back(token);
        return v;
    }

    double nmea2deg(double val)
    {
        int deg = val/100;
        double min = val - deg*100;
        return deg + min/60.0;
    }


    /* ========================================================= */
    /* ===================== NMEA PARSER ======================== */
    /* ========================================================= */

    void parseNMEA(string line)
    {
        if(line.empty() || line[0]!='$') return;

        size_t star = line.find('*');
        if(star!=string::npos) line=line.substr(0,star);

        auto p = split(line, ',');

        if(p.empty()) return;

        /* ================= GGA ================= */
        if(p[0]=="$GNGGA" || p[0]=="$GPGGA")
        {
            if(p.size()<10) return;

            lat_ = nmea2deg(stod(p[2]));
            if(p[3]=="S") lat_=-lat_;

            lon_ = nmea2deg(stod(p[4]));
            if(p[5]=="W") lon_=-lon_;

            alt_ = stod(p[9]);
            quality_ = stoi(p[6]);

            publishFix();
            publishOdom();
        }

        /* ================= GST ⭐⭐⭐ 关键 ================= */
        else if(p[0]=="$GNGST" || p[0]=="$GPGST")
        {
            if(p.size()<9) return;

            std_lat_ = stod(p[6]);
            std_lon_ = stod(p[7]);
            std_alt_ = stod(p[8]);
        }

        /* ================= GSA ================= */
        else if(p[0]=="$GNGSA" || p[0]=="$GPGSA")
        {
            if(p.size()<17) return;

            hdop_ = stod(p[16]);

            sat_used_=0;
            for(int i=3;i<=14;i++)
                if(!p[i].empty()) sat_used_++;
        }

        /* ================= HEADING ================= */
        else if(p[0].find("HDT")!=string::npos ||
                p[0].find("THS")!=string::npos)
        {
            heading_deg_=stod(p[1]);
            has_heading_=true;
            publishHeading();
        }
    }


    /* ========================================================= */
    /* ===================== PUBLISHERS ========================= */
    /* ========================================================= */

    void publishFix()
    {
        sensor_msgs::NavSatFix msg;

        msg.header.stamp=ros::Time::now();
        msg.header.frame_id=child_frame_id_;

        msg.latitude=lat_;
        msg.longitude=lon_;
        msg.altitude=alt_;

        /* ⭐⭐⭐ 真实协方差 */
        msg.position_covariance[0]=std_lat_*std_lat_;
        msg.position_covariance[4]=std_lon_*std_lon_;
        msg.position_covariance[8]=std_alt_*std_alt_;
        msg.position_covariance_type=
            sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

        pub_fix_.publish(msg);
    }


    void publishHeading()
    {
        sensor_msgs::Imu imu;

        imu.header.stamp=ros::Time::now();

        double yaw=(90-heading_deg_)*M_PI/180.0;

        tf2::Quaternion q;
        q.setRPY(0,0,yaw);

        imu.orientation.x=q.x();
        imu.orientation.y=q.y();
        imu.orientation.z=q.z();
        imu.orientation.w=q.w();

        imu.orientation_covariance[8]=1e-6;

        pub_heading_.publish(imu);
    }


    void publishOdom()
    {
        if(quality_==0) return;

        int zone; bool northp;
        double x,y;

        GeographicLib::UTMUPS::Forward(lat_,lon_,zone,northp,x,y);

        if(!origin_set_ && quality_>=4 && !localization_mode_)
        {
            origin_x_=x; origin_y_=y; origin_z_=alt_;
            origin_set_=true;
            // ==================== 新增：将原点保存到文件 ====================
            // 建议保存到你建图所在的工作空间目录下
            std::string origin_file = "/home/gsm/LIO-RTK/src/FAST_LIO-RTK/PCD/map_origin.txt"; 
            std::ofstream out_origin(origin_file, std::ios::out);
            if(out_origin.is_open()) {
                out_origin << std::fixed << std::setprecision(6);
                out_origin << "Lat: " << lat_ << "\n";
                out_origin << "Lon: " << lon_ << "\n";
                out_origin << "Alt: " << alt_ << "\n";
                out_origin << "UTM_Zone: " << zone << (northp ? " N" : " S") << "\n";
                out_origin << "UTM_X: " << origin_x_ << "\n";
                out_origin << "UTM_Y: " << origin_y_ << "\n";
                out_origin << "UTM_Z: " << origin_z_ << "\n";
                out_origin.close();
                ROS_INFO("\033[1;32m[ RTK Driver ] Map global origin saved to: %s\033[0m", origin_file.c_str());
            } else {
                ROS_ERROR("[ RTK Driver ] Failed to save map origin file!");
            }
            // ===============================================================
        }
        if(!origin_set_) return;

        nav_msgs::Odometry odom;

        odom.header.stamp=ros::Time::now();
        odom.header.frame_id=frame_id_;
        odom.child_frame_id=child_frame_id_;

        odom.pose.pose.position.x=x-origin_x_;
        odom.pose.pose.position.y=y-origin_y_;
        odom.pose.pose.position.z=alt_-origin_z_;

        if (has_heading_) 
        {
            // 1. 修正安装误差：主左从右，RTK读数指向车身右侧
            //    车头真实航向(NMEA) = RTK读数 - 90度
            // 2. 转换为 ROS ENU 坐标系 (ROS = 90 - NMEA)
            //    ROS = 90 - (RTK - 90) = 180 - RTK
            double fine_tune_bias = -3.75;  // evo_ape tum rtk_truth.txt FAST-LIO.txt -a -v，在使用前先注销，录制一段时间的 RTK 真值数据，使用 evo_ape 评估航向误差，微调这个偏置直到 yaw RMSE 最小！这个偏置是为了修正 RTK 天线安装误差导致的航向偏移，通常在 -5 到 +5 度之间。
            double offset_angle = 90.0 + fine_tune_bias; 
            
            double ros_yaw_deg = (90.0 - heading_deg_) + offset_angle;
            // 归一化到 [-180, 180]
            while(ros_yaw_deg > 180.0)  ros_yaw_deg -= 360.0;
            while(ros_yaw_deg < -180.0) ros_yaw_deg += 360.0;
            double yaw_rad = ros_yaw_deg * M_PI / 180.0;
            tf2::Quaternion q;
            q.setRPY(0, 0, yaw_rad); 
            odom.pose.pose.orientation.x = q.x();
            odom.pose.pose.orientation.y = q.y();
            odom.pose.pose.orientation.z = q.z();
            odom.pose.pose.orientation.w = q.w();
            odom.pose.covariance[35] = 0.01; 
        }
        else 
        {
            odom.pose.pose.orientation.w = 1.0;
        }

        /* ⭐⭐⭐ 关键：把真实协方差同步给 odom */
        odom.pose.covariance[0]=std_lat_*std_lat_;
        odom.pose.covariance[7]=std_lon_*std_lon_;
        odom.pose.covariance[14]=std_alt_*std_alt_;

        pub_odom_.publish(odom);

        // === 👇 新增：广播 TF (world → rtk_link) 👇 ===
        geometry_msgs::TransformStamped transform;
        transform.header.stamp = odom.header.stamp;
        transform.header.frame_id = frame_id_;      // "world"
        transform.child_frame_id = child_frame_id_; // "rtk_link"
        transform.transform.translation.x = odom.pose.pose.position.x;
        transform.transform.translation.y = odom.pose.pose.position.y;
        transform.transform.translation.z = odom.pose.pose.position.z;
        transform.transform.rotation = odom.pose.pose.orientation;
        tf_broadcaster_.sendTransform(transform);
    }
};


int main(int argc,char** argv)
{
    ros::init(argc,argv,"rtk_driver_node_enhanced");
    RTKDriver node;
    node.run();
    return 0;
}