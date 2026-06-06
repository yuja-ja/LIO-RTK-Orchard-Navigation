/**
 * rtk_ground_truth_node.cpp
 *
 * 功能：
 *  - 解析 GGA / GST / GSA / HDT/THS
 *  - 发布绝对 UTM 坐标作为 ground truth（不依赖 origin）
 *  - 广播 TF: world → rtk_gt_link
 *  - 用于评估 FAST-LIO2 + RTK 融合系统的精度
 *
 * 注意：此节点仅用于真值参考，不参与融合！
 * 兼容 ROS 1（Noetic/Melodic），已修复 tf2::toMsg 编译错误。
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

using namespace std;

class RTKGroundTruthDriver {
public:
    RTKGroundTruthDriver()
    {
        ros::NodeHandle nh("~");

        string port;
        int baud;
        nh.param("port", port, string("/dev/ttyUSB0"));
        nh.param("baud", baud, 115200);
        nh.param("frame_id", frame_id_, string("world"));
        nh.param("child_frame_id", child_frame_id_, string("rtk_gt_link"));

        pub_fix_     = nh.advertise<sensor_msgs::NavSatFix>("/ground_truth/fix", 10);
        pub_heading_ = nh.advertise<sensor_msgs::Imu>("/ground_truth/heading", 10);
        pub_odom_    = nh.advertise<nav_msgs::Odometry>("/ground_truth/odom", 10);

        ser_.setPort(port);
        ser_.setBaudrate(baud);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser_.setTimeout(to);
        ser_.open();

        ROS_INFO("RTK Ground Truth Driver started. Publishing absolute UTM as /ground_truth/odom.");
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
                    buffer.erase(0, pos + 2);
                    parseNMEA(line);
                }
            }

            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::Publisher pub_fix_, pub_heading_, pub_odom_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    serial::Serial ser_;

    string frame_id_, child_frame_id_;

    double lat_ = 0, lon_ = 0, alt_ = 0;
    double heading_deg_ = 0;
    bool has_heading_ = false;
    int quality_ = 0;

    // 协方差（假设高精度 RTK）
    double std_lat_ = 0.01;  // 1 cm
    double std_lon_ = 0.01;
    double std_alt_ = 0.02;  // 2 cm

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
        int deg = static_cast<int>(val / 100);
        double min = val - deg * 100;
        return deg + min / 60.0;
    }

    void parseNMEA(string line)
    {
        if(line.empty() || line[0] != '$') return;

        size_t star = line.find('*');
        if(star != string::npos) line = line.substr(0, star);

        auto p = split(line, ',');

        if(p.empty()) return;

        if(p[0] == "$GNGGA" || p[0] == "$GPGGA")
        {
            if(p.size() < 10) return;
            lat_ = nmea2deg(stod(p[2]));
            if(p[3] == "S") lat_ = -lat_;
            lon_ = nmea2deg(stod(p[4]));
            if(p[5] == "W") lon_ = -lon_;
            alt_ = stod(p[9]);
            quality_ = stoi(p[6]);

            publishFix();
            publishOdom();
        }
        else if(p[0] == "$GNGST" || p[0] == "$GPGST")
        {
            if(p.size() >= 9) {
                std_lat_ = stod(p[6]);
                std_lon_ = stod(p[7]);
                std_alt_ = stod(p[8]);
            }
        }
        else if(p[0].find("HDT") != string::npos || p[0].find("THS") != string::npos)
        {
            if(p.size() >= 2 && !p[1].empty()) {
                heading_deg_ = stod(p[1]);
                has_heading_ = true;
                publishHeading();
            }
        }
    }

    void publishFix()
    {
        sensor_msgs::NavSatFix msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = child_frame_id_;
        msg.latitude = lat_;
        msg.longitude = lon_;
        msg.altitude = alt_;

        msg.position_covariance[0] = std_lat_ * std_lat_;
        msg.position_covariance[4] = std_lon_ * std_lon_;
        msg.position_covariance[8] = std_alt_ * std_alt_;
        msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

        pub_fix_.publish(msg);
    }

    void publishHeading()
    {
        sensor_msgs::Imu imu;
        imu.header.stamp = ros::Time::now();
        imu.header.frame_id = child_frame_id_;

        double yaw = (90.0 - heading_deg_) * M_PI / 180.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);

        imu.orientation.x = q.x();
        imu.orientation.y = q.y();
        imu.orientation.z = q.z();
        imu.orientation.w = q.w();
        imu.orientation_covariance[8] = 1e-6;

        pub_heading_.publish(imu);
    }

    void publishOdom()
    {
        if(quality_ == 0) return;

        int zone; 
        bool northp;
        double x, y;
        try {
            GeographicLib::UTMUPS::Forward(lat_, lon_, zone, northp, x, y);
        } catch (const std::exception& e) {
            ROS_WARN_THROTTLE(1.0, "GeographicLib UTM conversion failed: %s", e.what());
            return;
        }

        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = frame_id_;
        odom.child_frame_id = child_frame_id_;

        // ⭐ 发布绝对 UTM 坐标（无 origin offset）
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = alt_;

        if (has_heading_)
        {
            double ros_yaw_deg = 90.0 - heading_deg_; // ENU convention
            while(ros_yaw_deg > 180.0)  ros_yaw_deg -= 360.0;
            while(ros_yaw_deg < -180.0) ros_yaw_deg += 360.0;
            double yaw_rad = ros_yaw_deg * M_PI / 180.0;
            tf2::Quaternion q;
            q.setRPY(0, 0, yaw_rad);

            // ✅ 手动赋值：兼容 ROS 1
            odom.pose.pose.orientation.x = q.x();
            odom.pose.pose.orientation.y = q.y();
            odom.pose.pose.orientation.z = q.z();
            odom.pose.pose.orientation.w = q.w();

            odom.pose.covariance[35] = 1e-6;
        }
        else
        {
            odom.pose.pose.orientation.w = 1.0;
        }

        // Position covariance
        odom.pose.covariance[0]  = std_lat_ * std_lat_;
        odom.pose.covariance[7]  = std_lon_ * std_lon_;
        odom.pose.covariance[14] = std_alt_ * std_alt_;

        pub_odom_.publish(odom);

        // Broadcast TF
        geometry_msgs::TransformStamped transform;
        transform.header.stamp = odom.header.stamp;
        transform.header.frame_id = frame_id_;
        transform.child_frame_id = child_frame_id_;
        transform.transform.translation.x = x;
        transform.transform.translation.y = y;
        transform.transform.translation.z = alt_;
        transform.transform.rotation = odom.pose.pose.orientation;
        tf_broadcaster_.sendTransform(transform);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rtk_ground_truth_node");
    RTKGroundTruthDriver node;
    node.run();
    return 0;
}