/**
 * Wit/Yahboom IMU Driver for ROS 1 (C++) - Final Version
 * Parses 0x55 0x51(Acc), 0x52(Gyro), 0x59(Quat)
 */

#include <ros/ros.h>
#include <serial/serial.h>
#include <sensor_msgs/Imu.h>
#include <iostream>
#include <vector>
#include <cmath>

// 协议常量
const uint8_t HEADER = 0x55;
const uint8_t TYPE_ACCEL = 0x51;
const uint8_t TYPE_GYRO = 0x52;
const uint8_t TYPE_QUAT = 0x59;

// 物理量转换参数
const double ACC_SCALE = 16.0 * 9.80665 / 32768.0;
const double GYRO_SCALE = 2000.0 * (M_PI / 180.0) / 32768.0;

class YahboomImuDriver {
public:
    YahboomImuDriver() : nh_("~") {
        std::string port_name;
        int baud_rate;
        nh_.param<std::string>("port", port_name, "/dev/ttyUSB0");
        nh_.param<int>("baud", baud_rate, 115200);
        nh_.param<std::string>("frame_id", frame_id_, "imu_link");
        nh_.param<std::string>("topic", topic_name_, "/imu/data_raw");

        imu_pub_ = nh_.advertise<sensor_msgs::Imu>(topic_name_, 200);

        try {
            ser_.setPort(port_name);
            ser_.setBaudrate(baud_rate);
            serial::Timeout to = serial::Timeout::simpleTimeout(50); // 降低超时时间
            ser_.setTimeout(to);
            ser_.open();
        } catch (serial::IOException& e) {
            ROS_ERROR_STREAM("Unable to open port " << port_name);
            ros::shutdown();
            return;
        }

        ROS_INFO_STREAM("IMU Connected: " << port_name << " @ " << baud_rate);
        
        // 初始化数据
        acc_data_[0] = 0; acc_data_[1] = 0; acc_data_[2] = 0;
        gyro_data_[0] = 0; gyro_data_[1] = 0; gyro_data_[2] = 0;
        quat_data_[0] = 0; quat_data_[1] = 0; quat_data_[2] = 0; quat_data_[3] = 1;
    }

    void run() {
        ros::Rate loop_rate(1000); // 1000Hz 循环处理
        
        while (ros::ok()) {
            if (ser_.available()) {
                // 读取数据到 buffer
                std::string data_read = ser_.read(ser_.available());
                buffer_.insert(buffer_.end(), data_read.begin(), data_read.end());
                processBuffer();
            }
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher imu_pub_;
    serial::Serial ser_;
    std::string frame_id_;
    std::string topic_name_;
    std::vector<uint8_t> buffer_;

    double acc_data_[3];
    double gyro_data_[3];
    double quat_data_[4];
    
    // 状态标志
    bool got_acc_ = false;

    void processBuffer() {
        // 循环直到 buffer 不足 11 字节
        while (buffer_.size() >= 11) {
            // 1. 寻找帧头 0x55
            if (buffer_[0] != HEADER) {
                buffer_.erase(buffer_.begin());
                continue;
            }

            // 2. 检查校验和 Sum = 0x55 + Type + 8 bytes data
            uint8_t checksum = 0x55;
            uint8_t type = buffer_[1];
            checksum += type;
            for (int i = 2; i < 10; i++) {
                checksum += buffer_[i];
            }
            
            if (checksum != buffer_[10]) {
                // 校验失败，可能是误判，移除第一个字节重新寻找 0x55
                buffer_.erase(buffer_.begin());
                continue;
            }

            // 3. 解析数据 (Little Endian)
            int16_t raw[4];
            for (int i = 0; i < 4; i++) {
                raw[i] = (int16_t)((buffer_[2 + i * 2]) | (buffer_[3 + i * 2] << 8));
            }

            if (type == TYPE_ACCEL) {
                acc_data_[0] = raw[0] * ACC_SCALE;
                acc_data_[1] = raw[1] * ACC_SCALE;
                acc_data_[2] = raw[2] * ACC_SCALE;
                got_acc_ = true;
            } 
            else if (type == TYPE_GYRO) {
                gyro_data_[0] = raw[0] * GYRO_SCALE;
                gyro_data_[1] = raw[1] * GYRO_SCALE;
                gyro_data_[2] = raw[2] * GYRO_SCALE;
                
                // 数据流顺序通常是 Acc -> Gyro -> Quat
                // 在收到 Gyro 时，如果有 Acc 数据，就可以发布了，这样延时最小
                if (got_acc_) {
                    publishImuMessage();
                    got_acc_ = false; // 重置标志，等待下一帧 Acc
                }
            }
            else if (type == TYPE_QUAT) {
                // Q0=W, Q1=X, Q2=Y, Q3=Z
                quat_data_[3] = raw[0] / 32768.0; // W
                quat_data_[0] = raw[1] / 32768.0; // X
                quat_data_[1] = raw[2] / 32768.0; // Y
                quat_data_[2] = raw[3] / 32768.0; // Z
            }

            // 移除已处理的包
            buffer_.erase(buffer_.begin(), buffer_.begin() + 11);
        }
    }

    void publishImuMessage() {
        sensor_msgs::Imu imu_msg;
        imu_msg.header.stamp = ros::Time::now();
        imu_msg.header.frame_id = frame_id_;

        imu_msg.linear_acceleration.x = acc_data_[0];
        imu_msg.linear_acceleration.y = acc_data_[1];
        imu_msg.linear_acceleration.z = acc_data_[2];

        imu_msg.angular_velocity.x = gyro_data_[0];
        imu_msg.angular_velocity.y = gyro_data_[1];
        imu_msg.angular_velocity.z = gyro_data_[2];

        imu_msg.orientation.x = quat_data_[0];
        imu_msg.orientation.y = quat_data_[1];
        imu_msg.orientation.z = quat_data_[2];
        imu_msg.orientation.w = quat_data_[3];
        
        // 协方差 (FAST-LIO2 内部会处理，这里给 0)
        imu_msg.orientation_covariance[0] = 0.0;
        imu_msg.angular_velocity_covariance[0] = 0.0;
        imu_msg.linear_acceleration_covariance[0] = 0.0;

        imu_pub_.publish(imu_msg);

        // 每 1 秒在终端打印一次数据，方便调试
        ROS_INFO_THROTTLE(1.0, "IMU Data -> Acc: [%.2f, %.2f, %.2f] Gyro: [%.2f, %.2f, %.2f]", 
                          acc_data_[0], acc_data_[1], acc_data_[2],
                          gyro_data_[0], gyro_data_[1], gyro_data_[2]);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "wit_imu_node");
    YahboomImuDriver driver;
    driver.run();
    return 0;
}
