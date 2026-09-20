#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <exception>
#include <string>
#include <vector>

#include <boost/array.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <serial/serial.h>

#include "imu_ros_driver/wit_protocol.hpp"

namespace imu_ros_driver {
namespace {

class DeviceStampMapper {
 public:
  DeviceStampMapper(double expected_rate_hz, double timestamp_offset_sec,
                    double max_gap_factor, double max_clock_skew_sec)
      : expected_period_ms_(1000.0 / expected_rate_hz),
        timestamp_offset_sec_(timestamp_offset_sec),
        max_gap_factor_(max_gap_factor),
        max_clock_skew_sec_(max_clock_skew_sec) {}

  bool map(const DeviceTime& device_time, const ros::Time& receipt_time,
           ros::Time* stamp) {
    std::int64_t unwrapped_millis = 0;
    std::int64_t delta_millis = 0;
    DeviceTimeUnwrapper::Status status = unwrapper_.update(
        device_time.millis_of_day, &unwrapped_millis, &delta_millis);

    if (status == DeviceTimeUnwrapper::Status::kDuplicate) {
      ++duplicate_timestamps_;
      return false;
    }

    if (status == DeviceTimeUnwrapper::Status::kBackward) {
      ++clock_resets_;
      ++clock_reanchors_;
      unwrapper_.reset();
      status = unwrapper_.update(device_time.millis_of_day,
                                 &unwrapped_millis, &delta_millis);
      resetAnchor(unwrapped_millis, receipt_time);
    }

    if (status == DeviceTimeUnwrapper::Status::kInitialized ||
        !anchor_initialized_) {
      resetAnchor(unwrapped_millis, receipt_time);
    } else if (delta_millis > max_gap_factor_ * expected_period_ms_) {
      ++gap_events_;
      const auto periods = static_cast<std::uint64_t>(
          std::llround(delta_millis / expected_period_ms_));
      if (periods > 1) {
        estimated_missing_samples_ += periods - 1;
      }
    }

    double base_stamp_sec =
        anchor_ros_sec_ +
        static_cast<double>(unwrapped_millis - anchor_device_millis_) / 1000.0;

    if (max_clock_skew_sec_ > 0.0) {
      const double clock_skew_sec =
          std::abs(receipt_time.toSec() - base_stamp_sec);
      if (clock_skew_sec > max_clock_skew_sec_) {
        if (!clock_skew_active_) {
          ++clock_skew_events_;
          clock_skew_active_ = true;
        }
      } else if (clock_skew_sec < max_clock_skew_sec_ * 0.5) {
        clock_skew_active_ = false;
      }
    }

    double output_stamp_sec = base_stamp_sec + timestamp_offset_sec_;
    if (have_last_stamp_ && output_stamp_sec <= last_stamp_sec_) {
      output_stamp_sec = last_stamp_sec_ + 1e-6;
      ++monotonic_corrections_;
    }
    if (output_stamp_sec <= 0.0) {
      return false;
    }

    stamp->fromSec(output_stamp_sec);
    last_stamp_sec_ = output_stamp_sec;
    have_last_stamp_ = true;
    return true;
  }

  void reset() {
    unwrapper_.reset();
    anchor_initialized_ = false;
    have_last_stamp_ = false;
    anchor_device_millis_ = 0;
    anchor_ros_sec_ = 0.0;
    last_stamp_sec_ = 0.0;
    clock_skew_active_ = false;
  }

  std::uint64_t duplicateTimestamps() const { return duplicate_timestamps_; }
  std::uint64_t clockResets() const { return clock_resets_; }
  std::uint64_t clockReanchors() const { return clock_reanchors_; }
  std::uint64_t clockSkewEvents() const { return clock_skew_events_; }
  std::uint64_t gapEvents() const { return gap_events_; }
  std::uint64_t estimatedMissingSamples() const {
    return estimated_missing_samples_;
  }
  std::uint64_t monotonicCorrections() const {
    return monotonic_corrections_;
  }

 private:
  void resetAnchor(std::int64_t unwrapped_millis,
                   const ros::Time& receipt_time) {
    anchor_device_millis_ = unwrapped_millis;
    anchor_ros_sec_ = receipt_time.toSec();
    anchor_initialized_ = true;
  }

  DeviceTimeUnwrapper unwrapper_;
  double expected_period_ms_;
  double timestamp_offset_sec_;
  double max_gap_factor_;
  double max_clock_skew_sec_;
  bool anchor_initialized_ = false;
  bool have_last_stamp_ = false;
  bool clock_skew_active_ = false;
  std::int64_t anchor_device_millis_ = 0;
  double anchor_ros_sec_ = 0.0;
  double last_stamp_sec_ = 0.0;

  std::uint64_t duplicate_timestamps_ = 0;
  std::uint64_t clock_resets_ = 0;
  std::uint64_t clock_reanchors_ = 0;
  std::uint64_t clock_skew_events_ = 0;
  std::uint64_t gap_events_ = 0;
  std::uint64_t estimated_missing_samples_ = 0;
  std::uint64_t monotonic_corrections_ = 0;
};

class ArrivalStampMapper {
 public:
  explicit ArrivalStampMapper(double timestamp_offset_sec)
      : timestamp_offset_sec_(timestamp_offset_sec) {}

  bool map(const ros::Time& receipt_time, ros::Time* stamp) {
    double output_stamp_sec = receipt_time.toSec() + timestamp_offset_sec_;
    if (have_last_stamp_ && output_stamp_sec <= last_stamp_sec_) {
      output_stamp_sec = last_stamp_sec_ + 1e-6;
      ++monotonic_corrections_;
    }
    if (output_stamp_sec <= 0.0) {
      return false;
    }
    stamp->fromSec(output_stamp_sec);
    last_stamp_sec_ = output_stamp_sec;
    have_last_stamp_ = true;
    return true;
  }

  void reset() {
    have_last_stamp_ = false;
    last_stamp_sec_ = 0.0;
  }

  std::uint64_t monotonicCorrections() const {
    return monotonic_corrections_;
  }

 private:
  double timestamp_offset_sec_;
  bool have_last_stamp_ = false;
  double last_stamp_sec_ = 0.0;
  std::uint64_t monotonic_corrections_ = 0;
};

class ImuRosDriver {
 public:
  ImuRosDriver()
      : nh_(),
        private_nh_("~"),
        assembler_(CycleAssembler::Mode::kRequireDeviceTime),
        device_stamp_mapper_(200.0, 0.0, 1.5, 0.5),
        arrival_stamp_mapper_(0.0) {
    loadParameters();

    assembler_ = CycleAssembler(
        use_device_time_ ? CycleAssembler::Mode::kRequireDeviceTime
                         : CycleAssembler::Mode::kImplicitAccGyro);
    device_stamp_mapper_ = DeviceStampMapper(
        expected_rate_hz_, timestamp_offset_sec_, max_gap_factor_,
        max_clock_skew_sec_);
    arrival_stamp_mapper_ = ArrivalStampMapper(timestamp_offset_sec_);

    imu_publisher_ = nh_.advertise<sensor_msgs::Imu>(topic_, publisher_queue_);
    last_diagnostics_wall_time_ = ros::WallTime::now();
    last_reconnect_attempt_ = ros::WallTime(0, 0);

    ROS_INFO_STREAM("imu_ros_driver configured: port=" << port_
                    << " baud=" << baud_ << " topic=" << topic_
                    << " expected_rate=" << expected_rate_hz_
                    << "Hz timestamp_source="
                    << (use_device_time_ ? "device" : "arrival"));
    if (use_device_time_) {
      ROS_INFO("Device must output 0x50 Time, 0x51 Acc and 0x52 Gyro frames.");
    } else {
      ROS_WARN("Using serial arrival timestamps. This mode is for diagnostics, "
               "not accurate LiDAR-IMU fusion.");
    }
  }

  void run() {
    ros::WallRate poll_rate(poll_rate_hz_);
    while (ros::ok()) {
      ensureSerialOpen();
      readAvailableBytes();
      reportDiagnostics();
      ros::spinOnce();
      poll_rate.sleep();
    }
    closeSerial();
  }

 private:
  void loadParameters() {
    private_nh_.param<std::string>("port", port_, "/dev/ttyUSB2");
    private_nh_.param<int>("baud", baud_, 115200);
    private_nh_.param<std::string>("topic", topic_, "/imu/data_raw");
    private_nh_.param<std::string>("frame_id", frame_id_, "imu_link");
    private_nh_.param<std::string>("timestamp_source", timestamp_source_,
                                   "device");
    private_nh_.param<double>("expected_rate_hz", expected_rate_hz_, 200.0);
    private_nh_.param<double>("timestamp_offset_sec", timestamp_offset_sec_,
                              0.0);
    private_nh_.param<double>("max_gap_factor", max_gap_factor_, 1.5);
    private_nh_.param<double>("max_clock_skew_sec", max_clock_skew_sec_, 0.5);
    private_nh_.param<double>("poll_rate_hz", poll_rate_hz_, 1000.0);
    private_nh_.param<double>("diagnostics_period_sec", diagnostics_period_sec_,
                              5.0);
    private_nh_.param<double>("rate_tolerance_fraction",
                              rate_tolerance_fraction_, 0.05);
    private_nh_.param<double>("reconnect_interval_sec",
                              reconnect_interval_sec_, 1.0);
    private_nh_.param<int>("serial_timeout_ms", serial_timeout_ms_, 20);
    private_nh_.param<int>("publisher_queue", publisher_queue_, 1000);
    private_nh_.param<double>("gyro_variance", gyro_variance_, 0.0);
    private_nh_.param<double>("accel_variance", accel_variance_, 0.0);

    if (expected_rate_hz_ <= 0.0) {
      ROS_WARN("expected_rate_hz must be positive; using 200 Hz.");
      expected_rate_hz_ = 200.0;
    }
    if (poll_rate_hz_ <= 0.0) {
      poll_rate_hz_ = 1000.0;
    }
    if (diagnostics_period_sec_ <= 0.0) {
      diagnostics_period_sec_ = 5.0;
    }
    if (publisher_queue_ <= 0) {
      publisher_queue_ = 1000;
    }
    use_device_time_ = timestamp_source_ != "arrival";
    if (timestamp_source_ != "device" && timestamp_source_ != "arrival") {
      ROS_WARN_STREAM("Unknown timestamp_source='" << timestamp_source_
                      << "'; using device time.");
      timestamp_source_ = "device";
      use_device_time_ = true;
    }
  }

  void ensureSerialOpen() {
    if (serial_.isOpen()) {
      return;
    }

    const ros::WallTime now = ros::WallTime::now();
    if ((now - last_reconnect_attempt_).toSec() < reconnect_interval_sec_) {
      return;
    }
    last_reconnect_attempt_ = now;

    try {
      serial_.setPort(port_);
      serial_.setBaudrate(static_cast<std::uint32_t>(baud_));
      serial::Timeout timeout =
        serial::Timeout::simpleTimeout(serial_timeout_ms_);
      serial_.setTimeout(timeout);
      serial_.open();
      serial_.flushInput();
      parser_.resetBuffer();
      assembler_.reset();
      device_stamp_mapper_.reset();
      arrival_stamp_mapper_.reset();
      ROS_INFO_STREAM("Opened IMU serial port " << port_ << " @ " << baud_);
    } catch (const std::exception& error) {
      ROS_ERROR_STREAM_THROTTLE(5.0, "Cannot open IMU serial port " << port_
                                          << ": " << error.what());
      closeSerial();
    }
  }

  void readAvailableBytes() {
    if (!serial_.isOpen()) {
      return;
    }

    try {
      const std::size_t available = serial_.available();
      if (available == 0) {
        return;
      }

      constexpr std::size_t kMaxReadSize = 4096;
      const std::size_t read_size = std::min(available, kMaxReadSize);
      std::vector<std::uint8_t> bytes(read_size);
      const std::size_t bytes_read = serial_.read(bytes.data(), bytes.size());
      parser_.append(bytes.data(), bytes_read, [this](const Frame& frame) {
        processFrame(frame);
      });
    } catch (const std::exception& error) {
      ROS_ERROR_STREAM("IMU serial read failed: " << error.what());
      closeSerial();
      parser_.resetBuffer();
      assembler_.reset();
      device_stamp_mapper_.reset();
      arrival_stamp_mapper_.reset();
    }
  }

  void processFrame(const Frame& frame) {
    if (frame.type == kTypeTime) {
      ++time_frames_;
    } else if (frame.type == kTypeAcceleration) {
      ++acceleration_frames_;
    } else if (frame.type == kTypeAngularVelocity) {
      ++angular_velocity_frames_;
    } else {
      ++other_frames_;
    }

    const ros::Time receipt_time = ros::Time::now();
    assembler_.consume(frame, [this, receipt_time](const ImuCycle& cycle) {
      publishCycle(cycle, receipt_time);
    });
  }

  void publishCycle(const ImuCycle& cycle, const ros::Time& receipt_time) {
    ros::Time sample_stamp;
    bool stamp_valid = false;
    if (use_device_time_) {
      stamp_valid = device_stamp_mapper_.map(cycle.device_time, receipt_time,
                                              &sample_stamp);
    } else {
      stamp_valid = arrival_stamp_mapper_.map(receipt_time, &sample_stamp);
    }
    if (!stamp_valid) {
      ++rejected_timestamps_;
      return;
    }

    sensor_msgs::Imu message;
    message.header.seq = sequence_++;
    message.header.stamp = sample_stamp;
    message.header.frame_id = frame_id_;

    message.orientation.x = 0.0;
    message.orientation.y = 0.0;
    message.orientation.z = 0.0;
    message.orientation.w = 1.0;
    message.orientation_covariance[0] = -1.0;

    message.linear_acceleration.x = cycle.acceleration[0];
    message.linear_acceleration.y = cycle.acceleration[1];
    message.linear_acceleration.z = cycle.acceleration[2];
    message.angular_velocity.x = cycle.angular_velocity[0];
    message.angular_velocity.y = cycle.angular_velocity[1];
    message.angular_velocity.z = cycle.angular_velocity[2];

    setDiagonalCovariance(accel_variance_,
                          message.linear_acceleration_covariance);
    setDiagonalCovariance(gyro_variance_,
                          message.angular_velocity_covariance);

    imu_publisher_.publish(message);
    ++published_samples_;
    last_publish_wall_time_ = ros::WallTime::now();
  }

  static void setDiagonalCovariance(
      double variance, boost::array<double, 9>& covariance) {
    covariance.assign(0.0);
    if (variance > 0.0) {
      covariance[0] = variance;
      covariance[4] = variance;
      covariance[8] = variance;
    }
  }

  void reportDiagnostics() {
    const ros::WallTime now = ros::WallTime::now();
    const double elapsed = (now - last_diagnostics_wall_time_).toSec();
    if (elapsed < diagnostics_period_sec_) {
      return;
    }

    const std::uint64_t samples_since_last =
        published_samples_ - last_diagnostics_sample_count_;
    const double measured_rate = samples_since_last / elapsed;
    const double allowed_error = expected_rate_hz_ * rate_tolerance_fraction_;

    if (samples_since_last == 0) {
      ROS_ERROR_STREAM("No IMU samples published in the last " << elapsed
                       << " s. Check that the module outputs "
                          "Time + Acc + Gyro and timestamp_source is correct.");
    } else if (std::abs(measured_rate - expected_rate_hz_) > allowed_error) {
      ROS_WARN_STREAM("IMU publish rate " << measured_rate
                      << " Hz differs from expected " << expected_rate_hz_
                      << " Hz.");
    } else {
      ROS_INFO_STREAM("IMU rate=" << measured_rate
                      << " Hz published=" << published_samples_
                      << " frames(time/acc/gyro/other)=" << time_frames_ << "/"
                      << acceleration_frames_ << "/" << angular_velocity_frames_
                      << "/" << other_frames_
                      << " checksum_errors=" << parser_.checksumErrors()
                      << " incomplete_cycles=" << assembler_.incompleteCycles()
                      << " missing_est="
                      << device_stamp_mapper_.estimatedMissingSamples());
    }

    if (use_device_time_ && time_frames_ == 0) {
      ROS_ERROR("No 0x50 device-time frames received. Enable Time output in "
                "the WIT configuration tool.");
    }
    if (assembler_.incompleteCycles() > last_incomplete_cycle_count_ ||
        parser_.checksumErrors() > last_checksum_error_count_) {
      ROS_WARN_STREAM("IMU data loss detected: incomplete_cycles="
                      << assembler_.incompleteCycles()
                      << " orphan_acc="
                      << assembler_.orphanAccelerationFrames()
                      << " orphan_gyro="
                      << assembler_.orphanAngularVelocityFrames()
                      << " checksum_errors=" << parser_.checksumErrors());
    }
    if (device_stamp_mapper_.clockResets() > last_clock_reset_count_ ||
        device_stamp_mapper_.duplicateTimestamps() >
            last_duplicate_timestamp_count_) {
      ROS_WARN_STREAM("IMU device-time anomaly: resets="
                      << device_stamp_mapper_.clockResets()
                      << " duplicates="
                      << device_stamp_mapper_.duplicateTimestamps()
                      << " reanchors="
                      << device_stamp_mapper_.clockReanchors()
                      << " skew_events="
                      << device_stamp_mapper_.clockSkewEvents());
    }

    last_diagnostics_wall_time_ = now;
    last_diagnostics_sample_count_ = published_samples_;
    last_incomplete_cycle_count_ = assembler_.incompleteCycles();
    last_checksum_error_count_ = parser_.checksumErrors();
    last_clock_reset_count_ = device_stamp_mapper_.clockResets();
    last_duplicate_timestamp_count_ =
        device_stamp_mapper_.duplicateTimestamps();
  }

  void closeSerial() {
    try {
      if (serial_.isOpen()) {
        serial_.close();
      }
    } catch (const std::exception& error) {
      ROS_WARN_STREAM("Error while closing IMU serial port: " << error.what());
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher imu_publisher_;
  serial::Serial serial_;
  StreamParser parser_;
  CycleAssembler assembler_;
  DeviceStampMapper device_stamp_mapper_;
  ArrivalStampMapper arrival_stamp_mapper_;

  std::string port_;
  std::string topic_;
  std::string frame_id_;
  std::string timestamp_source_;
  int baud_ = 115200;
  int serial_timeout_ms_ = 20;
  int publisher_queue_ = 1000;
  double expected_rate_hz_ = 200.0;
  double timestamp_offset_sec_ = 0.0;
  double max_gap_factor_ = 1.5;
  double max_clock_skew_sec_ = 0.5;
  double poll_rate_hz_ = 1000.0;
  double diagnostics_period_sec_ = 5.0;
  double rate_tolerance_fraction_ = 0.05;
  double reconnect_interval_sec_ = 1.0;
  double gyro_variance_ = 0.0;
  double accel_variance_ = 0.0;
  bool use_device_time_ = true;

  std::uint32_t sequence_ = 0;
  std::uint64_t time_frames_ = 0;
  std::uint64_t acceleration_frames_ = 0;
  std::uint64_t angular_velocity_frames_ = 0;
  std::uint64_t other_frames_ = 0;
  std::uint64_t published_samples_ = 0;
  std::uint64_t rejected_timestamps_ = 0;

  ros::WallTime last_reconnect_attempt_;
  ros::WallTime last_diagnostics_wall_time_;
  ros::WallTime last_publish_wall_time_;
  std::uint64_t last_diagnostics_sample_count_ = 0;
  std::uint64_t last_incomplete_cycle_count_ = 0;
  std::uint64_t last_checksum_error_count_ = 0;
  std::uint64_t last_clock_reset_count_ = 0;
  std::uint64_t last_duplicate_timestamp_count_ = 0;
};

}  // namespace
}  // namespace imu_ros_driver

int main(int argc, char** argv) {
  ros::init(argc, argv, "imu_ros_driver");
  imu_ros_driver::ImuRosDriver driver;
  driver.run();
  return 0;
}
