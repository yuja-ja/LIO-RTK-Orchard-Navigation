#include <ros/ros.h>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/TimeReference.h>
#include <serial/serial.h>
#include <std_msgs/String.h>

#include <nmea_rtk_driver/ReceiverStatus.h>
#include <nmea_rtk_driver/RtkEpoch.h>
#include <nmea_rtk_driver/RtkHeading.h>
#include <nmea_rtk_driver/epoch_assembler.hpp>
#include <nmea_rtk_driver/nmea_framer.hpp>
#include <nmea_rtk_driver/nmea_parser.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace nmea_rtk_driver {
namespace {

constexpr double kPi = 3.14159265358979323846;

std::string lower(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    if (c >= 'A' && c <= 'Z') {
      return static_cast<char>(c - 'A' + 'a');
    }
    return static_cast<char>(c);
  });
  return value;
}

template <typename T>
std::string numberString(const T& value) {
  std::ostringstream stream;
  stream << value;
  return stream.str();
}

diagnostic_msgs::KeyValue keyValue(const std::string& key,
                                   const std::string& value) {
  diagnostic_msgs::KeyValue output;
  output.key = key;
  output.value = value;
  return output;
}

double quietNan() {
  return std::numeric_limits<double>::quiet_NaN();
}

ros::Time rosTimeFromSec(double seconds) {
  ros::Time output;
  if (std::isfinite(seconds) && seconds >= 0.0 &&
      seconds <= static_cast<double>(std::numeric_limits<std::uint32_t>::max())) {
    output.fromSec(seconds);
  }
  return output;
}

protocol::PositionPolicy positionPolicy(const std::string& value) {
  if (value == "gga") {
    return protocol::PositionPolicy::kGgaOnly;
  }
  if (value == "gns") {
    return protocol::PositionPolicy::kGnsOnly;
  }
  return protocol::PositionPolicy::kAuto;
}

protocol::VelocityPolicy velocityPolicy(const std::string& value) {
  if (value == "vtg") {
    return protocol::VelocityPolicy::kVtgOnly;
  }
  if (value == "auto") {
    return protocol::VelocityPolicy::kAuto;
  }
  return protocol::VelocityPolicy::kRmcOnly;
}

class NmeaRtkDriver {
 public:
  NmeaRtkDriver() : private_nh_("~") {
    loadParameters();
    assembler_.reset(new protocol::EpochAssembler(assembler_config_));
    framer_.reset(new protocol::NmeaFramer(max_sentence_length_));

    epoch_pub_ = nh_.advertise<RtkEpoch>(epoch_topic_, publisher_queue_);
    fix_pub_ = nh_.advertise<sensor_msgs::NavSatFix>(fix_topic_, publisher_queue_);
    velocity_pub_ = nh_.advertise<geometry_msgs::TwistWithCovarianceStamped>(
        velocity_topic_, publisher_queue_);
    heading_pub_ =
        nh_.advertise<RtkHeading>(heading_topic_, publisher_queue_);
    receiver_status_pub_ = nh_.advertise<ReceiverStatus>(
        receiver_status_topic_, publisher_queue_);
    time_reference_pub_ = nh_.advertise<sensor_msgs::TimeReference>(
        time_reference_topic_, publisher_queue_);
    diagnostics_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>(
        diagnostics_topic_, 10);
    if (publish_raw_) {
      raw_pub_ = nh_.advertise<std_msgs::String>(raw_topic_, publisher_queue_);
    }

    const ros::WallTime now = ros::WallTime::now();
    start_wall_ = now;
    next_reconnect_wall_ = now;
    last_diagnostic_wall_ = now;
    last_valid_sentence_wall_ = now;
    last_epoch_wall_ = now;

    ROS_INFO_STREAM("nmea_rtk_driver configured: port=" << port_
                    << " baud=" << baud_ << " position_source="
                    << position_source_ << " velocity_source="
                    << velocity_source_ << " header_time=ROS arrival");
    ROS_INFO("The driver is read-only and never sends receiver configuration commands.");
  }

  void spin() {
    ros::WallRate rate(std::max(1.0, poll_rate_hz_));
    while (ros::ok()) {
      ensureSerialOpen();
      if (serial_.isOpen()) {
        readAvailableBytes();
      }
      publishCompletedEpochs(assembler_->flush(ros::WallTime::now().toSec()));
      maybePublishDiagnostics();
      ros::spinOnce();
      rate.sleep();
    }
    closeSerial();
  }

 private:
  void loadParameters() {
    private_nh_.param<std::string>("port", port_, "/dev/ttyUSB1");
    private_nh_.param<int>("baud", baud_, 115200);
    private_nh_.param<int>("serial_timeout_ms", serial_timeout_ms_, 20);
    private_nh_.param<double>("reconnect_delay_sec", reconnect_delay_sec_, 1.0);
    private_nh_.param<double>("poll_rate_hz", poll_rate_hz_, 1000.0);
    int maximum_length = 1024;
    private_nh_.param<int>("max_sentence_length", maximum_length, 1024);
    max_sentence_length_ =
        static_cast<std::size_t>(std::max(82, maximum_length));
    private_nh_.param<int>("publisher_queue", publisher_queue_, 100);
    private_nh_.param<double>("timestamp_offset_sec", timestamp_offset_sec_, 0.0);
    private_nh_.param<double>("serial_bits_per_byte", serial_bits_per_byte_, 10.0);

    private_nh_.param<std::string>("frame_id", frame_id_, "gnss_antenna");
    private_nh_.param<std::string>("velocity_frame_id", velocity_frame_id_, "enu");
    private_nh_.param<std::string>("epoch_topic", epoch_topic_, "/rtk/epoch");
    private_nh_.param<std::string>("fix_topic", fix_topic_, "/rtk/fix");
    private_nh_.param<std::string>("velocity_topic", velocity_topic_,
                                   "/rtk/velocity");
    private_nh_.param<std::string>("heading_topic", heading_topic_,
                                   "/rtk/heading");
    private_nh_.param<std::string>("receiver_status_topic",
                                   receiver_status_topic_, "/rtk/status");
    private_nh_.param<std::string>("time_reference_topic",
                                   time_reference_topic_,
                                   "/rtk/time_reference");
    private_nh_.param<std::string>("raw_topic", raw_topic_,
                                   "/rtk/raw_sentence");
    private_nh_.param<std::string>("diagnostics_topic", diagnostics_topic_,
                                   "/diagnostics");

    private_nh_.param<bool>("publish_raw", publish_raw_, true);
    private_nh_.param<bool>("require_checksum", require_checksum_, true);
    private_nh_.param<bool>("publish_invalid_fix", publish_invalid_fix_, true);
    private_nh_.param<bool>("publish_invalid_velocity", publish_invalid_velocity_,
                            false);
    private_nh_.param<bool>("publish_invalid_heading", publish_invalid_heading_,
                            true);
    private_nh_.param<int>("navsat_service_mask", navsat_service_mask_, 15);

    private_nh_.param<std::string>("position_source", position_source_, "auto");
    private_nh_.param<std::string>("preferred_position_sentence",
                                   preferred_position_sentence_, "gga");
    private_nh_.param<std::string>("velocity_source", velocity_source_, "rmc");
    private_nh_.param<std::string>("preferred_velocity_sentence",
                                   preferred_velocity_sentence_, "rmc");
    position_source_ = lower(position_source_);
    preferred_position_sentence_ = lower(preferred_position_sentence_);
    velocity_source_ = lower(velocity_source_);
    preferred_velocity_sentence_ = lower(preferred_velocity_sentence_);
    assembler_config_.position_policy = positionPolicy(position_source_);
    assembler_config_.velocity_policy = velocityPolicy(velocity_source_);
    assembler_config_.preferred_position = preferred_position_sentence_ == "gns"
                                               ? protocol::SentenceKind::kGns
                                               : protocol::SentenceKind::kGga;
    assembler_config_.preferred_velocity = preferred_velocity_sentence_ == "vtg"
                                               ? protocol::SentenceKind::kVtg
                                               : protocol::SentenceKind::kRmc;
    private_nh_.param<double>("utc_match_tolerance_sec",
                              assembler_config_.utc_match_tolerance_sec, 0.05);
    private_nh_.param<double>("epoch_assembly_delay_sec",
                              assembler_config_.assembly_delay_sec, 0.08);
    private_nh_.param<double>("epoch_stale_timeout_sec",
                              assembler_config_.stale_timeout_sec, 0.50);
    private_nh_.param<double>("duplicate_retention_sec",
                              assembler_config_.duplicate_retention_sec, 2.0);
    private_nh_.param<double>("vtg_match_window_sec",
                              assembler_config_.vtg_match_window_sec, 0.15);
    private_nh_.param<double>("gsa_max_age_sec",
                              assembler_config_.gsa_max_age_sec, 2.0);

    private_nh_.param<bool>("use_gst_covariance", use_gst_covariance_, true);
    private_nh_.param<bool>("wait_for_gst",
                            assembler_config_.require_covariance, true);
    private_nh_.param<bool>("wait_for_velocity",
                            assembler_config_.require_velocity, true);
    private_nh_.param<double>("gst_max_std_m", gst_max_std_m_, 100.0);
    private_nh_.param<double>("rtk_fixed_horizontal_std_m",
                              rtk_fixed_horizontal_std_m_, 0.03);
    private_nh_.param<double>("rtk_fixed_vertical_std_m",
                              rtk_fixed_vertical_std_m_, 0.06);
    private_nh_.param<double>("rtk_float_horizontal_std_m",
                              rtk_float_horizontal_std_m_, 0.30);
    private_nh_.param<double>("rtk_float_vertical_std_m",
                              rtk_float_vertical_std_m_, 0.60);
    private_nh_.param<double>("rtk_unspecified_horizontal_std_m",
                              rtk_unspecified_horizontal_std_m_, 0.50);
    private_nh_.param<double>("rtk_unspecified_vertical_std_m",
                              rtk_unspecified_vertical_std_m_, 1.00);
    private_nh_.param<double>("differential_horizontal_std_m",
                              differential_horizontal_std_m_, 1.0);
    private_nh_.param<double>("differential_vertical_std_m",
                              differential_vertical_std_m_, 2.0);
    private_nh_.param<double>("single_horizontal_std_m",
                              single_horizontal_std_m_, 3.0);
    private_nh_.param<double>("single_vertical_std_m", single_vertical_std_m_,
                              6.0);
    private_nh_.param<double>("unknown_horizontal_std_m",
                              unknown_horizontal_std_m_, 10.0);
    private_nh_.param<double>("unknown_vertical_std_m", unknown_vertical_std_m_,
                              20.0);
    private_nh_.param<double>("velocity_horizontal_std_mps",
                              velocity_horizontal_std_mps_, 0.20);
    private_nh_.param<double>("velocity_vertical_std_mps",
                              velocity_vertical_std_mps_, 100.0);
    private_nh_.param<double>("heading_std_deg", heading_std_deg_, 2.0);
    private_nh_.param<std::string>("heading_valid_modes", heading_valid_modes_,
                                   "A");

    private_nh_.param<std::string>("time_reference_source",
                                   time_reference_source_, "auto");
    time_reference_source_ = lower(time_reference_source_);
    private_nh_.param<int>("rmc_year_pivot", rmc_year_pivot_, 80);
    private_nh_.param<double>("zda_preference_timeout_sec",
                              zda_preference_timeout_sec_, 2.0);
    private_nh_.param<double>("diagnostic_period_sec", diagnostic_period_sec_,
                              5.0);
    private_nh_.param<double>("expected_epoch_rate_hz", expected_epoch_rate_hz_,
                              10.0);

    baud_ = std::max(1, baud_);
    serial_timeout_ms_ = std::max(1, serial_timeout_ms_);
    publisher_queue_ = std::max(1, publisher_queue_);
    reconnect_delay_sec_ = std::max(0.1, reconnect_delay_sec_);
    poll_rate_hz_ = std::max(1.0, poll_rate_hz_);
    diagnostic_period_sec_ = std::max(0.5, diagnostic_period_sec_);
    rmc_year_pivot_ = std::max(0, std::min(99, rmc_year_pivot_));
    if (!std::isfinite(timestamp_offset_sec_)) {
      timestamp_offset_sec_ = 0.0;
    }
    if (!std::isfinite(serial_bits_per_byte_) || serial_bits_per_byte_ <= 0.0) {
      serial_bits_per_byte_ = 10.0;
    }
  }

  void ensureSerialOpen() {
    if (serial_.isOpen() || ros::WallTime::now() < next_reconnect_wall_) {
      return;
    }
    try {
      serial_.setPort(port_);
      serial_.setBaudrate(static_cast<std::uint32_t>(baud_));
      serial::Timeout timeout = serial::Timeout::simpleTimeout(
          static_cast<std::uint32_t>(serial_timeout_ms_));
      serial_.setTimeout(timeout);
      serial_.open();
      framer_->reset();
      ++open_count_;
      ROS_INFO_STREAM("Opened RTK serial port " << port_ << " @ " << baud_);
    } catch (const std::exception& error) {
      ++open_error_count_;
      next_reconnect_wall_ =
          ros::WallTime::now() + ros::WallDuration(reconnect_delay_sec_);
      ROS_ERROR_STREAM_THROTTLE(5.0, "Cannot open RTK serial port " << port_
                                      << ": " << error.what());
    }
  }

  void closeSerial() {
    if (serial_.isOpen()) {
      try {
        serial_.close();
      } catch (const std::exception&) {
      }
    }
  }

  void readAvailableBytes() {
    try {
      const std::size_t available = serial_.available();
      if (available == 0U) {
        return;
      }
      const std::string bytes = serial_.read(std::min<std::size_t>(available, 4096U));
      byte_count_ += bytes.size();
      const ros::Time chunk_end_stamp = ros::Time::now();
      const ros::WallTime chunk_end_wall = ros::WallTime::now();
      const double seconds_per_byte =
          serial_bits_per_byte_ / static_cast<double>(baud_);
      for (std::size_t index = 0U; index < bytes.size(); ++index) {
        const char byte = bytes[index];
        const bool was_collecting = framer_->collecting();
        std::string sentence;
        const protocol::FrameEvent event = framer_->push(byte, &sentence);
        if (event == protocol::FrameEvent::kStarted) {
          if (was_collecting) {
            ++framing_resync_count_;
          }
          const double trailing_bytes =
              static_cast<double>(bytes.size() - 1U - index);
          sentence_arrival_stamp_ = rosTimeFromSec(std::max(
              0.0, chunk_end_stamp.toSec() - trailing_bytes * seconds_per_byte +
                       timestamp_offset_sec_));
          sentence_arrival_wall_.fromSec(std::max(
              0.0, chunk_end_wall.toSec() - trailing_bytes * seconds_per_byte));
        } else if (event == protocol::FrameEvent::kCompleted) {
          processSentence(sentence, sentence_arrival_stamp_, sentence_arrival_wall_);
        } else if (event == protocol::FrameEvent::kDroppedMalformed) {
          ++framing_error_count_;
        } else if (event == protocol::FrameEvent::kDroppedOversize) {
          ++oversize_count_;
        }
      }
    } catch (const std::exception& error) {
      ++read_error_count_;
      ROS_ERROR_STREAM("RTK serial read failed: " << error.what());
      closeSerial();
      framer_->reset();
      next_reconnect_wall_ =
          ros::WallTime::now() + ros::WallDuration(reconnect_delay_sec_);
    }
  }

  void processSentence(const std::string& sentence,
                       const ros::Time& arrival_stamp,
                       const ros::WallTime& arrival_wall) {
    ++line_count_;
    if (publish_raw_) {
      std_msgs::String raw;
      raw.data = sentence;
      raw_pub_.publish(raw);
    }

    const protocol::ParseResult parsed = protocol::parseLine(sentence);
    if (!parsed.syntax_ok) {
      ++syntax_error_count_;
      return;
    }
    if ((parsed.checksum_present && !parsed.checksum_valid) ||
        (require_checksum_ && !parsed.checksum_present)) {
      ++checksum_error_count_;
      return;
    }
    if (!parsed.recognized) {
      ++unknown_count_;
      return;
    }
    if (parsed.secondary_antenna) {
      ++secondary_antenna_count_;
      return;
    }

    ++valid_sentence_count_;
    last_valid_sentence_wall_ = arrival_wall;
    assembler_->ingest(parsed, arrival_stamp.toSec(), arrival_wall.toSec());

    if (parsed.kind == protocol::SentenceKind::kThs) {
      publishHeading(parsed, arrival_stamp);
    } else if (parsed.kind == protocol::SentenceKind::kGsa) {
      publishReceiverStatus(parsed, arrival_stamp);
    } else if (parsed.kind == protocol::SentenceKind::kZda) {
      publishTimeReferenceFromZda(parsed, arrival_stamp, arrival_wall);
    } else if (parsed.kind == protocol::SentenceKind::kRmc) {
      publishTimeReferenceFromRmc(parsed, arrival_stamp, arrival_wall);
    }
  }

  void publishHeading(const protocol::ParseResult& parsed,
                      const ros::Time& arrival_stamp) {
    const bool accepted_mode = parsed.ths.mode.size() == 1U &&
                               heading_valid_modes_.find(parsed.ths.mode[0]) !=
                                   std::string::npos;
    const bool valid = parsed.ths.has_heading && accepted_mode;
    if (!valid && !publish_invalid_heading_) {
      return;
    }
    RtkHeading message;
    message.header.stamp = arrival_stamp;
    message.header.frame_id = frame_id_;
    message.arrival_stamp = arrival_stamp;
    message.source_sentence = parsed.formatter;
    message.talker = parsed.talker;
    message.valid = valid;
    message.mode = parsed.ths.mode;
    message.heading_true_deg = parsed.ths.has_heading
                                   ? parsed.ths.heading_true_deg
                                   : quietNan();
    message.heading_true_rad = parsed.ths.has_heading
                                   ? parsed.ths.heading_true_deg * kPi / 180.0
                                   : quietNan();
    message.heading_std_deg = heading_std_deg_;
    message.covariance_from_receiver = false;
    heading_pub_.publish(message);
    ++heading_count_;
  }

  void publishReceiverStatus(const protocol::ParseResult& parsed,
                             const ros::Time& arrival_stamp) {
    ReceiverStatus message;
    message.header.stamp = arrival_stamp;
    message.header.frame_id = frame_id_;
    message.arrival_stamp = arrival_stamp;
    message.source_sentence = parsed.formatter;
    message.talker = parsed.talker;
    message.valid = parsed.gsa.fix_dimension >= 2;
    message.selection_mode = parsed.gsa.selection_mode;
    message.fix_dimension = static_cast<std::uint8_t>(parsed.gsa.fix_dimension);
    message.satellite_ids = parsed.gsa.satellite_ids;
    message.satellites_used =
        static_cast<std::uint16_t>(parsed.gsa.satellite_ids.size());
    message.pdop_valid = parsed.gsa.has_pdop;
    message.pdop = parsed.gsa.has_pdop ? parsed.gsa.pdop : quietNan();
    message.hdop_valid = parsed.gsa.has_hdop;
    message.hdop = parsed.gsa.has_hdop ? parsed.gsa.hdop : quietNan();
    message.vdop_valid = parsed.gsa.has_vdop;
    message.vdop = parsed.gsa.has_vdop ? parsed.gsa.vdop : quietNan();
    message.system_id = parsed.gsa.system_id;
    receiver_status_pub_.publish(message);
    ++status_count_;
  }

  void publishTimeReferenceFromZda(const protocol::ParseResult& parsed,
                                   const ros::Time& arrival_stamp,
                                   const ros::WallTime& arrival_wall) {
    if (time_reference_source_ == "off" || time_reference_source_ == "rmc") {
      return;
    }
    double unix_seconds = 0.0;
    if (!protocol::utcUnixSeconds(parsed.zda.year, parsed.zda.month,
                                  parsed.zda.day,
                                  parsed.zda.utc_seconds_of_day,
                                  &unix_seconds)) {
      return;
    }
    publishTimeReference(arrival_stamp, unix_seconds, "NMEA_ZDA");
    last_zda_wall_ = arrival_wall;
  }

  void publishTimeReferenceFromRmc(const protocol::ParseResult& parsed,
                                   const ros::Time& arrival_stamp,
                                   const ros::WallTime& arrival_wall) {
    if (time_reference_source_ == "off" || time_reference_source_ == "zda" ||
        parsed.rmc.date_ddmmyy.size() != 6U) {
      return;
    }
    if (time_reference_source_ == "auto" && !last_zda_wall_.isZero() &&
        (arrival_wall - last_zda_wall_).toSec() <=
            zda_preference_timeout_sec_) {
      return;
    }
    const int day = std::atoi(parsed.rmc.date_ddmmyy.substr(0U, 2U).c_str());
    const int month = std::atoi(parsed.rmc.date_ddmmyy.substr(2U, 2U).c_str());
    const int two_digit_year =
        std::atoi(parsed.rmc.date_ddmmyy.substr(4U, 2U).c_str());
    const int year = two_digit_year >= rmc_year_pivot_
                         ? 1900 + two_digit_year
                         : 2000 + two_digit_year;
    double unix_seconds = 0.0;
    if (protocol::utcUnixSeconds(year, month, day,
                                 parsed.rmc.utc_seconds_of_day,
                                 &unix_seconds)) {
      publishTimeReference(arrival_stamp, unix_seconds, "NMEA_RMC");
    }
  }

  void publishTimeReference(const ros::Time& arrival_stamp,
                            double unix_seconds,
                            const std::string& source) {
    sensor_msgs::TimeReference message;
    message.header.stamp = arrival_stamp;
    message.header.frame_id = frame_id_;
    message.time_ref = rosTimeFromSec(unix_seconds);
    message.source = source;
    time_reference_pub_.publish(message);
    ++time_reference_count_;
  }

  void configuredPositionStd(int quality,
                             double* horizontal_std,
                             double* vertical_std) const {
    if (quality == 4) {
      *horizontal_std = rtk_fixed_horizontal_std_m_;
      *vertical_std = rtk_fixed_vertical_std_m_;
    } else if (quality == 5) {
      *horizontal_std = rtk_float_horizontal_std_m_;
      *vertical_std = rtk_float_vertical_std_m_;
    } else if (quality == 9) {
      *horizontal_std = rtk_unspecified_horizontal_std_m_;
      *vertical_std = rtk_unspecified_vertical_std_m_;
    } else if (quality == 2) {
      *horizontal_std = differential_horizontal_std_m_;
      *vertical_std = differential_vertical_std_m_;
    } else if (quality == 1 || quality == 3) {
      *horizontal_std = single_horizontal_std_m_;
      *vertical_std = single_vertical_std_m_;
    } else {
      *horizontal_std = unknown_horizontal_std_m_;
      *vertical_std = unknown_vertical_std_m_;
    }
  }

  void publishCompletedEpochs(
      const std::vector<protocol::EpochObservation>& epochs) {
    for (const protocol::EpochObservation& epoch : epochs) {
      publishEpoch(epoch);
    }
  }

  void publishEpoch(const protocol::EpochObservation& epoch) {
    const protocol::PositionObservation& position = epoch.position;
    const protocol::VelocityObservation& velocity = epoch.velocity;
    const ros::Time position_stamp =
        rosTimeFromSec(position.arrival_stamp_sec);

    std::array<double, 9> position_covariance{{0.0, 0.0, 0.0,
                                               0.0, 0.0, 0.0,
                                               0.0, 0.0, 0.0}};
    bool covariance_valid = false;
    std::string covariance_source = "NONE";
    if (position.valid && use_gst_covariance_ &&
        epoch.covariance.has_error_ellipse &&
        epoch.covariance.has_position_std &&
        epoch.covariance.semi_major_std_m > 0.0 &&
        epoch.covariance.semi_minor_std_m > 0.0 &&
        epoch.covariance.altitude_std_m > 0.0 &&
        epoch.covariance.semi_major_std_m <= gst_max_std_m_ &&
        epoch.covariance.semi_minor_std_m <= gst_max_std_m_ &&
        epoch.covariance.altitude_std_m <= gst_max_std_m_) {
      const double orientation_rad =
          epoch.covariance.orientation_deg * kPi / 180.0;
      const double sine = std::sin(orientation_rad);
      const double cosine = std::cos(orientation_rad);
      const double major_variance =
          std::pow(epoch.covariance.semi_major_std_m, 2.0);
      const double minor_variance =
          std::pow(epoch.covariance.semi_minor_std_m, 2.0);
      position_covariance[0] = major_variance * sine * sine +
                               minor_variance * cosine * cosine;
      position_covariance[4] = major_variance * cosine * cosine +
                               minor_variance * sine * sine;
      position_covariance[1] =
          (major_variance - minor_variance) * sine * cosine;
      position_covariance[3] = position_covariance[1];
      position_covariance[8] = std::pow(epoch.covariance.altitude_std_m, 2.0);
      covariance_valid = true;
      covariance_source = "GST_ERROR_ELLIPSE";
    } else if (position.valid && use_gst_covariance_ &&
        epoch.covariance.has_position_std &&
        epoch.covariance.latitude_std_m > 0.0 &&
        epoch.covariance.longitude_std_m > 0.0 &&
        epoch.covariance.altitude_std_m > 0.0 &&
        epoch.covariance.latitude_std_m <= gst_max_std_m_ &&
        epoch.covariance.longitude_std_m <= gst_max_std_m_ &&
        epoch.covariance.altitude_std_m <= gst_max_std_m_) {
      position_covariance[0] = std::pow(epoch.covariance.longitude_std_m, 2.0);
      position_covariance[4] = std::pow(epoch.covariance.latitude_std_m, 2.0);
      position_covariance[8] = std::pow(epoch.covariance.altitude_std_m, 2.0);
      covariance_valid = true;
      covariance_source = "GST_LAT_LON_ALT";
    } else if (position.valid) {
      double horizontal_std = 0.0;
      double vertical_std = 0.0;
      configuredPositionStd(position.fix_quality, &horizontal_std, &vertical_std);
      position_covariance[0] = horizontal_std * horizontal_std;
      position_covariance[4] = horizontal_std * horizontal_std;
      position_covariance[8] = vertical_std * vertical_std;
      covariance_valid = true;
      covariance_source = "CONFIGURED_" +
                          protocol::fixQualityName(position.fix_quality);
    }

    const double track_rad = velocity.track_true_deg * kPi / 180.0;
    const double velocity_east = velocity.present
                                     ? velocity.speed_mps * std::sin(track_rad)
                                     : quietNan();
    const double velocity_north = velocity.present
                                      ? velocity.speed_mps * std::cos(track_rad)
                                      : quietNan();
    const double velocity_horizontal_variance =
        velocity_horizontal_std_mps_ * velocity_horizontal_std_mps_;
    const double velocity_vertical_variance =
        velocity_vertical_std_mps_ * velocity_vertical_std_mps_;

    RtkEpoch message;
    message.header.stamp = position_stamp;
    message.header.frame_id = frame_id_;
    message.publish_stamp = ros::Time::now();
    message.position_arrival_stamp = position_stamp;
    message.covariance_arrival_stamp = epoch.covariance.present
                                           ? rosTimeFromSec(
                                                 epoch.covariance.arrival_stamp_sec)
                                           : ros::Time();
    message.velocity_arrival_stamp = velocity.present
                                         ? rosTimeFromSec(velocity.arrival_stamp_sec)
                                         : ros::Time();
    message.gsa_arrival_stamp = epoch.gsa.present
                                   ? rosTimeFromSec(epoch.gsa.arrival_stamp_sec)
                                   : ros::Time();
    message.utc_time = epoch.utc_text;
    message.utc_date_ddmmyy = epoch.utc_date_ddmmyy;
    message.utc_seconds_of_day = epoch.utc_seconds_of_day;
    message.utc_valid = false;
    message.utc_stamp = ros::Time();
    if (epoch.utc_date_ddmmyy.size() == 6U) {
      const int utc_day =
          std::atoi(epoch.utc_date_ddmmyy.substr(0U, 2U).c_str());
      const int utc_month =
          std::atoi(epoch.utc_date_ddmmyy.substr(2U, 2U).c_str());
      const int two_digit_year =
          std::atoi(epoch.utc_date_ddmmyy.substr(4U, 2U).c_str());
      const int utc_year = two_digit_year >= rmc_year_pivot_
                               ? 1900 + two_digit_year
                               : 2000 + two_digit_year;
      double unix_seconds = 0.0;
      if (protocol::utcUnixSeconds(utc_year, utc_month, utc_day,
                                   epoch.utc_seconds_of_day, &unix_seconds)) {
        message.utc_valid = true;
        message.utc_stamp = rosTimeFromSec(unix_seconds);
      }
    }
    message.position_source = protocol::sentenceKindName(position.source);
    message.position_talker = position.talker;
    message.position_valid = position.valid;
    message.coordinates_valid = position.has_coordinates;
    message.fix_quality = static_cast<std::uint8_t>(position.fix_quality);
    message.fix_mode = position.fix_mode;
    message.navigation_status = position.navigation_status;
    message.satellites_used =
        static_cast<std::uint16_t>(std::max(0, position.satellites_used));
    message.latitude_deg = position.has_coordinates ? position.latitude_deg
                                                    : quietNan();
    message.longitude_deg = position.has_coordinates ? position.longitude_deg
                                                     : quietNan();
    message.msl_height_valid = position.has_msl_height;
    message.msl_height_m = position.has_msl_height ? position.msl_height_m
                                                  : quietNan();
    message.geoid_separation_valid = position.has_geoid_separation;
    message.geoid_separation_m = position.has_geoid_separation
                                     ? position.geoid_separation_m
                                     : quietNan();
    message.ellipsoid_height_valid = position.has_ellipsoid_height;
    message.ellipsoid_height_m = position.has_ellipsoid_height
                                     ? position.ellipsoid_height_m
                                     : quietNan();
    message.differential_age_valid = position.has_differential_age;
    message.differential_age_sec = position.has_differential_age
                                       ? position.differential_age_sec
                                       : quietNan();
    message.differential_station_id = position.differential_station_id;
    message.hdop_valid = position.has_hdop;
    message.hdop = position.has_hdop ? position.hdop : quietNan();
    message.gsa_valid = epoch.gsa.present;
    message.fix_dimension = epoch.gsa.present
                                ? static_cast<std::uint8_t>(epoch.gsa.fix_dimension)
                                : 0U;
    message.pdop_valid = epoch.gsa.present && epoch.gsa.has_pdop;
    message.pdop = message.pdop_valid ? epoch.gsa.pdop : quietNan();
    message.gsa_hdop_valid = epoch.gsa.present && epoch.gsa.has_hdop;
    message.gsa_hdop = message.gsa_hdop_valid ? epoch.gsa.hdop : quietNan();
    message.vdop_valid = epoch.gsa.present && epoch.gsa.has_vdop;
    message.vdop = message.vdop_valid ? epoch.gsa.vdop : quietNan();
    message.position_covariance_valid = covariance_valid;
    message.position_covariance_source = covariance_source;
    for (std::size_t index = 0U; index < 9U; ++index) {
      message.position_covariance_enu[index] = position_covariance[index];
    }
    message.gst_rms_valid = epoch.covariance.has_rms;
    message.gst_rms_m = epoch.covariance.has_rms ? epoch.covariance.rms_m
                                                : quietNan();
    message.gst_error_ellipse_valid = epoch.covariance.has_error_ellipse;
    message.gst_semi_major_std_m = epoch.covariance.has_error_ellipse
                                       ? epoch.covariance.semi_major_std_m
                                       : quietNan();
    message.gst_semi_minor_std_m = epoch.covariance.has_error_ellipse
                                       ? epoch.covariance.semi_minor_std_m
                                       : quietNan();
    message.gst_orientation_deg = epoch.covariance.has_error_ellipse
                                      ? epoch.covariance.orientation_deg
                                      : quietNan();
    message.gst_position_std_valid = epoch.covariance.has_position_std;
    message.gst_latitude_std_m = epoch.covariance.has_position_std
                                     ? epoch.covariance.latitude_std_m
                                     : quietNan();
    message.gst_longitude_std_m = epoch.covariance.has_position_std
                                      ? epoch.covariance.longitude_std_m
                                      : quietNan();
    message.gst_altitude_std_m = epoch.covariance.has_position_std
                                     ? epoch.covariance.altitude_std_m
                                     : quietNan();
    message.velocity_present = velocity.present;
    message.velocity_valid = velocity.valid;
    message.velocity_covariance_valid = velocity.valid;
    message.velocity_source = protocol::sentenceKindName(velocity.source);
    message.velocity_talker = velocity.talker;
    message.velocity_mode = velocity.mode;
    message.velocity_navigation_status = velocity.navigation_status;
    message.speed_mps = velocity.present ? velocity.speed_mps : quietNan();
    message.track_true_deg = velocity.present ? velocity.track_true_deg
                                              : quietNan();
    message.velocity_enu_mps.x = velocity_east;
    message.velocity_enu_mps.y = velocity_north;
    message.velocity_enu_mps.z = velocity.present ? 0.0 : quietNan();
    message.velocity_covariance_enu[0] = velocity_horizontal_variance;
    message.velocity_covariance_enu[4] = velocity_horizontal_variance;
    message.velocity_covariance_enu[8] = velocity_vertical_variance;
    epoch_pub_.publish(message);

    if (position.valid || publish_invalid_fix_) {
      sensor_msgs::NavSatFix fix;
      fix.header = message.header;
      fix.status.status = position.valid ? sensor_msgs::NavSatStatus::STATUS_FIX
                                         : sensor_msgs::NavSatStatus::STATUS_NO_FIX;
      fix.status.service = static_cast<std::uint16_t>(navsat_service_mask_);
      fix.latitude = position.has_coordinates ? position.latitude_deg : quietNan();
      fix.longitude = position.has_coordinates ? position.longitude_deg : quietNan();
      fix.altitude = position.has_ellipsoid_height
                         ? position.ellipsoid_height_m
                         : quietNan();
      for (std::size_t index = 0U; index < 9U; ++index) {
        fix.position_covariance[index] = position_covariance[index];
      }
      fix.position_covariance_type =
          covariance_valid
              ? (covariance_source == "GST_ERROR_ELLIPSE"
                     ? sensor_msgs::NavSatFix::COVARIANCE_TYPE_KNOWN
                     : sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN)
              : sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
      fix_pub_.publish(fix);
    }

    if (velocity.present && (velocity.valid || publish_invalid_velocity_)) {
      geometry_msgs::TwistWithCovarianceStamped twist;
      twist.header.stamp = rosTimeFromSec(velocity.arrival_stamp_sec);
      twist.header.frame_id = velocity_frame_id_;
      twist.twist.twist.linear.x = velocity_east;
      twist.twist.twist.linear.y = velocity_north;
      twist.twist.twist.linear.z = 0.0;
      twist.twist.covariance[0] = velocity_horizontal_variance;
      twist.twist.covariance[7] = velocity_horizontal_variance;
      twist.twist.covariance[14] = velocity_vertical_variance;
      twist.twist.covariance[21] = 1.0e6;
      twist.twist.covariance[28] = 1.0e6;
      twist.twist.covariance[35] = 1.0e6;
      velocity_pub_.publish(twist);
    }

    ++epoch_count_;
    last_epoch_wall_ = ros::WallTime::now();
  }

  void maybePublishDiagnostics() {
    const ros::WallTime now = ros::WallTime::now();
    if ((now - last_diagnostic_wall_).toSec() < diagnostic_period_sec_) {
      return;
    }
    last_diagnostic_wall_ = now;
    const double elapsed = std::max(1.0e-6, (now - start_wall_).toSec());
    const double epoch_rate = static_cast<double>(epoch_count_) / elapsed;

    diagnostic_msgs::DiagnosticArray array;
    array.header.stamp = ros::Time::now();
    diagnostic_msgs::DiagnosticStatus status;
    status.name = "nmea_rtk_driver";
    status.hardware_id = port_;
    if (!serial_.isOpen()) {
      status.level = diagnostic_msgs::DiagnosticStatus::ERROR;
      status.message = "serial port disconnected";
    } else if ((now - last_valid_sentence_wall_).toSec() >
               2.0 * diagnostic_period_sec_) {
      status.level = diagnostic_msgs::DiagnosticStatus::WARN;
      status.message = "no valid NMEA sentence recently";
    } else if (expected_epoch_rate_hz_ > 0.0 && epoch_count_ > 0U &&
               epoch_rate < 0.5 * expected_epoch_rate_hz_) {
      status.level = diagnostic_msgs::DiagnosticStatus::WARN;
      status.message = "epoch rate below configured expectation";
    } else {
      status.level = diagnostic_msgs::DiagnosticStatus::OK;
      status.message = "receiving NMEA";
    }
    status.values.push_back(keyValue("serial_open", serial_.isOpen() ? "true" : "false"));
    status.values.push_back(keyValue("bytes", numberString(byte_count_)));
    status.values.push_back(keyValue("lines", numberString(line_count_)));
    status.values.push_back(keyValue("valid_sentences", numberString(valid_sentence_count_)));
    status.values.push_back(keyValue("epochs", numberString(epoch_count_)));
    status.values.push_back(keyValue("epoch_rate_hz", numberString(epoch_rate)));
    status.values.push_back(keyValue("syntax_errors", numberString(syntax_error_count_)));
    status.values.push_back(keyValue("checksum_errors", numberString(checksum_error_count_)));
    status.values.push_back(keyValue("secondary_antenna_rejected", numberString(secondary_antenna_count_)));
    status.values.push_back(keyValue("unknown_sentences", numberString(unknown_count_)));
    status.values.push_back(keyValue("framing_errors", numberString(framing_error_count_)));
    status.values.push_back(keyValue("framing_resyncs", numberString(framing_resync_count_)));
    status.values.push_back(keyValue("oversize_sentences", numberString(oversize_count_)));
    status.values.push_back(keyValue("epoch_duplicates", numberString(assembler_->duplicateCount())));
    status.values.push_back(keyValue("epoch_stale_drops", numberString(assembler_->staleDropCount())));
    status.values.push_back(keyValue("serial_open_errors", numberString(open_error_count_)));
    status.values.push_back(keyValue("serial_read_errors", numberString(read_error_count_)));
    array.status.push_back(status);
    diagnostics_pub_.publish(array);

    ROS_INFO_STREAM("RTK epoch_rate=" << epoch_rate << " Hz epochs="
                    << epoch_count_ << " lines=" << line_count_
                    << " checksum_errors=" << checksum_error_count_
                    << " secondary_rejected=" << secondary_antenna_count_);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher epoch_pub_;
  ros::Publisher fix_pub_;
  ros::Publisher velocity_pub_;
  ros::Publisher heading_pub_;
  ros::Publisher receiver_status_pub_;
  ros::Publisher time_reference_pub_;
  ros::Publisher raw_pub_;
  ros::Publisher diagnostics_pub_;
  serial::Serial serial_;
  std::unique_ptr<protocol::EpochAssembler> assembler_;
  std::unique_ptr<protocol::NmeaFramer> framer_;
  protocol::AssemblerConfig assembler_config_;

  std::string port_;
  int baud_ = 115200;
  int serial_timeout_ms_ = 20;
  double reconnect_delay_sec_ = 1.0;
  double poll_rate_hz_ = 1000.0;
  std::size_t max_sentence_length_ = 1024U;
  int publisher_queue_ = 100;
  double timestamp_offset_sec_ = 0.0;
  double serial_bits_per_byte_ = 10.0;
  std::string frame_id_;
  std::string velocity_frame_id_;
  std::string epoch_topic_;
  std::string fix_topic_;
  std::string velocity_topic_;
  std::string heading_topic_;
  std::string receiver_status_topic_;
  std::string time_reference_topic_;
  std::string raw_topic_;
  std::string diagnostics_topic_;
  bool publish_raw_ = true;
  bool require_checksum_ = true;
  bool publish_invalid_fix_ = true;
  bool publish_invalid_velocity_ = false;
  bool publish_invalid_heading_ = true;
  int navsat_service_mask_ = 15;
  std::string position_source_;
  std::string preferred_position_sentence_;
  std::string velocity_source_;
  std::string preferred_velocity_sentence_;
  bool use_gst_covariance_ = true;
  double gst_max_std_m_ = 100.0;
  double rtk_fixed_horizontal_std_m_ = 0.03;
  double rtk_fixed_vertical_std_m_ = 0.06;
  double rtk_float_horizontal_std_m_ = 0.30;
  double rtk_float_vertical_std_m_ = 0.60;
  double rtk_unspecified_horizontal_std_m_ = 0.50;
  double rtk_unspecified_vertical_std_m_ = 1.00;
  double differential_horizontal_std_m_ = 1.0;
  double differential_vertical_std_m_ = 2.0;
  double single_horizontal_std_m_ = 3.0;
  double single_vertical_std_m_ = 6.0;
  double unknown_horizontal_std_m_ = 10.0;
  double unknown_vertical_std_m_ = 20.0;
  double velocity_horizontal_std_mps_ = 0.20;
  double velocity_vertical_std_mps_ = 100.0;
  double heading_std_deg_ = 2.0;
  std::string heading_valid_modes_ = "A";
  std::string time_reference_source_ = "auto";
  int rmc_year_pivot_ = 80;
  double zda_preference_timeout_sec_ = 2.0;
  double diagnostic_period_sec_ = 5.0;
  double expected_epoch_rate_hz_ = 10.0;

  ros::Time sentence_arrival_stamp_;
  ros::WallTime sentence_arrival_wall_;
  ros::WallTime start_wall_;
  ros::WallTime next_reconnect_wall_;
  ros::WallTime last_diagnostic_wall_;
  ros::WallTime last_valid_sentence_wall_;
  ros::WallTime last_epoch_wall_;
  ros::WallTime last_zda_wall_;
  std::uint64_t byte_count_ = 0U;
  std::uint64_t line_count_ = 0U;
  std::uint64_t valid_sentence_count_ = 0U;
  std::uint64_t epoch_count_ = 0U;
  std::uint64_t heading_count_ = 0U;
  std::uint64_t status_count_ = 0U;
  std::uint64_t time_reference_count_ = 0U;
  std::uint64_t syntax_error_count_ = 0U;
  std::uint64_t checksum_error_count_ = 0U;
  std::uint64_t secondary_antenna_count_ = 0U;
  std::uint64_t unknown_count_ = 0U;
  std::uint64_t framing_error_count_ = 0U;
  std::uint64_t framing_resync_count_ = 0U;
  std::uint64_t oversize_count_ = 0U;
  std::uint64_t open_count_ = 0U;
  std::uint64_t open_error_count_ = 0U;
  std::uint64_t read_error_count_ = 0U;
};

}  // namespace
}  // namespace nmea_rtk_driver

int main(int argc, char** argv) {
  ros::init(argc, argv, "nmea_rtk_driver");
  nmea_rtk_driver::NmeaRtkDriver driver;
  driver.spin();
  return 0;
}
