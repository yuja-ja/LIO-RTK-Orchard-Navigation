#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>

namespace imu_ros_driver {

constexpr std::uint8_t kFrameHeader = 0x55;
constexpr std::uint8_t kTypeTime = 0x50;
constexpr std::uint8_t kTypeAcceleration = 0x51;
constexpr std::uint8_t kTypeAngularVelocity = 0x52;
constexpr std::size_t kFrameSize = 11;
constexpr std::int64_t kMillisPerDay = 24LL * 60LL * 60LL * 1000LL;

struct Frame {
  std::uint8_t type = 0;
  std::array<std::uint8_t, 8> data{};
};

class StreamParser {
 public:
  using Callback = std::function<void(const Frame&)>;

  void append(const std::uint8_t* data, std::size_t size,
              const Callback& callback) {
    bytes_received_ += size;
    buffer_.insert(buffer_.end(), data, data + size);

    while (buffer_.size() >= kFrameSize) {
      if (buffer_.front() != kFrameHeader) {
        buffer_.pop_front();
        ++discarded_bytes_;
        continue;
      }

      std::uint8_t checksum = 0;
      for (std::size_t i = 0; i < kFrameSize - 1; ++i) {
        checksum = static_cast<std::uint8_t>(checksum + buffer_[i]);
      }

      if (checksum != buffer_[kFrameSize - 1]) {
        buffer_.pop_front();
        ++discarded_bytes_;
        ++checksum_errors_;
        continue;
      }

      Frame frame;
      frame.type = buffer_[1];
      for (std::size_t i = 0; i < frame.data.size(); ++i) {
        frame.data[i] = buffer_[i + 2];
      }
      for (std::size_t i = 0; i < kFrameSize; ++i) {
        buffer_.pop_front();
      }

      ++valid_frames_;
      callback(frame);
    }
  }

  void resetBuffer() { buffer_.clear(); }

  std::uint64_t bytesReceived() const { return bytes_received_; }
  std::uint64_t validFrames() const { return valid_frames_; }
  std::uint64_t checksumErrors() const { return checksum_errors_; }
  std::uint64_t discardedBytes() const { return discarded_bytes_; }

 private:
  std::deque<std::uint8_t> buffer_;
  std::uint64_t bytes_received_ = 0;
  std::uint64_t valid_frames_ = 0;
  std::uint64_t checksum_errors_ = 0;
  std::uint64_t discarded_bytes_ = 0;
};

inline std::int16_t readInt16(const std::array<std::uint8_t, 8>& data,
                             std::size_t offset) {
  const std::uint16_t value =
      static_cast<std::uint16_t>(data[offset]) |
      (static_cast<std::uint16_t>(data[offset + 1]) << 8U);
  return static_cast<std::int16_t>(value);
}

struct DeviceTime {
  std::uint8_t year = 0;
  std::uint8_t month = 0;
  std::uint8_t day = 0;
  std::uint8_t hour = 0;
  std::uint8_t minute = 0;
  std::uint8_t second = 0;
  std::uint16_t millisecond = 0;
  std::uint32_t millis_of_day = 0;
};

inline bool decodeDeviceTime(const Frame& frame, DeviceTime* output) {
  if (frame.type != kTypeTime || output == nullptr) {
    return false;
  }

  DeviceTime decoded;
  decoded.year = frame.data[0];
  decoded.month = frame.data[1];
  decoded.day = frame.data[2];
  decoded.hour = frame.data[3];
  decoded.minute = frame.data[4];
  decoded.second = frame.data[5];
  decoded.millisecond =
      static_cast<std::uint16_t>(frame.data[6]) |
      (static_cast<std::uint16_t>(frame.data[7]) << 8U);

  // Some modules ship with an unset calendar. Only time-of-day is required
  // for relative sample timing, so month/day are intentionally not rejected.
  if (decoded.hour > 23 || decoded.minute > 59 || decoded.second > 59 ||
      decoded.millisecond > 999) {
    return false;
  }

  decoded.millis_of_day =
      (((static_cast<std::uint32_t>(decoded.hour) * 60U) + decoded.minute) *
           60U +
       decoded.second) *
          1000U +
      decoded.millisecond;
  *output = decoded;
  return true;
}

inline bool decodeAcceleration(const Frame& frame,
                               std::array<double, 3>* output) {
  if (frame.type != kTypeAcceleration || output == nullptr) {
    return false;
  }

  constexpr double kScale = 16.0 * 9.80665 / 32768.0;
  for (std::size_t axis = 0; axis < output->size(); ++axis) {
    (*output)[axis] = readInt16(frame.data, axis * 2U) * kScale;
  }
  return true;
}

inline bool decodeAngularVelocity(const Frame& frame,
                                  std::array<double, 3>* output) {
  if (frame.type != kTypeAngularVelocity || output == nullptr) {
    return false;
  }

  constexpr double kPi = 3.14159265358979323846;
  constexpr double kScale = 2000.0 * kPi / 180.0 / 32768.0;
  for (std::size_t axis = 0; axis < output->size(); ++axis) {
    (*output)[axis] = readInt16(frame.data, axis * 2U) * kScale;
  }
  return true;
}

struct ImuCycle {
  bool has_device_time = false;
  DeviceTime device_time;
  std::array<double, 3> acceleration{};
  std::array<double, 3> angular_velocity{};
};

class CycleAssembler {
 public:
  enum class Mode { kRequireDeviceTime, kImplicitAccGyro };
  using Callback = std::function<void(const ImuCycle&)>;

  explicit CycleAssembler(Mode mode) : mode_(mode) {}

  void consume(const Frame& frame, const Callback& callback) {
    if (frame.type == kTypeTime) {
      beginTimedCycle(frame);
      return;
    }

    if (frame.type == kTypeAcceleration) {
      consumeAcceleration(frame, callback);
      return;
    }

    if (frame.type == kTypeAngularVelocity) {
      consumeAngularVelocity(frame, callback);
    }
  }

  void reset() {
    cycle_open_ = false;
    have_time_ = false;
    have_acceleration_ = false;
    have_angular_velocity_ = false;
    published_ = false;
  }

  std::uint64_t completeCycles() const { return complete_cycles_; }
  std::uint64_t incompleteCycles() const { return incomplete_cycles_; }
  std::uint64_t orphanAccelerationFrames() const {
    return orphan_acceleration_frames_;
  }
  std::uint64_t orphanAngularVelocityFrames() const {
    return orphan_angular_velocity_frames_;
  }
  std::uint64_t invalidTimeFrames() const { return invalid_time_frames_; }
  std::uint64_t duplicateFrames() const { return duplicate_frames_; }

 private:
  void beginTimedCycle(const Frame& frame) {
    if (cycle_open_ && !published_ &&
        (have_acceleration_ || have_angular_velocity_)) {
      ++incomplete_cycles_;
    }

    reset();
    DeviceTime decoded;
    if (!decodeDeviceTime(frame, &decoded)) {
      ++invalid_time_frames_;
      return;
    }

    cycle_open_ = true;
    have_time_ = true;
    cycle_.has_device_time = true;
    cycle_.device_time = decoded;
  }

  void consumeAcceleration(const Frame& frame, const Callback& callback) {
    if (mode_ == Mode::kRequireDeviceTime && !have_time_) {
      ++orphan_acceleration_frames_;
      return;
    }

    if (mode_ == Mode::kImplicitAccGyro) {
      if (cycle_open_ && !published_ && have_acceleration_) {
        ++incomplete_cycles_;
      }
      reset();
      cycle_open_ = true;
      cycle_.has_device_time = false;
    } else if (published_) {
      ++duplicate_frames_;
      return;
    } else if (have_acceleration_) {
      // A second Acc without a new Time frame means the cycle boundary was
      // lost. Invalidate the old cycle so a later Gyro cannot be paired with
      // stale acceleration and time.
      ++duplicate_frames_;
      ++incomplete_cycles_;
      ++orphan_acceleration_frames_;
      reset();
      return;
    }

    if (!decodeAcceleration(frame, &cycle_.acceleration)) {
      return;
    }
    have_acceleration_ = true;
    publishIfComplete(callback);
  }

  void consumeAngularVelocity(const Frame& frame, const Callback& callback) {
    if (!cycle_open_ ||
        (mode_ == Mode::kRequireDeviceTime && !have_time_)) {
      ++orphan_angular_velocity_frames_;
      return;
    }
    if (!have_acceleration_) {
      ++orphan_angular_velocity_frames_;
      if (mode_ == Mode::kRequireDeviceTime) {
        // The configured frame order is Time -> Acc -> Gyro. Seeing Gyro
        // before Acc means this timed cycle is incomplete; keeping its Time
        // would allow a later cycle to attach to a stale timestamp.
        ++incomplete_cycles_;
        reset();
      }
      return;
    }
    if (published_ || have_angular_velocity_) {
      ++duplicate_frames_;
      return;
    }

    if (!decodeAngularVelocity(frame, &cycle_.angular_velocity)) {
      return;
    }
    have_angular_velocity_ = true;
    publishIfComplete(callback);
  }

  void publishIfComplete(const Callback& callback) {
    if (!have_acceleration_ || !have_angular_velocity_ ||
        (mode_ == Mode::kRequireDeviceTime && !have_time_)) {
      return;
    }

    published_ = true;
    ++complete_cycles_;
    callback(cycle_);

    if (mode_ == Mode::kImplicitAccGyro) {
      reset();
    }
  }

  Mode mode_;
  ImuCycle cycle_;
  bool cycle_open_ = false;
  bool have_time_ = false;
  bool have_acceleration_ = false;
  bool have_angular_velocity_ = false;
  bool published_ = false;

  std::uint64_t complete_cycles_ = 0;
  std::uint64_t incomplete_cycles_ = 0;
  std::uint64_t orphan_acceleration_frames_ = 0;
  std::uint64_t orphan_angular_velocity_frames_ = 0;
  std::uint64_t invalid_time_frames_ = 0;
  std::uint64_t duplicate_frames_ = 0;
};

class DeviceTimeUnwrapper {
 public:
  enum class Status { kInitialized, kOk, kDuplicate, kBackward };

  Status update(std::uint32_t millis_of_day, std::int64_t* unwrapped_millis,
                std::int64_t* delta_millis) {
    if (!initialized_) {
      initialized_ = true;
      last_raw_millis_ = millis_of_day;
      unwrapped_millis_ = millis_of_day;
      if (unwrapped_millis != nullptr) {
        *unwrapped_millis = unwrapped_millis_;
      }
      if (delta_millis != nullptr) {
        *delta_millis = 0;
      }
      return Status::kInitialized;
    }

    std::int64_t delta = static_cast<std::int64_t>(millis_of_day) -
                         static_cast<std::int64_t>(last_raw_millis_);
    if (delta < -(kMillisPerDay / 2)) {
      delta += kMillisPerDay;
    }

    if (delta == 0) {
      return Status::kDuplicate;
    }
    if (delta < 0 || delta > (kMillisPerDay / 2)) {
      return Status::kBackward;
    }

    last_raw_millis_ = millis_of_day;
    unwrapped_millis_ += delta;
    if (unwrapped_millis != nullptr) {
      *unwrapped_millis = unwrapped_millis_;
    }
    if (delta_millis != nullptr) {
      *delta_millis = delta;
    }
    return Status::kOk;
  }

  void reset() {
    initialized_ = false;
    last_raw_millis_ = 0;
    unwrapped_millis_ = 0;
  }

 private:
  bool initialized_ = false;
  std::uint32_t last_raw_millis_ = 0;
  std::int64_t unwrapped_millis_ = 0;
};

}  // namespace imu_ros_driver
