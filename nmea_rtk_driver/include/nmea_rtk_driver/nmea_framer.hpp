#pragma once

#include <cstddef>
#include <string>

namespace nmea_rtk_driver {
namespace protocol {

enum class FrameEvent {
  kNone,
  kStarted,
  kCompleted,
  kDroppedMalformed,
  kDroppedOversize,
};

class NmeaFramer {
 public:
  explicit NmeaFramer(std::size_t maximum_length = 1024U)
      : maximum_length_(maximum_length < 16U ? 16U : maximum_length) {}

  FrameEvent push(char byte, std::string* completed_sentence) {
    if (completed_sentence != nullptr) {
      completed_sentence->clear();
    }

    if (byte == '$') {
      buffer_.assign(1U, '$');
      collecting_ = true;
      return FrameEvent::kStarted;
    }
    if (!collecting_) {
      return FrameEvent::kNone;
    }
    if (byte == '\n') {
      if (completed_sentence != nullptr) {
        *completed_sentence = buffer_;
      }
      buffer_.clear();
      collecting_ = false;
      return FrameEvent::kCompleted;
    }
    if (byte == '\r') {
      return FrameEvent::kNone;
    }

    const unsigned char value = static_cast<unsigned char>(byte);
    if (value < 0x20U || value > 0x7eU) {
      buffer_.clear();
      collecting_ = false;
      return FrameEvent::kDroppedMalformed;
    }
    buffer_.push_back(byte);
    if (buffer_.size() > maximum_length_) {
      buffer_.clear();
      collecting_ = false;
      return FrameEvent::kDroppedOversize;
    }
    return FrameEvent::kNone;
  }

  void reset() {
    buffer_.clear();
    collecting_ = false;
  }

  bool collecting() const { return collecting_; }
  std::size_t bufferedSize() const { return buffer_.size(); }

 private:
  std::size_t maximum_length_;
  bool collecting_ = false;
  std::string buffer_;
};

}  // namespace protocol
}  // namespace nmea_rtk_driver
