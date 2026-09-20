#pragma once

#include "nmea_rtk_driver/nmea_parser.hpp"

#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>
#include <vector>

namespace nmea_rtk_driver {
namespace protocol {

enum class PositionPolicy {
  kAuto,
  kGgaOnly,
  kGnsOnly,
};

enum class VelocityPolicy {
  kAuto,
  kRmcOnly,
  kVtgOnly,
};

struct AssemblerConfig {
  PositionPolicy position_policy = PositionPolicy::kAuto;
  VelocityPolicy velocity_policy = VelocityPolicy::kRmcOnly;
  SentenceKind preferred_position = SentenceKind::kGga;
  SentenceKind preferred_velocity = SentenceKind::kRmc;
  double utc_match_tolerance_sec = 0.05;
  double assembly_delay_sec = 0.08;
  double stale_timeout_sec = 0.50;
  double duplicate_retention_sec = 2.0;
  double vtg_match_window_sec = 0.15;
  double gsa_max_age_sec = 2.0;
  bool require_covariance = false;
  bool require_velocity = false;
  std::size_t maximum_pending_epochs = 32U;
};

struct PositionObservation {
  bool present = false;
  bool valid = false;
  bool has_coordinates = false;
  SentenceKind source = SentenceKind::kUnknown;
  std::string talker;
  std::string formatter;
  std::string utc_text;
  double utc_seconds_of_day = 0.0;
  double arrival_stamp_sec = 0.0;
  double latitude_deg = 0.0;
  double longitude_deg = 0.0;
  int fix_quality = 0;
  std::string fix_mode;
  int satellites_used = 0;
  bool has_hdop = false;
  double hdop = 0.0;
  bool has_msl_height = false;
  double msl_height_m = 0.0;
  bool has_geoid_separation = false;
  double geoid_separation_m = 0.0;
  bool has_ellipsoid_height = false;
  double ellipsoid_height_m = 0.0;
  bool has_differential_age = false;
  double differential_age_sec = 0.0;
  std::string differential_station_id;
  std::string navigation_status;
};

struct CovarianceObservation {
  bool present = false;
  std::string talker;
  std::string formatter;
  std::string utc_text;
  double utc_seconds_of_day = 0.0;
  double arrival_stamp_sec = 0.0;
  bool has_rms = false;
  double rms_m = 0.0;
  bool has_error_ellipse = false;
  double semi_major_std_m = 0.0;
  double semi_minor_std_m = 0.0;
  double orientation_deg = 0.0;
  bool has_position_std = false;
  double latitude_std_m = 0.0;
  double longitude_std_m = 0.0;
  double altitude_std_m = 0.0;
};

struct VelocityObservation {
  bool present = false;
  bool valid = false;
  SentenceKind source = SentenceKind::kUnknown;
  std::string talker;
  std::string formatter;
  std::string utc_text;
  bool has_utc = false;
  double utc_seconds_of_day = 0.0;
  double arrival_stamp_sec = 0.0;
  double speed_mps = 0.0;
  double track_true_deg = 0.0;
  std::string mode;
  std::string navigation_status;
  std::string date_ddmmyy;
};

struct GsaObservation {
  bool present = false;
  double arrival_stamp_sec = 0.0;
  double received_wall_sec = 0.0;
  std::string talker;
  std::string selection_mode;
  int fix_dimension = 1;
  std::vector<std::string> satellite_ids;
  bool has_pdop = false;
  double pdop = 0.0;
  bool has_hdop = false;
  double hdop = 0.0;
  bool has_vdop = false;
  double vdop = 0.0;
  std::string system_id;
};

struct EpochObservation {
  double reference_arrival_stamp_sec = 0.0;
  double utc_seconds_of_day = 0.0;
  std::string utc_text;
  std::string utc_date_ddmmyy;
  PositionObservation position;
  CovarianceObservation covariance;
  VelocityObservation velocity;
  GsaObservation gsa;
};

class EpochAssembler {
 public:
  explicit EpochAssembler(const AssemblerConfig& config = AssemblerConfig());

  void ingest(const ParseResult& parsed,
              double arrival_stamp_sec,
              double received_wall_sec);
  std::vector<EpochObservation> flush(double current_wall_sec);
  void reset();

  std::size_t pendingCount() const { return pending_.size(); }
  std::uint64_t duplicateCount() const { return duplicate_count_; }
  std::uint64_t staleDropCount() const { return stale_drop_count_; }

 private:
  struct PendingEpoch {
    double utc_seconds_of_day = 0.0;
    std::string utc_text;
    double first_seen_wall_sec = 0.0;
    double last_seen_wall_sec = 0.0;
    PositionObservation position;
    CovarianceObservation covariance;
    VelocityObservation velocity;
    GsaObservation gsa;
    std::string utc_date_ddmmyy;
  };

  struct EmittedEpoch {
    double utc_seconds_of_day = 0.0;
    double emitted_wall_sec = 0.0;
  };

  struct OrphanVtg {
    bool present = false;
    VelocityObservation velocity;
    double received_wall_sec = 0.0;
  };

  PendingEpoch* findOrCreate(double utc_seconds_of_day,
                             const std::string& utc_text,
                             double received_wall_sec);
  bool wasRecentlyEmitted(double utc_seconds_of_day,
                          double received_wall_sec);
  void selectPosition(const PositionObservation& incoming,
                      PendingEpoch* pending);
  void selectVelocity(const VelocityObservation& incoming,
                      PendingEpoch* pending);
  void attachLatestGsa(double received_wall_sec, PendingEpoch* pending) const;
  void pruneEmitted(double current_wall_sec);

  AssemblerConfig config_;
  std::deque<PendingEpoch> pending_;
  std::deque<EmittedEpoch> emitted_;
  GsaObservation latest_gsa_;
  OrphanVtg orphan_vtg_;
  std::uint64_t duplicate_count_ = 0U;
  std::uint64_t stale_drop_count_ = 0U;
};

}  // namespace protocol
}  // namespace nmea_rtk_driver
