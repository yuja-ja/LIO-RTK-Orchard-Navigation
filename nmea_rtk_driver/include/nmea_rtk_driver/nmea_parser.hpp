#pragma once

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace nmea_rtk_driver {
namespace protocol {

enum class SentenceKind {
  kUnknown,
  kGga,
  kGns,
  kGst,
  kRmc,
  kVtg,
  kThs,
  kGsa,
  kZda,
};

const char* sentenceKindName(SentenceKind kind);

struct Gga {
  bool has_utc = false;
  std::string utc_text;
  double utc_seconds_of_day = 0.0;
  bool has_position = false;
  double latitude_deg = 0.0;
  double longitude_deg = 0.0;
  int quality = 0;
  int satellites_used = 0;
  bool has_hdop = false;
  double hdop = 0.0;
  bool has_msl_height = false;
  double msl_height_m = 0.0;
  bool has_geoid_separation = false;
  double geoid_separation_m = 0.0;
  bool has_differential_age = false;
  double differential_age_sec = 0.0;
  std::string differential_station_id;
};

struct Gns {
  bool has_utc = false;
  std::string utc_text;
  double utc_seconds_of_day = 0.0;
  bool has_position = false;
  double latitude_deg = 0.0;
  double longitude_deg = 0.0;
  std::string mode;
  int satellites_used = 0;
  bool has_hdop = false;
  double hdop = 0.0;
  bool has_msl_height = false;
  double msl_height_m = 0.0;
  bool has_geoid_separation = false;
  double geoid_separation_m = 0.0;
  bool has_differential_age = false;
  double differential_age_sec = 0.0;
  std::string differential_station_id;
  std::string navigation_status;
};

struct Gst {
  bool has_utc = false;
  std::string utc_text;
  double utc_seconds_of_day = 0.0;
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

struct Rmc {
  bool has_utc = false;
  std::string utc_text;
  double utc_seconds_of_day = 0.0;
  bool status_active = false;
  bool has_position = false;
  double latitude_deg = 0.0;
  double longitude_deg = 0.0;
  bool has_speed = false;
  double speed_mps = 0.0;
  bool has_track = false;
  double track_true_deg = 0.0;
  std::string date_ddmmyy;
  std::string mode;
  std::string navigation_status;
  bool valid = false;
};

struct Vtg {
  bool has_track = false;
  double track_true_deg = 0.0;
  bool has_speed = false;
  double speed_mps = 0.0;
  std::string mode;
  bool valid = false;
};

struct Ths {
  bool has_heading = false;
  double heading_true_deg = 0.0;
  std::string mode;
  bool valid = false;
};

struct Gsa {
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

struct Zda {
  bool has_utc = false;
  std::string utc_text;
  double utc_seconds_of_day = 0.0;
  int day = 0;
  int month = 0;
  int year = 0;
  bool has_local_zone = false;
  int local_zone_hours = 0;
  int local_zone_minutes = 0;
};

struct ParseResult {
  bool syntax_ok = false;
  bool checksum_present = false;
  bool checksum_valid = false;
  bool recognized = false;
  SentenceKind kind = SentenceKind::kUnknown;
  std::string error;
  std::string talker;
  std::string formatter;
  bool secondary_antenna = false;
  Gga gga;
  Gns gns;
  Gst gst;
  Rmc rmc;
  Vtg vtg;
  Ths ths;
  Gsa gsa;
  Zda zda;
};

std::uint8_t nmeaChecksum(const std::string& payload);
ParseResult parseLine(const std::string& line);

bool gnsModeHasFix(const std::string& mode);
int qualityFromGnsMode(const std::string& mode);
std::string fixQualityName(int quality);
bool utcUnixSeconds(int year,
                    int month,
                    int day,
                    double utc_seconds_of_day,
                    double* unix_seconds);

}  // namespace protocol
}  // namespace nmea_rtk_driver
