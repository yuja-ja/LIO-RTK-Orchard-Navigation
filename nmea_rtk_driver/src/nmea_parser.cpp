#include "nmea_rtk_driver/nmea_parser.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <sstream>

namespace nmea_rtk_driver {
namespace protocol {
namespace {

std::string trim(const std::string& input) {
  const std::string whitespace = " \t\r\n";
  const std::size_t first = input.find_first_not_of(whitespace);
  if (first == std::string::npos) {
    return std::string();
  }
  const std::size_t last = input.find_last_not_of(whitespace);
  return input.substr(first, last - first + 1U);
}

std::string upper(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    if (c >= 'a' && c <= 'z') {
      return static_cast<char>(c - 'a' + 'A');
    }
    return static_cast<char>(c);
  });
  return value;
}

std::vector<std::string> split(const std::string& input, char delimiter) {
  std::vector<std::string> fields;
  std::size_t begin = 0U;
  while (true) {
    const std::size_t end = input.find(delimiter, begin);
    if (end == std::string::npos) {
      fields.push_back(input.substr(begin));
      break;
    }
    fields.push_back(input.substr(begin, end - begin));
    begin = end + 1U;
  }
  return fields;
}

bool parseDouble(const std::string& text, double* value) {
  if (text.empty() || value == nullptr) {
    return false;
  }
  errno = 0;
  char* end = nullptr;
  const double parsed = std::strtod(text.c_str(), &end);
  if (errno == ERANGE || end == text.c_str() || *end != '\0' ||
      !std::isfinite(parsed)) {
    return false;
  }
  *value = parsed;
  return true;
}

bool parseInt(const std::string& text, int* value) {
  if (text.empty() || value == nullptr) {
    return false;
  }
  errno = 0;
  char* end = nullptr;
  const long parsed = std::strtol(text.c_str(), &end, 10);
  if (errno == ERANGE || end == text.c_str() || *end != '\0' ||
      parsed < std::numeric_limits<int>::min() ||
      parsed > std::numeric_limits<int>::max()) {
    return false;
  }
  *value = static_cast<int>(parsed);
  return true;
}

bool parseOptionalDouble(const std::string& text,
                         bool nonnegative,
                         bool* present,
                         double* value) {
  if (present == nullptr || value == nullptr) {
    return false;
  }
  *present = false;
  if (text.empty()) {
    return true;
  }
  double parsed = 0.0;
  if (!parseDouble(text, &parsed) || (nonnegative && parsed < 0.0)) {
    return false;
  }
  *present = true;
  *value = parsed;
  return true;
}

bool parseUtc(const std::string& text, double* seconds_of_day) {
  if (seconds_of_day == nullptr || text.size() < 6U) {
    return false;
  }
  int hour = 0;
  int minute = 0;
  double second = 0.0;
  if (!parseInt(text.substr(0U, 2U), &hour) ||
      !parseInt(text.substr(2U, 2U), &minute) ||
      !parseDouble(text.substr(4U), &second)) {
    return false;
  }
  if (hour < 0 || hour > 23 || minute < 0 || minute > 59 ||
      second < 0.0 || second >= 61.0) {
    return false;
  }
  *seconds_of_day = static_cast<double>(hour * 3600 + minute * 60) + second;
  return true;
}

bool parseCoordinate(const std::string& text,
                     const std::string& hemisphere,
                     bool latitude,
                     double* degrees) {
  if (degrees == nullptr || text.empty() || hemisphere.size() != 1U) {
    return false;
  }
  const char hemi = upper(hemisphere)[0];
  if (latitude) {
    if (hemi != 'N' && hemi != 'S') {
      return false;
    }
  } else if (hemi != 'E' && hemi != 'W') {
    return false;
  }

  double nmea_value = 0.0;
  if (!parseDouble(text, &nmea_value) || nmea_value < 0.0) {
    return false;
  }
  const double whole_degrees = std::floor(nmea_value / 100.0);
  const double minutes = nmea_value - whole_degrees * 100.0;
  const double maximum = latitude ? 90.0 : 180.0;
  if (minutes < 0.0 || minutes >= 60.0 || whole_degrees > maximum ||
      (whole_degrees == maximum && minutes > 0.0)) {
    return false;
  }
  double output = whole_degrees + minutes / 60.0;
  if (hemi == 'S' || hemi == 'W') {
    output = -output;
  }
  *degrees = output;
  return true;
}

bool parsePosition(const std::string& latitude,
                   const std::string& north_south,
                   const std::string& longitude,
                   const std::string& east_west,
                   bool* present,
                   double* latitude_deg,
                   double* longitude_deg) {
  if (present == nullptr || latitude_deg == nullptr || longitude_deg == nullptr) {
    return false;
  }
  *present = false;
  const bool all_empty = latitude.empty() && north_south.empty() &&
                         longitude.empty() && east_west.empty();
  if (all_empty) {
    return true;
  }
  if (latitude.empty() || north_south.empty() || longitude.empty() ||
      east_west.empty()) {
    return false;
  }
  if (!parseCoordinate(latitude, north_south, true, latitude_deg) ||
      !parseCoordinate(longitude, east_west, false, longitude_deg)) {
    return false;
  }
  *present = true;
  return true;
}

bool validDate(const std::string& text) {
  if (text.empty()) {
    return true;
  }
  if (text.size() != 6U ||
      !std::all_of(text.begin(), text.end(), [](unsigned char c) {
        return c >= '0' && c <= '9';
      })) {
    return false;
  }
  int day = 0;
  int month = 0;
  int year = 0;
  return parseInt(text.substr(0U, 2U), &day) &&
         parseInt(text.substr(2U, 2U), &month) &&
         parseInt(text.substr(4U, 2U), &year) && day >= 1 && day <= 31 &&
         month >= 1 && month <= 12 && year >= 0 && year <= 99;
}

bool validCalendarDate(int year, int month, int day) {
  if (year < 1970 || year > 2400 || month < 1 || month > 12 || day < 1) {
    return false;
  }
  static const int kDaysPerMonth[] = {
      31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  int maximum_day = kDaysPerMonth[month - 1];
  const bool leap = (year % 4 == 0 && year % 100 != 0) || year % 400 == 0;
  if (month == 2 && leap) {
    maximum_day = 29;
  }
  return day <= maximum_day;
}

bool validTrack(double track_deg) {
  return track_deg >= 0.0 && track_deg < 360.0;
}

bool usableNavigationMode(const std::string& mode) {
  if (mode.empty()) {
    return true;
  }
  const char value = upper(mode)[0];
  return value == 'A' || value == 'D' || value == 'F' || value == 'P' ||
         value == 'R';
}

bool safeNavigationStatus(const std::string& status) {
  if (status.empty()) {
    return true;
  }
  return upper(status)[0] == 'S';
}

bool parseGga(const std::vector<std::string>& fields, ParseResult* result) {
  if (fields.size() < 15U) {
    result->error = "GGA has fewer than 14 data fields";
    return false;
  }
  Gga& output = result->gga;
  output.utc_text = fields[1];
  output.has_utc = parseUtc(fields[1], &output.utc_seconds_of_day);
  if (!output.has_utc) {
    result->error = "invalid GGA UTC field";
    return false;
  }
  if (!parsePosition(fields[2], fields[3], fields[4], fields[5],
                     &output.has_position, &output.latitude_deg,
                     &output.longitude_deg)) {
    result->error = "invalid GGA latitude or longitude";
    return false;
  }
  if (!parseInt(fields[6], &output.quality) || output.quality < 0 ||
      output.quality > 9) {
    result->error = "invalid GGA quality";
    return false;
  }
  if (!fields[7].empty() &&
      (!parseInt(fields[7], &output.satellites_used) ||
       output.satellites_used < 0 || output.satellites_used > 255)) {
    result->error = "invalid GGA satellite count";
    return false;
  }
  if (!parseOptionalDouble(fields[8], true, &output.has_hdop, &output.hdop) ||
      !parseOptionalDouble(fields[9], false, &output.has_msl_height,
                           &output.msl_height_m) ||
      !parseOptionalDouble(fields[11], false, &output.has_geoid_separation,
                           &output.geoid_separation_m) ||
      !parseOptionalDouble(fields[13], true, &output.has_differential_age,
                           &output.differential_age_sec)) {
    result->error = "invalid GGA numeric field";
    return false;
  }
  output.differential_station_id = fields[14];
  if (output.quality > 0 && !output.has_position) {
    result->error = "GGA reports a fix without coordinates";
    return false;
  }
  return true;
}

bool parseGns(const std::vector<std::string>& fields, ParseResult* result) {
  if (fields.size() < 13U) {
    result->error = "GNS has fewer than 12 data fields";
    return false;
  }
  Gns& output = result->gns;
  output.utc_text = fields[1];
  output.has_utc = parseUtc(fields[1], &output.utc_seconds_of_day);
  if (!output.has_utc) {
    result->error = "invalid GNS UTC field";
    return false;
  }
  if (!parsePosition(fields[2], fields[3], fields[4], fields[5],
                     &output.has_position, &output.latitude_deg,
                     &output.longitude_deg)) {
    result->error = "invalid GNS latitude or longitude";
    return false;
  }
  output.mode = upper(fields[6]);
  if (output.mode.empty()) {
    result->error = "empty GNS mode";
    return false;
  }
  for (char mode : output.mode) {
    const std::string valid_modes = "NADPRFEMS";
    if (valid_modes.find(mode) == std::string::npos) {
      result->error = "invalid GNS mode";
      return false;
    }
  }
  if (!fields[7].empty() &&
      (!parseInt(fields[7], &output.satellites_used) ||
       output.satellites_used < 0 || output.satellites_used > 255)) {
    result->error = "invalid GNS satellite count";
    return false;
  }
  if (!parseOptionalDouble(fields[8], true, &output.has_hdop, &output.hdop) ||
      !parseOptionalDouble(fields[9], false, &output.has_msl_height,
                           &output.msl_height_m) ||
      !parseOptionalDouble(fields[10], false, &output.has_geoid_separation,
                           &output.geoid_separation_m) ||
      !parseOptionalDouble(fields[11], true, &output.has_differential_age,
                           &output.differential_age_sec)) {
    result->error = "invalid GNS numeric field";
    return false;
  }
  output.differential_station_id = fields[12];
  if (fields.size() > 13U) {
    output.navigation_status = upper(fields[13]);
    if (!output.navigation_status.empty() &&
        std::string("SCUV").find(output.navigation_status[0]) ==
            std::string::npos) {
      result->error = "invalid GNS navigation status";
      return false;
    }
  }
  if (gnsModeHasFix(output.mode) && !output.has_position) {
    result->error = "GNS reports a fix without coordinates";
    return false;
  }
  return true;
}

bool parseGst(const std::vector<std::string>& fields, ParseResult* result) {
  if (fields.size() < 9U) {
    result->error = "GST has fewer than 8 data fields";
    return false;
  }
  Gst& output = result->gst;
  output.utc_text = fields[1];
  output.has_utc = parseUtc(fields[1], &output.utc_seconds_of_day);
  if (!output.has_utc) {
    result->error = "invalid GST UTC field";
    return false;
  }
  if (!parseOptionalDouble(fields[2], true, &output.has_rms, &output.rms_m)) {
    result->error = "invalid GST RMS";
    return false;
  }

  const bool any_ellipse = !fields[3].empty() || !fields[4].empty() ||
                           !fields[5].empty();
  if (any_ellipse) {
    if (!parseDouble(fields[3], &output.semi_major_std_m) ||
        !parseDouble(fields[4], &output.semi_minor_std_m) ||
        !parseDouble(fields[5], &output.orientation_deg) ||
        output.semi_major_std_m < 0.0 || output.semi_minor_std_m < 0.0 ||
        !validTrack(output.orientation_deg)) {
      result->error = "invalid GST error ellipse";
      return false;
    }
    output.has_error_ellipse = true;
  }

  const bool any_position_std = !fields[6].empty() || !fields[7].empty() ||
                                !fields[8].empty();
  if (any_position_std) {
    if (!parseDouble(fields[6], &output.latitude_std_m) ||
        !parseDouble(fields[7], &output.longitude_std_m) ||
        !parseDouble(fields[8], &output.altitude_std_m) ||
        output.latitude_std_m < 0.0 || output.longitude_std_m < 0.0 ||
        output.altitude_std_m < 0.0) {
      result->error = "invalid GST position standard deviation";
      return false;
    }
    output.has_position_std = true;
  }
  return true;
}

bool parseRmc(const std::vector<std::string>& fields, ParseResult* result) {
  if (fields.size() < 10U) {
    result->error = "RMC has fewer than 9 data fields";
    return false;
  }
  Rmc& output = result->rmc;
  output.utc_text = fields[1];
  output.has_utc = parseUtc(fields[1], &output.utc_seconds_of_day);
  if (!output.has_utc) {
    result->error = "invalid RMC UTC field";
    return false;
  }
  const std::string status = upper(fields[2]);
  if (status != "A" && status != "V") {
    result->error = "invalid RMC status";
    return false;
  }
  output.status_active = status == "A";
  if (!parsePosition(fields[3], fields[4], fields[5], fields[6],
                     &output.has_position, &output.latitude_deg,
                     &output.longitude_deg)) {
    result->error = "invalid RMC latitude or longitude";
    return false;
  }
  if (!parseOptionalDouble(fields[7], true, &output.has_speed,
                           &output.speed_mps)) {
    result->error = "invalid RMC speed";
    return false;
  }
  if (output.has_speed) {
    output.speed_mps *= 0.5144444444444445;
  }
  if (!parseOptionalDouble(fields[8], true, &output.has_track,
                           &output.track_true_deg) ||
      (output.has_track && !validTrack(output.track_true_deg))) {
    result->error = "invalid RMC true track";
    return false;
  }
  output.date_ddmmyy = fields[9];
  if (!validDate(output.date_ddmmyy)) {
    result->error = "invalid RMC date";
    return false;
  }
  if (fields.size() > 12U) {
    output.mode = upper(fields[12]);
  }
  if (fields.size() > 13U) {
    output.navigation_status = upper(fields[13]);
  }
  output.valid = output.status_active && output.has_speed && output.has_track &&
                 usableNavigationMode(output.mode) &&
                 safeNavigationStatus(output.navigation_status);
  return true;
}

bool parseVtg(const std::vector<std::string>& fields, ParseResult* result) {
  if (fields.size() < 9U) {
    result->error = "VTG has fewer than 8 data fields";
    return false;
  }
  Vtg& output = result->vtg;
  if (!fields[2].empty() && upper(fields[2]) != "T") {
    result->error = "VTG true-track unit is not T";
    return false;
  }
  if (!fields[6].empty() && upper(fields[6]) != "N") {
    result->error = "VTG knot-speed unit is not N";
    return false;
  }
  if (!fields[8].empty() && upper(fields[8]) != "K") {
    result->error = "VTG metric-speed unit is not K";
    return false;
  }
  if (!parseOptionalDouble(fields[1], true, &output.has_track,
                           &output.track_true_deg) ||
      (output.has_track && !validTrack(output.track_true_deg))) {
    result->error = "invalid VTG true track";
    return false;
  }
  bool has_kmh = false;
  double kmh = 0.0;
  bool has_knots = false;
  double knots = 0.0;
  if (!parseOptionalDouble(fields[7], true, &has_kmh, &kmh) ||
      !parseOptionalDouble(fields[5], true, &has_knots, &knots)) {
    result->error = "invalid VTG speed";
    return false;
  }
  if (has_kmh) {
    output.has_speed = true;
    output.speed_mps = kmh / 3.6;
  } else if (has_knots) {
    output.has_speed = true;
    output.speed_mps = knots * 0.5144444444444445;
  }
  if (fields.size() > 9U) {
    output.mode = upper(fields[9]);
  }
  output.valid = output.has_track && output.has_speed &&
                 usableNavigationMode(output.mode);
  return true;
}

bool parseThs(const std::vector<std::string>& fields, ParseResult* result) {
  if (fields.size() < 3U) {
    result->error = "THS has fewer than 2 data fields";
    return false;
  }
  Ths& output = result->ths;
  if (!fields[1].empty()) {
    if (!parseDouble(fields[1], &output.heading_true_deg) ||
        !validTrack(output.heading_true_deg)) {
      result->error = "invalid THS heading";
      return false;
    }
    output.has_heading = true;
  }
  output.mode = upper(fields[2]);
  if (output.mode.size() != 1U ||
      std::string("AEMSV").find(output.mode[0]) == std::string::npos) {
    result->error = "invalid THS mode";
    return false;
  }
  output.valid = output.has_heading && output.mode == "A";
  return true;
}

bool parseGsa(const std::vector<std::string>& fields, ParseResult* result) {
  if (fields.size() < 18U) {
    result->error = "GSA has fewer than 17 data fields";
    return false;
  }
  Gsa& output = result->gsa;
  output.selection_mode = upper(fields[1]);
  if (output.selection_mode != "A" && output.selection_mode != "M") {
    result->error = "invalid GSA selection mode";
    return false;
  }
  if (!parseInt(fields[2], &output.fix_dimension) || output.fix_dimension < 1 ||
      output.fix_dimension > 3) {
    result->error = "invalid GSA fix dimension";
    return false;
  }
  for (std::size_t index = 3U; index <= 14U; ++index) {
    if (!fields[index].empty()) {
      output.satellite_ids.push_back(fields[index]);
    }
  }
  if (!parseOptionalDouble(fields[15], true, &output.has_pdop, &output.pdop) ||
      !parseOptionalDouble(fields[16], true, &output.has_hdop, &output.hdop) ||
      !parseOptionalDouble(fields[17], true, &output.has_vdop, &output.vdop)) {
    result->error = "invalid GSA DOP";
    return false;
  }
  if (fields.size() > 18U) {
    output.system_id = fields[18];
  }
  return true;
}

bool parseZda(const std::vector<std::string>& fields, ParseResult* result) {
  if (fields.size() < 5U) {
    result->error = "ZDA has fewer than 4 data fields";
    return false;
  }
  Zda& output = result->zda;
  output.utc_text = fields[1];
  output.has_utc = parseUtc(fields[1], &output.utc_seconds_of_day);
  if (!output.has_utc || !parseInt(fields[2], &output.day) ||
      !parseInt(fields[3], &output.month) || !parseInt(fields[4], &output.year) ||
      !validCalendarDate(output.year, output.month, output.day)) {
    result->error = "invalid ZDA UTC date or time";
    return false;
  }
  if (fields.size() > 5U && !fields[5].empty()) {
    if (!parseInt(fields[5], &output.local_zone_hours) ||
        output.local_zone_hours < -13 || output.local_zone_hours > 13) {
      result->error = "invalid ZDA local-zone hour";
      return false;
    }
    output.has_local_zone = true;
  }
  if (fields.size() > 6U && !fields[6].empty()) {
    if (!parseInt(fields[6], &output.local_zone_minutes) ||
        output.local_zone_minutes < 0 || output.local_zone_minutes > 59) {
      result->error = "invalid ZDA local-zone minute";
      return false;
    }
    output.has_local_zone = true;
  }
  return true;
}

int hexValue(char value) {
  if (value >= '0' && value <= '9') {
    return value - '0';
  }
  if (value >= 'A' && value <= 'F') {
    return value - 'A' + 10;
  }
  if (value >= 'a' && value <= 'f') {
    return value - 'a' + 10;
  }
  return -1;
}

}  // namespace

const char* sentenceKindName(SentenceKind kind) {
  switch (kind) {
    case SentenceKind::kGga:
      return "GGA";
    case SentenceKind::kGns:
      return "GNS";
    case SentenceKind::kGst:
      return "GST";
    case SentenceKind::kRmc:
      return "RMC";
    case SentenceKind::kVtg:
      return "VTG";
    case SentenceKind::kThs:
      return "THS";
    case SentenceKind::kGsa:
      return "GSA";
    case SentenceKind::kZda:
      return "ZDA";
    case SentenceKind::kUnknown:
    default:
      return "UNKNOWN";
  }
}

std::uint8_t nmeaChecksum(const std::string& payload) {
  std::uint8_t checksum = 0U;
  for (unsigned char value : payload) {
    checksum ^= value;
  }
  return checksum;
}

ParseResult parseLine(const std::string& line) {
  ParseResult result;
  const std::string cleaned = trim(line);
  if (cleaned.empty() || cleaned[0] != '$') {
    result.error = "NMEA sentence must start with $";
    return result;
  }

  const std::size_t star = cleaned.find('*');
  std::string payload;
  if (star == std::string::npos) {
    payload = cleaned.substr(1U);
  } else {
    result.checksum_present = true;
    if (star + 3U != cleaned.size()) {
      result.error = "NMEA checksum must contain exactly two hexadecimal digits";
      return result;
    }
    const int high = hexValue(cleaned[star + 1U]);
    const int low = hexValue(cleaned[star + 2U]);
    if (high < 0 || low < 0) {
      result.error = "NMEA checksum is not hexadecimal";
      return result;
    }
    payload = cleaned.substr(1U, star - 1U);
    const std::uint8_t expected = static_cast<std::uint8_t>((high << 4) | low);
    result.checksum_valid = nmeaChecksum(payload) == expected;
  }

  const std::vector<std::string> fields = split(payload, ',');
  if (fields.empty() || fields[0].size() < 3U) {
    result.error = "invalid NMEA sentence identifier";
    return result;
  }
  const std::string identifier = upper(fields[0]);
  static const char* const kSecondaryFormatters[] = {
      "GGAH", "GNSH", "GSTH", "RMCH", "VTGH"};
  for (const char* candidate : kSecondaryFormatters) {
    const std::string suffix(candidate);
    if (identifier.size() >= suffix.size() &&
        identifier.compare(identifier.size() - suffix.size(), suffix.size(),
                           suffix) == 0) {
      result.formatter = suffix;
      result.talker = identifier.substr(0U, identifier.size() - suffix.size());
      result.secondary_antenna = true;
      break;
    }
  }
  if (result.formatter.empty()) {
    result.formatter = identifier.substr(identifier.size() - 3U);
    result.talker = identifier.substr(0U, identifier.size() - 3U);
  }

  const std::string base_formatter = result.secondary_antenna
                                         ? result.formatter.substr(0U, 3U)
                                         : result.formatter;

  bool parsed = true;
  if (base_formatter == "GGA") {
    result.kind = SentenceKind::kGga;
    parsed = parseGga(fields, &result);
  } else if (base_formatter == "GNS") {
    result.kind = SentenceKind::kGns;
    parsed = parseGns(fields, &result);
  } else if (base_formatter == "GST") {
    result.kind = SentenceKind::kGst;
    parsed = parseGst(fields, &result);
  } else if (base_formatter == "RMC") {
    result.kind = SentenceKind::kRmc;
    parsed = parseRmc(fields, &result);
  } else if (base_formatter == "VTG") {
    result.kind = SentenceKind::kVtg;
    parsed = parseVtg(fields, &result);
  } else if (result.formatter == "THS") {
    result.kind = SentenceKind::kThs;
    parsed = parseThs(fields, &result);
  } else if (result.formatter == "GSA") {
    result.kind = SentenceKind::kGsa;
    parsed = parseGsa(fields, &result);
  } else if (result.formatter == "ZDA") {
    result.kind = SentenceKind::kZda;
    parsed = parseZda(fields, &result);
  } else {
    result.kind = SentenceKind::kUnknown;
  }

  result.recognized = result.kind != SentenceKind::kUnknown;
  result.syntax_ok = parsed;
  if (parsed && result.error.empty()) {
    result.error.clear();
  }
  return result;
}

bool gnsModeHasFix(const std::string& mode) {
  const std::string normalized = upper(mode);
  return normalized.find_first_of("ADPRF") != std::string::npos;
}

int qualityFromGnsMode(const std::string& mode) {
  const std::string normalized = upper(mode);
  if (normalized.find_first_of("FR") != std::string::npos) {
    return 9;
  }
  if (normalized.find('D') != std::string::npos) {
    return 2;
  }
  if (normalized.find_first_of("AP") != std::string::npos) {
    return 1;
  }
  if (normalized.find('E') != std::string::npos) {
    return 6;
  }
  return 0;
}

std::string fixQualityName(int quality) {
  switch (quality) {
    case 0:
      return "NO_FIX";
    case 1:
      return "SINGLE";
    case 2:
      return "DIFFERENTIAL";
    case 3:
      return "PPS";
    case 4:
      return "RTK_FIXED";
    case 5:
      return "RTK_FLOAT";
    case 6:
      return "ESTIMATED";
    case 7:
      return "MANUAL";
    case 8:
      return "SIMULATION";
    case 9:
      return "RTK_UNSPECIFIED";
    default: {
      std::ostringstream stream;
      stream << "QUALITY_" << quality;
      return stream.str();
    }
  }
}

bool utcUnixSeconds(int year,
                    int month,
                    int day,
                    double utc_seconds_of_day,
                    double* unix_seconds) {
  if (unix_seconds == nullptr || !std::isfinite(utc_seconds_of_day) ||
      utc_seconds_of_day < 0.0 || utc_seconds_of_day >= 86401.0 ||
      !validCalendarDate(year, month, day)) {
    return false;
  }

  int adjusted_year = year;
  adjusted_year -= month <= 2 ? 1 : 0;
  const int era = (adjusted_year >= 0 ? adjusted_year : adjusted_year - 399) / 400;
  const unsigned int year_of_era =
      static_cast<unsigned int>(adjusted_year - era * 400);
  const unsigned int adjusted_month = static_cast<unsigned int>(
      month + (month > 2 ? -3 : 9));
  const unsigned int day_of_year =
      (153U * adjusted_month + 2U) / 5U + static_cast<unsigned int>(day) - 1U;
  const unsigned int day_of_era =
      year_of_era * 365U + year_of_era / 4U - year_of_era / 100U +
      day_of_year;
  const std::int64_t days_since_epoch =
      static_cast<std::int64_t>(era) * 146097LL +
      static_cast<std::int64_t>(day_of_era) - 719468LL;
  *unix_seconds = static_cast<double>(days_since_epoch) * 86400.0 +
                  utc_seconds_of_day;
  return true;
}

}  // namespace protocol
}  // namespace nmea_rtk_driver
