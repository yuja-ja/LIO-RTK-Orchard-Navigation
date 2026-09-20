#include "nmea_rtk_driver/epoch_assembler.hpp"
#include "nmea_rtk_driver/nmea_framer.hpp"
#include "nmea_rtk_driver/nmea_parser.hpp"

#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {

using nmea_rtk_driver::protocol::AssemblerConfig;
using nmea_rtk_driver::protocol::EpochAssembler;
using nmea_rtk_driver::protocol::FrameEvent;
using nmea_rtk_driver::protocol::NmeaFramer;
using nmea_rtk_driver::protocol::ParseResult;
using nmea_rtk_driver::protocol::SentenceKind;
using nmea_rtk_driver::protocol::nmeaChecksum;
using nmea_rtk_driver::protocol::parseLine;

int g_failures = 0;

void expectTrue(bool condition,
                const char* expression,
                const char* file,
                int line) {
  if (!condition) {
    std::cerr << file << ':' << line << ": expected " << expression << '\n';
    ++g_failures;
  }
}

void expectNear(double actual,
                double expected,
                double tolerance,
                const char* expression,
                const char* file,
                int line) {
  if (!std::isfinite(actual) || std::abs(actual - expected) > tolerance) {
    std::cerr << file << ':' << line << ": expected " << expression << " ~= "
              << std::setprecision(16) << expected << ", actual " << actual
              << ", tolerance " << tolerance << '\n';
    ++g_failures;
  }
}

#define EXPECT_TRUE(expression) \
  expectTrue((expression), #expression, __FILE__, __LINE__)
#define EXPECT_FALSE(expression) \
  expectTrue(!(expression), "!(" #expression ")", __FILE__, __LINE__)
#define EXPECT_NEAR(actual, expected, tolerance) \
  expectNear((actual), (expected), (tolerance), #actual, __FILE__, __LINE__)

std::string sentence(const std::string& payload) {
  std::ostringstream stream;
  stream << '$' << payload << '*' << std::hex << std::uppercase
         << std::setfill('0') << std::setw(2)
         << static_cast<unsigned int>(nmeaChecksum(payload));
  return stream.str();
}

void testParser() {
  EXPECT_TRUE(nmeaChecksum(
                  "GNGGA,023634.00,4004.73871635,N,11614.19729418,E,1,28,"
                  "0.7,61.0988,M,-8.4923,M,,") == 0x58U);

  const ParseResult gga = parseLine(sentence(
      "GNGGA,054733.00,4004.73893635,N,11614.19823325,E,4,28,0.7,"
      "61.0988,M,-8.4923,M,0.5,0042"));
  EXPECT_TRUE(gga.syntax_ok && gga.checksum_valid);
  EXPECT_TRUE(gga.kind == SentenceKind::kGga);
  EXPECT_NEAR(gga.gga.latitude_deg, 40.0789822725, 1e-10);
  EXPECT_TRUE(gga.gga.quality == 4);
  EXPECT_NEAR(gga.gga.msl_height_m + gga.gga.geoid_separation_m,
              52.6065, 1e-9);

  const ParseResult secondary = parseLine(sentence(
      "GNGGAH,054733.00,4004.73893635,N,11614.19823325,E,4,28,0.7,"
      "61.0988,M,-8.4923,M,0.5,0042"));
  EXPECT_TRUE(secondary.syntax_ok && secondary.checksum_valid);
  EXPECT_TRUE(secondary.kind == SentenceKind::kGga);
  EXPECT_TRUE(secondary.secondary_antenna);
  EXPECT_TRUE(secondary.formatter == "GGAH");

  const std::vector<std::string> other_secondary_payloads = {
      "GNGNSH,054733.00,4004.73893635,N,11614.19823325,E,FR,28,0.7,"
      "61.0988,-8.4923,0.5,0042,S",
      "GNGSTH,054733.00,0.02,0.03,0.01,25.0,0.02,0.03,0.05",
      "GNRMCH,054733.00,A,4004.73893635,N,11614.19823325,E,1.000,"
      "90.0,301221,6.9,W,D,S",
      "GNVTGH,90.0,T,,M,1.000,N,1.852,K,D"};
  const std::vector<std::string> other_secondary_formatters = {
      "GNSH", "GSTH", "RMCH", "VTGH"};
  for (std::size_t index = 0U; index < other_secondary_payloads.size(); ++index) {
    const ParseResult parsed =
        parseLine(sentence(other_secondary_payloads[index]));
    EXPECT_TRUE(parsed.syntax_ok && parsed.checksum_valid);
    EXPECT_TRUE(parsed.secondary_antenna);
    EXPECT_TRUE(parsed.formatter == other_secondary_formatters[index]);
  }

  const ParseResult gns = parseLine(sentence(
      "GNGNS,054733.00,4004.73893635,N,11614.19823325,E,FR,28,0.7,"
      "61.0988,-8.4923,0.5,0042,S"));
  EXPECT_TRUE(gns.syntax_ok && gns.checksum_valid);
  EXPECT_TRUE(gns.kind == SentenceKind::kGns);
  EXPECT_TRUE(nmea_rtk_driver::protocol::qualityFromGnsMode(gns.gns.mode) == 9);
  EXPECT_TRUE(nmea_rtk_driver::protocol::fixQualityName(9) ==
              "RTK_UNSPECIFIED");
  EXPECT_TRUE(gns.gns.navigation_status == "S");

  const ParseResult gst = parseLine(sentence(
      "GNGST,054733.00,0.02,0.03,0.01,25.0,0.02,0.03,0.05"));
  EXPECT_TRUE(gst.syntax_ok && gst.checksum_valid);
  EXPECT_TRUE(gst.gst.has_error_ellipse && gst.gst.has_position_std);
  EXPECT_NEAR(gst.gst.longitude_std_m, 0.03, 1e-12);

  const ParseResult rmc = parseLine(sentence(
      "GNRMC,054733.00,A,4004.73893635,N,11614.19823325,E,1.000,"
      "90.0,301221,6.9,W,D,S"));
  EXPECT_TRUE(rmc.syntax_ok && rmc.checksum_valid && rmc.rmc.valid);
  EXPECT_NEAR(rmc.rmc.speed_mps, 0.5144444444444445, 1e-14);
  EXPECT_TRUE(rmc.rmc.date_ddmmyy == "301221");

  const ParseResult unsafe_rmc = parseLine(sentence(
      "GNRMC,054733.00,A,4004.73893635,N,11614.19823325,E,1.000,"
      "90.0,301221,6.9,W,D,C"));
  EXPECT_TRUE(unsafe_rmc.syntax_ok);
  EXPECT_FALSE(unsafe_rmc.rmc.valid);

  const ParseResult vtg = parseLine(
      sentence("GNVTG,90.0,T,,M,1.000,N,1.852,K,D"));
  EXPECT_TRUE(vtg.syntax_ok && vtg.vtg.valid);
  EXPECT_NEAR(vtg.vtg.speed_mps, 1.852 / 3.6, 1e-12);

  const ParseResult ths = parseLine(sentence("GNTHS,341.3344,A"));
  EXPECT_TRUE(ths.syntax_ok && ths.ths.valid);

  const ParseResult gsa = parseLine(sentence(
      "GNGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.2,0.7,1.0,1"));
  EXPECT_TRUE(gsa.syntax_ok && gsa.checksum_valid);
  EXPECT_TRUE(gsa.gsa.satellite_ids.size() == 12U);
  EXPECT_NEAR(gsa.gsa.pdop, 1.2, 1e-12);

  const ParseResult zda =
      parseLine(sentence("GNZDA,054733.00,30,12,2021,00,00"));
  EXPECT_TRUE(zda.syntax_ok && zda.checksum_valid);
  double unix_seconds = 0.0;
  EXPECT_TRUE(nmea_rtk_driver::protocol::utcUnixSeconds(
      zda.zda.year, zda.zda.month, zda.zda.day,
      zda.zda.utc_seconds_of_day, &unix_seconds));
  EXPECT_NEAR(unix_seconds, 1640843253.0, 1e-9);

  ParseResult damaged = gga;
  (void)damaged;
  std::string bad = sentence(
      "GNGGA,054733.00,4004.73893635,N,11614.19823325,E,4,28,0.7,"
      "61.0988,M,-8.4923,M,0.5,0042");
  bad[bad.size() - 1U] = bad[bad.size() - 1U] == '0' ? '1' : '0';
  const ParseResult bad_checksum = parseLine(bad);
  EXPECT_TRUE(bad_checksum.syntax_ok);
  EXPECT_FALSE(bad_checksum.checksum_valid);
}

void testEpochAssembly() {
  AssemblerConfig config;
  config.assembly_delay_sec = 0.05;
  config.stale_timeout_sec = 0.5;
  config.preferred_position = SentenceKind::kGga;
  config.velocity_policy = nmea_rtk_driver::protocol::VelocityPolicy::kAuto;
  config.preferred_velocity = SentenceKind::kRmc;
  EpochAssembler assembler(config);

  const ParseResult gns = parseLine(sentence(
      "GNGNS,054733.00,4004.73893635,N,11614.19823325,E,FR,28,0.7,"
      "61.0988,-8.4923,0.5,0042,S"));
  const ParseResult gga = parseLine(sentence(
      "GNGGA,054733.00,4004.73893635,N,11614.19823325,E,4,28,0.7,"
      "61.0988,M,-8.4923,M,0.5,0042"));
  const ParseResult gst = parseLine(sentence(
      "GNGST,054733.00,0.02,0.03,0.01,25.0,0.02,0.03,0.05"));
  const ParseResult vtg = parseLine(
      sentence("GNVTG,80.0,T,,M,1.000,N,1.852,K,D"));
  const ParseResult rmc = parseLine(sentence(
      "GNRMC,054733.00,A,4004.73893635,N,11614.19823325,E,1.000,"
      "90.0,301221,6.9,W,D,S"));
  const ParseResult gsa = parseLine(sentence(
      "GNGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.2,0.7,1.0,1"));

  assembler.ingest(gsa, 1000.000, 10.000);
  assembler.ingest(gns, 1000.001, 10.001);
  assembler.ingest(gga, 1000.002, 10.002);
  assembler.ingest(gst, 1000.003, 10.003);
  assembler.ingest(vtg, 1000.004, 10.004);
  assembler.ingest(rmc, 1000.005, 10.005);
  EXPECT_TRUE(assembler.flush(10.02).empty());
  const std::vector<nmea_rtk_driver::protocol::EpochObservation> output =
      assembler.flush(10.10);
  EXPECT_TRUE(output.size() == 1U);
  if (!output.empty()) {
    EXPECT_TRUE(output[0].position.source == SentenceKind::kGga);
    EXPECT_TRUE(output[0].position.fix_quality == 4);
    EXPECT_TRUE(output[0].velocity.source == SentenceKind::kRmc);
    EXPECT_TRUE(output[0].velocity.valid);
    EXPECT_TRUE(output[0].covariance.has_position_std);
    EXPECT_TRUE(output[0].gsa.present);
    EXPECT_TRUE(output[0].utc_date_ddmmyy == "301221");
  }

  assembler.ingest(gga, 1000.100, 10.11);
  EXPECT_TRUE(assembler.pendingCount() == 0U);
  EXPECT_TRUE(assembler.duplicateCount() == 1U);

  AssemblerConfig gns_config;
  gns_config.position_policy =
      nmea_rtk_driver::protocol::PositionPolicy::kGnsOnly;
  gns_config.assembly_delay_sec = 0.01;
  EpochAssembler gns_assembler(gns_config);
  const ParseResult caution_gns = parseLine(sentence(
      "GNGNS,054734.00,4004.73893635,N,11614.19823325,E,FR,28,0.7,"
      "61.0988,-8.4923,0.5,0042,C"));
  gns_assembler.ingest(caution_gns, 1001.0, 11.0);
  const auto caution_output = gns_assembler.flush(11.1);
  EXPECT_TRUE(caution_output.size() == 1U);
  if (!caution_output.empty()) {
    EXPECT_FALSE(caution_output[0].position.valid);
    EXPECT_TRUE(caution_output[0].position.fix_quality == 9);
  }

  AssemblerConfig atomic_config;
  atomic_config.require_covariance = true;
  atomic_config.require_velocity = true;
  atomic_config.assembly_delay_sec = 0.05;
  atomic_config.stale_timeout_sec = 0.50;
  EpochAssembler atomic_assembler(atomic_config);
  atomic_assembler.ingest(gga, 2000.000, 20.000);
  EXPECT_TRUE(atomic_assembler.flush(20.10).empty());
  atomic_assembler.ingest(gst, 2000.001, 20.110);
  EXPECT_TRUE(atomic_assembler.flush(20.12).empty());
  atomic_assembler.ingest(rmc, 2000.002, 20.130);
  const auto atomic_output = atomic_assembler.flush(20.14);
  EXPECT_TRUE(atomic_output.size() == 1U);
  if (!atomic_output.empty()) {
    EXPECT_TRUE(atomic_output[0].covariance.present);
    EXPECT_TRUE(atomic_output[0].velocity.present);
  }

  const ParseResult inertial_gga = parseLine(sentence(
      "GNGGA,054735.00,4004.73893635,N,11614.19823325,E,6,28,0.7,"
      "61.0988,M,-8.4923,M,,"));
  EpochAssembler inertial_assembler;
  inertial_assembler.ingest(inertial_gga, 3000.0, 30.0);
  const auto inertial_output = inertial_assembler.flush(30.1);
  EXPECT_TRUE(inertial_output.size() == 1U);
  if (!inertial_output.empty()) {
    EXPECT_FALSE(inertial_output[0].position.valid);
  }
}

void testFraming() {
  NmeaFramer framer(128U);
  std::vector<std::string> completed;
  auto feed = [&framer, &completed](const std::string& bytes) {
    for (char byte : bytes) {
      std::string frame;
      if (framer.push(byte, &frame) == FrameEvent::kCompleted) {
        completed.push_back(frame);
      }
    }
  };

  feed("noise-before-$GNGGA,1,2");
  EXPECT_TRUE(completed.empty());
  feed(",3*00\r\n$GNTHS,1.0,A*00\r\n");
  EXPECT_TRUE(completed.size() == 2U);
  EXPECT_TRUE(completed[0] == "$GNGGA,1,2,3*00");
  EXPECT_TRUE(completed[1] == "$GNTHS,1.0,A*00");

  std::string ignored;
  EXPECT_TRUE(framer.push('$', &ignored) == FrameEvent::kStarted);
  EXPECT_TRUE(framer.push('A', &ignored) == FrameEvent::kNone);
  EXPECT_TRUE(framer.push(static_cast<char>(0x01), &ignored) ==
              FrameEvent::kDroppedMalformed);
  EXPECT_FALSE(framer.collecting());

  NmeaFramer short_framer(20U);
  FrameEvent last_event = FrameEvent::kNone;
  for (char byte : std::string("$1234567890123456789012345")) {
    std::string frame;
    const FrameEvent event = short_framer.push(byte, &frame);
    if (event != FrameEvent::kNone) {
      last_event = event;
    }
  }
  EXPECT_TRUE(last_event == FrameEvent::kDroppedOversize);
  std::string recovered;
  for (char byte : std::string("$A*00\n")) {
    std::string frame;
    if (short_framer.push(byte, &frame) == FrameEvent::kCompleted) {
      recovered = frame;
    }
  }
  EXPECT_TRUE(recovered == "$A*00");
}

}  // namespace

int main() {
  testParser();
  testEpochAssembly();
  testFraming();
  if (g_failures != 0) {
    std::cerr << g_failures << " NMEA protocol test(s) failed\n";
    return 1;
  }
  std::cout << "All NMEA protocol tests passed\n";
  return 0;
}
