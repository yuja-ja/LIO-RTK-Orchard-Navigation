#include "nmea_rtk_driver/epoch_assembler.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace nmea_rtk_driver {
namespace protocol {
namespace {

double circularUtcDifference(double lhs, double rhs) {
  double difference = std::fmod(std::abs(lhs - rhs), 86400.0);
  if (difference < 0.0) {
    difference += 86400.0;
  }
  return std::min(difference, 86400.0 - difference);
}

bool usableGgaQuality(int quality) {
  return quality >= 1 && quality <= 5;
}

PositionObservation fromGga(const ParseResult& parsed,
                            double arrival_stamp_sec) {
  PositionObservation output;
  const Gga& input = parsed.gga;
  output.present = true;
  output.valid = usableGgaQuality(input.quality) && input.has_position;
  output.has_coordinates = input.has_position;
  output.source = SentenceKind::kGga;
  output.talker = parsed.talker;
  output.formatter = parsed.formatter;
  output.utc_text = input.utc_text;
  output.utc_seconds_of_day = input.utc_seconds_of_day;
  output.arrival_stamp_sec = arrival_stamp_sec;
  output.latitude_deg = input.latitude_deg;
  output.longitude_deg = input.longitude_deg;
  output.fix_quality = input.quality;
  output.fix_mode = fixQualityName(input.quality);
  output.satellites_used = input.satellites_used;
  output.has_hdop = input.has_hdop;
  output.hdop = input.hdop;
  output.has_msl_height = input.has_msl_height;
  output.msl_height_m = input.msl_height_m;
  output.has_geoid_separation = input.has_geoid_separation;
  output.geoid_separation_m = input.geoid_separation_m;
  output.has_ellipsoid_height =
      input.has_msl_height && input.has_geoid_separation;
  if (output.has_ellipsoid_height) {
    output.ellipsoid_height_m =
        input.msl_height_m + input.geoid_separation_m;
  }
  output.has_differential_age = input.has_differential_age;
  output.differential_age_sec = input.differential_age_sec;
  output.differential_station_id = input.differential_station_id;
  return output;
}

PositionObservation fromGns(const ParseResult& parsed,
                            double arrival_stamp_sec) {
  PositionObservation output;
  const Gns& input = parsed.gns;
  output.present = true;
  output.valid = gnsModeHasFix(input.mode) && input.has_position &&
                 (input.navigation_status.empty() ||
                  input.navigation_status == "S");
  output.has_coordinates = input.has_position;
  output.source = SentenceKind::kGns;
  output.talker = parsed.talker;
  output.formatter = parsed.formatter;
  output.utc_text = input.utc_text;
  output.utc_seconds_of_day = input.utc_seconds_of_day;
  output.arrival_stamp_sec = arrival_stamp_sec;
  output.latitude_deg = input.latitude_deg;
  output.longitude_deg = input.longitude_deg;
  output.fix_quality = qualityFromGnsMode(input.mode);
  output.fix_mode = input.mode;
  output.satellites_used = input.satellites_used;
  output.has_hdop = input.has_hdop;
  output.hdop = input.hdop;
  output.has_msl_height = input.has_msl_height;
  output.msl_height_m = input.msl_height_m;
  output.has_geoid_separation = input.has_geoid_separation;
  output.geoid_separation_m = input.geoid_separation_m;
  output.has_ellipsoid_height =
      input.has_msl_height && input.has_geoid_separation;
  if (output.has_ellipsoid_height) {
    output.ellipsoid_height_m =
        input.msl_height_m + input.geoid_separation_m;
  }
  output.has_differential_age = input.has_differential_age;
  output.differential_age_sec = input.differential_age_sec;
  output.differential_station_id = input.differential_station_id;
  output.navigation_status = input.navigation_status;
  return output;
}

CovarianceObservation fromGst(const ParseResult& parsed,
                              double arrival_stamp_sec) {
  CovarianceObservation output;
  const Gst& input = parsed.gst;
  output.present = true;
  output.talker = parsed.talker;
  output.formatter = parsed.formatter;
  output.utc_text = input.utc_text;
  output.utc_seconds_of_day = input.utc_seconds_of_day;
  output.arrival_stamp_sec = arrival_stamp_sec;
  output.has_rms = input.has_rms;
  output.rms_m = input.rms_m;
  output.has_error_ellipse = input.has_error_ellipse;
  output.semi_major_std_m = input.semi_major_std_m;
  output.semi_minor_std_m = input.semi_minor_std_m;
  output.orientation_deg = input.orientation_deg;
  output.has_position_std = input.has_position_std;
  output.latitude_std_m = input.latitude_std_m;
  output.longitude_std_m = input.longitude_std_m;
  output.altitude_std_m = input.altitude_std_m;
  return output;
}

VelocityObservation fromRmc(const ParseResult& parsed,
                            double arrival_stamp_sec) {
  VelocityObservation output;
  const Rmc& input = parsed.rmc;
  output.present = input.has_speed && input.has_track;
  output.valid = input.valid;
  output.source = SentenceKind::kRmc;
  output.talker = parsed.talker;
  output.formatter = parsed.formatter;
  output.utc_text = input.utc_text;
  output.has_utc = input.has_utc;
  output.utc_seconds_of_day = input.utc_seconds_of_day;
  output.arrival_stamp_sec = arrival_stamp_sec;
  output.speed_mps = input.speed_mps;
  output.track_true_deg = input.track_true_deg;
  output.mode = input.mode;
  output.navigation_status = input.navigation_status;
  output.date_ddmmyy = input.date_ddmmyy;
  return output;
}

VelocityObservation fromVtg(const ParseResult& parsed,
                            double arrival_stamp_sec) {
  VelocityObservation output;
  const Vtg& input = parsed.vtg;
  output.present = input.has_speed && input.has_track;
  output.valid = input.valid;
  output.source = SentenceKind::kVtg;
  output.talker = parsed.talker;
  output.formatter = parsed.formatter;
  output.arrival_stamp_sec = arrival_stamp_sec;
  output.speed_mps = input.speed_mps;
  output.track_true_deg = input.track_true_deg;
  output.mode = input.mode;
  return output;
}

}  // namespace

EpochAssembler::EpochAssembler(const AssemblerConfig& config) : config_(config) {
  if (!std::isfinite(config_.utc_match_tolerance_sec) ||
      config_.utc_match_tolerance_sec <= 0.0) {
    config_.utc_match_tolerance_sec = 0.05;
  }
  if (!std::isfinite(config_.assembly_delay_sec) ||
      config_.assembly_delay_sec < 0.0) {
    config_.assembly_delay_sec = 0.08;
  }
  if (!std::isfinite(config_.stale_timeout_sec) ||
      config_.stale_timeout_sec <= config_.assembly_delay_sec) {
    config_.stale_timeout_sec = std::max(0.50, config_.assembly_delay_sec * 2.0);
  }
  if (!std::isfinite(config_.duplicate_retention_sec) ||
      config_.duplicate_retention_sec <= 0.0) {
    config_.duplicate_retention_sec = 2.0;
  }
  if (!std::isfinite(config_.vtg_match_window_sec) ||
      config_.vtg_match_window_sec <= 0.0) {
    config_.vtg_match_window_sec = 0.15;
  }
  if (!std::isfinite(config_.gsa_max_age_sec) || config_.gsa_max_age_sec <= 0.0) {
    config_.gsa_max_age_sec = 2.0;
  }
  config_.maximum_pending_epochs =
      std::max<std::size_t>(2U, config_.maximum_pending_epochs);
}

void EpochAssembler::ingest(const ParseResult& parsed,
                            double arrival_stamp_sec,
                            double received_wall_sec) {
  if (!parsed.syntax_ok || !parsed.recognized || parsed.secondary_antenna ||
      !std::isfinite(arrival_stamp_sec) || !std::isfinite(received_wall_sec)) {
    return;
  }

  if (parsed.kind == SentenceKind::kGsa) {
    latest_gsa_.present = true;
    latest_gsa_.arrival_stamp_sec = arrival_stamp_sec;
    latest_gsa_.received_wall_sec = received_wall_sec;
    latest_gsa_.talker = parsed.talker;
    latest_gsa_.selection_mode = parsed.gsa.selection_mode;
    latest_gsa_.fix_dimension = parsed.gsa.fix_dimension;
    latest_gsa_.satellite_ids = parsed.gsa.satellite_ids;
    latest_gsa_.has_pdop = parsed.gsa.has_pdop;
    latest_gsa_.pdop = parsed.gsa.pdop;
    latest_gsa_.has_hdop = parsed.gsa.has_hdop;
    latest_gsa_.hdop = parsed.gsa.hdop;
    latest_gsa_.has_vdop = parsed.gsa.has_vdop;
    latest_gsa_.vdop = parsed.gsa.vdop;
    latest_gsa_.system_id = parsed.gsa.system_id;
    return;
  }

  if (parsed.kind == SentenceKind::kVtg) {
    if (config_.velocity_policy == VelocityPolicy::kRmcOnly) {
      return;
    }
    const VelocityObservation velocity = fromVtg(parsed, arrival_stamp_sec);
    PendingEpoch* best = nullptr;
    double best_age = std::numeric_limits<double>::infinity();
    for (PendingEpoch& pending : pending_) {
      const double age = std::abs(received_wall_sec - pending.last_seen_wall_sec);
      if (age <= config_.vtg_match_window_sec && age < best_age &&
          pending.position.present) {
        best = &pending;
        best_age = age;
      }
    }
    if (best != nullptr) {
      selectVelocity(velocity, best);
      best->last_seen_wall_sec = received_wall_sec;
    } else {
      orphan_vtg_.present = true;
      orphan_vtg_.velocity = velocity;
      orphan_vtg_.received_wall_sec = received_wall_sec;
    }
    return;
  }

  double utc_seconds_of_day = 0.0;
  std::string utc_text;
  if (parsed.kind == SentenceKind::kGga) {
    utc_seconds_of_day = parsed.gga.utc_seconds_of_day;
    utc_text = parsed.gga.utc_text;
  } else if (parsed.kind == SentenceKind::kGns) {
    utc_seconds_of_day = parsed.gns.utc_seconds_of_day;
    utc_text = parsed.gns.utc_text;
  } else if (parsed.kind == SentenceKind::kGst) {
    utc_seconds_of_day = parsed.gst.utc_seconds_of_day;
    utc_text = parsed.gst.utc_text;
  } else if (parsed.kind == SentenceKind::kRmc) {
    utc_seconds_of_day = parsed.rmc.utc_seconds_of_day;
    utc_text = parsed.rmc.utc_text;
  } else {
    return;
  }

  PendingEpoch* pending =
      findOrCreate(utc_seconds_of_day, utc_text, received_wall_sec);
  if (pending == nullptr) {
    return;
  }
  pending->last_seen_wall_sec = received_wall_sec;

  if (parsed.kind == SentenceKind::kGga) {
    selectPosition(fromGga(parsed, arrival_stamp_sec), pending);
  } else if (parsed.kind == SentenceKind::kGns) {
    selectPosition(fromGns(parsed, arrival_stamp_sec), pending);
  } else if (parsed.kind == SentenceKind::kGst) {
    pending->covariance = fromGst(parsed, arrival_stamp_sec);
  } else if (parsed.kind == SentenceKind::kRmc) {
    const VelocityObservation velocity = fromRmc(parsed, arrival_stamp_sec);
    selectVelocity(velocity, pending);
    if (!velocity.date_ddmmyy.empty()) {
      pending->utc_date_ddmmyy = velocity.date_ddmmyy;
    }
  }

  if (pending->position.present && orphan_vtg_.present &&
      std::abs(received_wall_sec - orphan_vtg_.received_wall_sec) <=
          config_.vtg_match_window_sec) {
    selectVelocity(orphan_vtg_.velocity, pending);
    orphan_vtg_.present = false;
  }
  attachLatestGsa(received_wall_sec, pending);
}

std::vector<EpochObservation> EpochAssembler::flush(double current_wall_sec) {
  std::vector<EpochObservation> completed;
  if (!std::isfinite(current_wall_sec)) {
    return completed;
  }
  pruneEmitted(current_wall_sec);
  if (orphan_vtg_.present &&
      current_wall_sec - orphan_vtg_.received_wall_sec >
          config_.vtg_match_window_sec) {
    orphan_vtg_.present = false;
  }

  for (auto iterator = pending_.begin(); iterator != pending_.end();) {
    const double age = current_wall_sec - iterator->first_seen_wall_sec;
    const bool complete =
        iterator->position.present &&
        (!config_.require_covariance || iterator->covariance.present) &&
        (!config_.require_velocity || iterator->velocity.present);
    const bool ready = complete && age >= config_.assembly_delay_sec;
    const bool stale = age >= config_.stale_timeout_sec;
    if (!ready && !stale) {
      ++iterator;
      continue;
    }
    if (!iterator->position.present) {
      ++stale_drop_count_;
      iterator = pending_.erase(iterator);
      continue;
    }

    attachLatestGsa(current_wall_sec, &(*iterator));
    EpochObservation output;
    output.utc_seconds_of_day = iterator->utc_seconds_of_day;
    output.utc_text = iterator->position.utc_text.empty()
                          ? iterator->utc_text
                          : iterator->position.utc_text;
    output.utc_date_ddmmyy = iterator->utc_date_ddmmyy;
    output.position = iterator->position;
    output.covariance = iterator->covariance;
    output.velocity = iterator->velocity;
    output.gsa = iterator->gsa;
    output.reference_arrival_stamp_sec = iterator->position.arrival_stamp_sec;
    completed.push_back(output);

    EmittedEpoch emitted;
    emitted.utc_seconds_of_day = iterator->utc_seconds_of_day;
    emitted.emitted_wall_sec = current_wall_sec;
    emitted_.push_back(emitted);
    iterator = pending_.erase(iterator);
  }
  return completed;
}

void EpochAssembler::reset() {
  pending_.clear();
  emitted_.clear();
  latest_gsa_ = GsaObservation();
  orphan_vtg_ = OrphanVtg();
  duplicate_count_ = 0U;
  stale_drop_count_ = 0U;
}

EpochAssembler::PendingEpoch* EpochAssembler::findOrCreate(
    double utc_seconds_of_day,
    const std::string& utc_text,
    double received_wall_sec) {
  pruneEmitted(received_wall_sec);
  if (wasRecentlyEmitted(utc_seconds_of_day, received_wall_sec)) {
    ++duplicate_count_;
    return nullptr;
  }

  PendingEpoch* best = nullptr;
  double best_difference = std::numeric_limits<double>::infinity();
  for (PendingEpoch& pending : pending_) {
    const double difference =
        circularUtcDifference(pending.utc_seconds_of_day, utc_seconds_of_day);
    if (difference <= config_.utc_match_tolerance_sec &&
        difference < best_difference) {
      best = &pending;
      best_difference = difference;
    }
  }
  if (best != nullptr) {
    return best;
  }

  if (pending_.size() >= config_.maximum_pending_epochs) {
    pending_.pop_front();
    ++stale_drop_count_;
  }
  PendingEpoch pending;
  pending.utc_seconds_of_day = utc_seconds_of_day;
  pending.utc_text = utc_text;
  pending.first_seen_wall_sec = received_wall_sec;
  pending.last_seen_wall_sec = received_wall_sec;
  pending_.push_back(pending);
  return &pending_.back();
}

bool EpochAssembler::wasRecentlyEmitted(double utc_seconds_of_day,
                                        double received_wall_sec) {
  for (const EmittedEpoch& emitted : emitted_) {
    if (received_wall_sec - emitted.emitted_wall_sec <=
            config_.duplicate_retention_sec &&
        circularUtcDifference(emitted.utc_seconds_of_day, utc_seconds_of_day) <=
            config_.utc_match_tolerance_sec) {
      return true;
    }
  }
  return false;
}

void EpochAssembler::selectPosition(const PositionObservation& incoming,
                                    PendingEpoch* pending) {
  if (pending == nullptr || !incoming.present) {
    return;
  }
  if (config_.position_policy == PositionPolicy::kGgaOnly &&
      incoming.source != SentenceKind::kGga) {
    return;
  }
  if (config_.position_policy == PositionPolicy::kGnsOnly &&
      incoming.source != SentenceKind::kGns) {
    return;
  }
  bool replace = !pending->position.present;
  if (pending->position.present) {
    if (incoming.valid != pending->position.valid) {
      replace = incoming.valid;
    } else if (incoming.source == pending->position.source) {
      replace = true;
    } else if (config_.position_policy == PositionPolicy::kAuto) {
      replace = incoming.source == config_.preferred_position;
    }
  }
  if (replace) {
    pending->position = incoming;
    pending->utc_text = incoming.utc_text;
  }
}

void EpochAssembler::selectVelocity(const VelocityObservation& incoming,
                                    PendingEpoch* pending) {
  if (pending == nullptr || !incoming.present) {
    return;
  }
  if (config_.velocity_policy == VelocityPolicy::kRmcOnly &&
      incoming.source != SentenceKind::kRmc) {
    return;
  }
  if (config_.velocity_policy == VelocityPolicy::kVtgOnly &&
      incoming.source != SentenceKind::kVtg) {
    return;
  }
  bool replace = !pending->velocity.present;
  if (pending->velocity.present) {
    if (incoming.valid != pending->velocity.valid) {
      replace = incoming.valid;
    } else if (incoming.source == pending->velocity.source) {
      replace = true;
    } else if (config_.velocity_policy == VelocityPolicy::kAuto) {
      replace = incoming.source == config_.preferred_velocity;
    }
  }
  if (replace) {
    pending->velocity = incoming;
  }
}

void EpochAssembler::attachLatestGsa(double received_wall_sec,
                                     PendingEpoch* pending) const {
  if (pending == nullptr || !latest_gsa_.present) {
    return;
  }
  const double age = std::abs(received_wall_sec - latest_gsa_.received_wall_sec);
  if (age <= config_.gsa_max_age_sec) {
    pending->gsa = latest_gsa_;
  }
}

void EpochAssembler::pruneEmitted(double current_wall_sec) {
  while (!emitted_.empty() &&
         current_wall_sec - emitted_.front().emitted_wall_sec >
             config_.duplicate_retention_sec) {
    emitted_.pop_front();
  }
}

}  // namespace protocol
}  // namespace nmea_rtk_driver
