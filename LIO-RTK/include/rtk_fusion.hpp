#pragma once

#include <ros/ros.h>

#include <geometry_msgs/Vector3.h>

#include <nmea_rtk_driver/RtkEpoch.h>
#include <nmea_rtk_driver/RtkHeading.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <algorithm>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

#include "use-ikfom.hpp"

namespace fast_lio_rtk {

using IkfomFilter = esekfom::esekf<state_ikfom, 12, input_ikfom>;

struct TimedRtkEvent {
  enum class Type { kEpoch, kHeading };

  Type type = Type::kEpoch;
  double stamp = 0.0;
  nmea_rtk_driver::RtkEpoch epoch;
  nmea_rtk_driver::RtkHeading heading;
};

struct GlobalPose {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  bool valid = false;
};

struct PositionHistorySample {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double stamp = -1.0;
  Eigen::Vector3d measured_enu = Eigen::Vector3d::Zero();
  Eigen::Vector3d antenna_world = Eigen::Vector3d::Zero();
};

class RtkFusion {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  void loadParameters(ros::NodeHandle& nh);
  void epochCallback(const nmea_rtk_driver::RtkEpoch::ConstPtr& message);
  void headingCallback(const nmea_rtk_driver::RtkHeading::ConstPtr& message);

  std::vector<TimedRtkEvent> takeEvents(double begin_time, double end_time);
  bool applyEvent(const TimedRtkEvent& event,
                  IkfomFilter& filter,
                  const input_ikfom& imu_input);
  void updateTimeout(double filter_time);

  bool enabled() const { return enabled_; }
  bool globalTransformMode() const;
  double waitDelaySec() const { return wait_delay_sec_; }
  const std::string& epochTopic() const { return epoch_topic_; }
  const std::string& headingTopic() const { return heading_topic_; }
  const std::string& globalFrameId() const { return global_frame_id_; }
  GlobalPose toGlobalPose(const Eigen::Vector3d& position_world,
                          const Eigen::Matrix3d& rotation_world) const;
  bool globalTransform(Eigen::Matrix3d* rotation_enu_world,
                       Eigen::Vector3d* translation_enu_world) const;
  bool interventionReady(double filter_time) const;

  // Request a global-output intervention when the local LiDAR estimate is
  // declared degraded.  This never modifies state_ikfom or the LiDAR map.
  void setLocalDegraded(bool degraded);

 private:
  enum class FusionMode { kSharedState, kGlobalTransform };
  enum class Health { kWaiting, kCandidate, kActive, kDegraded, kLost, kRecovering };

  bool epochQualityValid(const nmea_rtk_driver::RtkEpoch& message,
                         double* covariance_scale,
                         std::string* rejection_reason) const;
  bool initializeAlignment(const nmea_rtk_driver::RtkEpoch& message,
                           double epoch_stamp,
                           const state_ikfom& state);
  bool applyEpoch(const nmea_rtk_driver::RtkEpoch& message,
                  double event_stamp,
                  IkfomFilter& filter,
                  const input_ikfom& imu_input);
  bool applyHeading(const nmea_rtk_driver::RtkHeading& message,
                    double event_stamp,
                    IkfomFilter& filter);
  bool applyGlobalEpoch(const nmea_rtk_driver::RtkEpoch& message,
                        double event_stamp,
                        IkfomFilter& filter);
  bool applyGlobalHeading(const nmea_rtk_driver::RtkHeading& message,
                          double event_stamp,
                          const state_ikfom& state);
  bool beginPositionEpoch(const nmea_rtk_driver::RtkEpoch& message,
                          double event_stamp,
                          const state_ikfom& state,
                          double* quality_scale);
  void markPositionAccepted(std::uint8_t fix_quality, double event_stamp);
  void markBad(const char* reason);
  void enterPositionQuarantine(const char* reason);
  double recoveryScale() const;
  bool headingUpdateAllowed(double event_stamp) const;
  bool findHeadingForStamp(double stamp,
                           nmea_rtk_driver::RtkHeading* heading,
                           double* age_sec) const;
  void propagateGlobalTransformCovariance(double event_stamp,
                                          double horizontal_process_scale = 1.0);
  bool updateGlobalTransform(const Eigen::VectorXd& residual,
                             const Eigen::MatrixXd& h,
                             const Eigen::MatrixXd& covariance,
                             double nis_gate,
                             bool allow_yaw_correction,
                             bool pivot_about_anchor,
                             const Eigen::Vector3d& anchor_world,
                             const Eigen::Vector3d& anchor_enu,
                             double* nis,
                             bool clip_large_innovation = false,
                             bool* innovation_was_clipped = nullptr);

  static Eigen::Matrix3d skew(const Eigen::Vector3d& value);
  static double wrapAngle(double angle);
  static Eigen::Vector3d geodeticToEcef(double latitude_rad,
                                        double longitude_rad,
                                        double height_m);
  Eigen::Vector3d geodeticToEnu(double latitude_deg,
                                double longitude_deg,
                                double height_m) const;

  mutable std::mutex mutex_;
  std::deque<TimedRtkEvent> events_;

  bool enabled_ = false;
  FusionMode fusion_mode_ = FusionMode::kGlobalTransform;
  std::string epoch_topic_ = "/rtk/epoch";
  std::string heading_topic_ = "/rtk/heading";
  std::string global_frame_id_ = "enu";
  double time_offset_sec_ = 0.0;
  double heading_time_offset_sec_ = 0.0;
  bool use_utc_measurement_time_ = true;
  double max_utc_ros_offset_sec_ = 1.0;
  double wait_delay_sec_ = 0.15;
  double timeout_sec_ = 1.0;
  int min_good_epochs_ = 5;
  int max_bad_epochs_ = 3;
  int recovery_ramp_epochs_ = 20;
  double recovery_initial_scale_ = 25.0;
  bool allow_float_ = true;
  double float_covariance_scale_ = 10.0;
  int min_satellites_ = 10;
  double max_hdop_ = 2.5;
  double max_pdop_ = 4.0;
  double max_differential_age_sec_ = 5.0;
  double max_position_innovation_m_ = 5.0;
  double max_velocity_innovation_mps_ = 3.0;
  double max_heading_innovation_rad_ = 0.5235987755982988;
  double nis_gate_ = 25.0;
  double position_nis_gate_ = 16.27;
  double velocity_nis_gate_ = 13.82;
  double heading_nis_gate_ = 10.83;
  double min_horizontal_std_m_ = 0.03;
  double min_vertical_std_m_ = 0.06;
  double velocity_std_mps_ = 0.20;
  double heading_std_rad_ = 0.03490658503988659;
  double heading_match_window_sec_ = 0.25;
  double heading_max_position_age_sec_ = 0.5;
  bool require_heading_for_alignment_ = true;
  bool use_manual_yaw_ = false;
  // The global-transform path can constrain the full antenna position. Keep
  // this enabled by default so a valid ellipsoid height also protects the
  // vertical component of the FAST-LIO trajectory.
  bool use_vertical_position_ = true;
  bool use_velocity_ = false;
  bool use_heading_updates_ = false;
  bool global_position_updates_yaw_ = false;
  double manual_yaw_rad_ = 0.0;
  Eigen::Vector3d antenna_lever_arm_i_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d heading_baseline_i_ = Eigen::Vector3d::UnitX();

  Health health_ = Health::kWaiting;
  int quality_good_streak_ = 0;
  int rejected_epoch_streak_ = 0;
  int accepted_recovery_epochs_ = 0;
  double last_received_epoch_stamp_ = -1.0;
  double last_accepted_epoch_stamp_ = -1.0;
  std::uint64_t late_epoch_drop_count_ = 0;
  std::uint64_t late_heading_drop_count_ = 0;
  std::uint64_t accepted_position_count_ = 0;
  std::uint64_t accepted_velocity_count_ = 0;
  std::uint64_t accepted_heading_count_ = 0;
  std::uint64_t rejected_position_count_ = 0;
  std::uint64_t rejected_heading_count_ = 0;
  double measurement_time_from_arrival_sec_ = 0.0;
  bool has_measurement_time_from_arrival_ = false;

  bool aligned_ = false;
  bool origin_has_height_ = false;
  double origin_latitude_rad_ = 0.0;
  double origin_longitude_rad_ = 0.0;
  Eigen::Vector3d origin_ecef_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3d ecef_to_enu_ = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d enu_to_world_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d enu_origin_in_world_ = Eigen::Vector3d::Zero();
  std::deque<nmea_rtk_driver::RtkHeading> heading_history_;

  Eigen::Matrix4d global_transform_covariance_ = Eigen::Matrix4d::Identity();
  double global_transform_last_stamp_ = -1.0;
  double global_initial_translation_std_m_ = 1.0;
  double global_initial_yaw_std_rad_ = 0.17453292519943295;
  double global_translation_random_walk_m_sqrt_s_ = 0.10;
  double global_yaw_random_walk_rad_sqrt_s_ = 0.017453292519943295;
  // Robust position protection for the dynamic ENU <- camera_init transform.
  // The ordinary NIS gate is deliberately kept as a statistical check, but it
  // cannot reject a fixed-quality, metre-level position fault when the
  // transform prior is uncertain. Independent temporal gates compare RTK
  // displacement to a frozen local-trajectory reference. CUSUM is retained as
  // a diagnostic trend statistic and is not a single-sample hard gate.
  bool robust_position_gate_enabled_ = true;
  double max_position_residual_hard_m_ = 3.0;
  double max_vertical_position_residual_m_ = 1.0;
  double max_position_recovery_residual_m_ = 0.80;
  // A genuine RTK outage can let the local LIO drift farther than the
  // anomaly-recovery gate.  This wider gate is used only to build a
  // post-outage candidate segment; it never authorizes a first-frame
  // transform jump.
  double max_position_outage_recovery_residual_m_ = 3.0;
  // For an anomaly (as opposed to an outage), recovery must return close to
  // the absolute residual that was present immediately before quarantine.
  double max_position_recovery_return_residual_m_ = 0.60;
  // While an anomaly is quarantined, candidate samples are retained only for
  // a return-to-clean fixed-lag test.  This gate is stricter than the normal
  // fault-detection lag gates and prevents a slowly varying fault from
  // reactivating before its residual has returned to the trusted baseline.
  double max_position_recovery_lag_residual_m_ = 0.25;
  double max_position_step_residual_m_ = 0.25;
  double max_position_step_rate_mps_ = 0.75;
  double position_return_edge_min_ratio_ = 0.50;
  double position_return_edge_max_ratio_ = 1.50;
  double position_return_edge_closure_m_ = 0.35;
  double position_segment_gate_m_ = 0.30;
  double position_segment_drift_rate_mps_ = 0.004;
  double position_segment_gate_max_m_ = 1.00;
  double position_segment_refresh_sec_ = 5.0;
  double position_history_timeout_sec_ = 2.0;
  int position_quarantine_good_epochs_ = 5;
  double min_fixed_horizontal_std_m_ = 0.03;
  double min_fixed_vertical_std_m_ = 0.06;
  double max_position_correction_m_ = 0.05;
  double position_short_lag_sec_ = 1.0;
  double position_long_lag_sec_ = 5.0;
  double position_short_lag_gate_m_ = 0.30;
  double position_long_lag_gate_m_ = 0.28;
  double position_cusum_deadband_m_ = 0.08;
  double position_cusum_limit_m_ = 0.40;
  int position_cusum_trigger_epochs_ = 5;
  int position_history_max_samples_ = 120;
  // Temporal checks are confidence indicators for ordinary slow drift.  They
  // are converted to a bounded covariance inflation instead of freezing the
  // complete ENU transform.  Only a clearly instantaneous, large jump enters
  // the quarantine path.
  double robust_soft_scale_gain_ = 10.0;
  double robust_soft_scale_max_ = 1000.0;
  double robust_soft_deadband_ratio_ = 1.25;
  double robust_cusum_soft_factor_ = 0.75;
  double robust_soft_activation_m_ = 0.35;
  double robust_hard_fault_ratio_ = 3.0;

  bool position_quarantine_ = false;
  bool position_recovery_pending_ = false;
  // True only after a genuine RTK outage has passed the candidate-segment
  // hysteresis. Large but smooth local-LIO drift can then be corrected through
  // bounded innovation clipping instead of permanent NIS rejection.
  bool quarantine_recovery_active_ = false;
  int position_quarantine_good_streak_ = 0;
  // Position quarantine freezes the position reference, but an independently
  // valid dual-antenna heading may still be used.  Keep a snapshot of the
  // transform at quarantine entry so heading corrections do not change the
  // position recovery residuals.
  bool has_quarantine_transform_ = false;
  Eigen::Matrix3d quarantine_rotation_enu_world_ =
      Eigen::Matrix3d::Identity();
  Eigen::Vector3d quarantine_translation_enu_world_ =
      Eigen::Vector3d::Zero();
  bool has_position_segment_anchor_ = false;
  Eigen::Vector3d segment_anchor_measured_enu_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d segment_anchor_antenna_world_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3d segment_anchor_rotation_enu_world_ =
      Eigen::Matrix3d::Identity();
  double segment_anchor_stamp_ = -1.0;
  // Residual at the last trusted sample before an anomaly quarantine.  A
  // recovery check against the original alignment anchor would become
  // needlessly strict after a long, otherwise healthy trajectory; comparing
  // against this local baseline preserves the fault-return test without
  // accumulating hundreds of seconds of LIO drift in the gate.
  bool has_quarantine_reference_ = false;
  Eigen::Vector2d quarantine_segment_residual_ = Eigen::Vector2d::Zero();
  Eigen::Vector2d quarantine_absolute_residual_ = Eigen::Vector2d::Zero();
  double quarantine_reference_stamp_ = -1.0;
  bool has_last_accepted_position_ = false;
  Eigen::Vector3d last_accepted_measured_enu_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d last_accepted_antenna_world_ = Eigen::Vector3d::Zero();
  double last_accepted_position_stamp_ = -1.0;
  std::deque<PositionHistorySample,
             Eigen::aligned_allocator<PositionHistorySample>>
      position_history_;
  // Timestamp of the most recent epoch actually consumed by the filter.
  // Callback arrival timestamps can point into the future during rosbag
  // playback and must not mask a real dropout in updateTimeout().
  double last_processed_epoch_stamp_ = -1.0;
  Eigen::Vector2d position_cusum_ = Eigen::Vector2d::Zero();
  int position_cusum_bad_streak_ = 0;
  Eigen::Vector3d last_anchor_world_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d last_anchor_enu_ = Eigen::Vector3d::Zero();
  bool has_last_anchor_ = false;

  bool local_degraded_ = false;
  double local_intervention_max_correction_m_ = 1.0;
  double local_intervention_max_rtk_speed_mps_ = 3.0;
  double local_intervention_step_margin_m_ = 0.25;
  bool intervention_fault_active_ = false;
  int intervention_recovery_good_streak_ = 0;
  bool has_intervention_received_ = false;
  Eigen::Vector3d intervention_last_received_enu_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d intervention_last_received_antenna_world_ =
      Eigen::Vector3d::Zero();
  Eigen::Vector2d intervention_last_step_enu_ = Eigen::Vector2d::Zero();
  Eigen::Vector2d latest_received_motion_residual_ =
      Eigen::Vector2d::Zero();
  bool has_latest_received_motion_residual_ = false;
  double intervention_last_received_stamp_ = -1.0;
  bool latest_received_step_good_ = true;

  // A position quarantine caused by a discontinuity must not wait forever
  // for the absolute LIO/RTK residual to return to its pre-fault value: the
  // local LIO can drift while the RTK stream is held.  For a step-like fault,
  // the opposite raw RTK edge followed by several ordinary increments is a
  // stronger return-to-clean cue than that absolute residual alone.
  bool quarantine_step_fault_seen_ = false;
  bool quarantine_return_edge_seen_ = false;
  int quarantine_return_good_streak_ = 0;
  Eigen::Vector2d quarantine_fault_step_ = Eigen::Vector2d::Zero();
};

}  // namespace fast_lio_rtk
