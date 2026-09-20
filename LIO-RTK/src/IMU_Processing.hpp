#include <cmath>
#include <math.h>
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>
#include <fstream>
#include <csignal>
#include <ros/ros.h>
#include <so3_math.h>
#include <Eigen/Eigen>
#include <common_lib.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <condition_variable>
#include <nav_msgs/Odometry.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3.h>
#include "use-ikfom.hpp"
#include "preprocess.h"

/// *************Preconfiguration

#define MAX_INI_COUNT (10)

const bool time_list(PointType &x, PointType &y) {return (x.curvature < y.curvature);};

/// *************IMU Process and undistortion
class ImuProcess
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using IkfomFilter = esekfom::esekf<state_ikfom, 12, input_ikfom>;

  // A measurement update executed after IMU propagation reaches timestamp.
  struct TimedStateUpdate
  {
    double timestamp;
    std::function<bool(IkfomFilter &, const input_ikfom &)> apply;

    TimedStateUpdate(
        const double stamp,
        const std::function<bool(IkfomFilter &, const input_ikfom &)> &callback)
        : timestamp(stamp), apply(callback)
    {
    }
  };

  ImuProcess();
  ~ImuProcess();
  
  void Reset();
  void Reset(double start_timestamp, const sensor_msgs::ImuConstPtr &lastimu);
  void set_extrinsic(const V3D &transl, const M3D &rot);
  void set_extrinsic(const V3D &transl);
  void set_extrinsic(const MD(4,4) &T);
  void set_gyr_cov(const V3D &scaler);
  void set_acc_cov(const V3D &scaler);
  void set_gyr_bias_cov(const V3D &b_g);
  void set_acc_bias_cov(const V3D &b_a);
  Eigen::Matrix<double, 12, 12> Q;
  void Process(const MeasureGroup &meas, IkfomFilter &kf_state, PointCloudXYZI::Ptr pcl_un_);
  void Process(const MeasureGroup &meas, IkfomFilter &kf_state, PointCloudXYZI::Ptr pcl_un_,
               const std::vector<TimedStateUpdate> &timed_updates);

  ofstream fout_imu;
  V3D cov_acc;
  V3D cov_gyr;
  V3D cov_acc_scale;
  V3D cov_gyr_scale;
  V3D cov_bias_gyr;
  V3D cov_bias_acc;
  double first_lidar_time;
  int lidar_type;

 private:
  void IMU_init(const MeasureGroup &meas, IkfomFilter &kf_state, int &N);
  void UndistortPcl(const MeasureGroup &meas, IkfomFilter &kf_state, PointCloudXYZI &pcl_in_out,
                    const std::vector<TimedStateUpdate> &timed_updates);

  PointCloudXYZI::Ptr cur_pcl_un_;
  sensor_msgs::ImuConstPtr last_imu_;
  deque<sensor_msgs::ImuConstPtr> v_imu_;
  vector<Pose6D> IMUpose;
  // Marks the posterior side of a zero-duration measurement discontinuity.
  vector<uint8_t> imu_pose_reset_before_;
  vector<M3D>    v_rot_pcl_;
  M3D Lidar_R_wrt_IMU;
  V3D Lidar_T_wrt_IMU;
  V3D mean_acc;
  V3D mean_gyr;
  V3D angvel_last;
  V3D acc_s_last;
  double start_timestamp_;
  double last_lidar_end_time_;
  int    init_iter_num = 1;
  bool   b_first_frame_ = true;
  bool   imu_need_init_ = true;
};

ImuProcess::ImuProcess()
    : start_timestamp_(-1), last_lidar_end_time_(-1),
      b_first_frame_(true), imu_need_init_(true)
{
  init_iter_num = 1;
  Q = process_noise_cov();
  cov_acc       = V3D(0.1, 0.1, 0.1);
  cov_gyr       = V3D(0.1, 0.1, 0.1);
  cov_bias_gyr  = V3D(0.0001, 0.0001, 0.0001);
  cov_bias_acc  = V3D(0.0001, 0.0001, 0.0001);
  mean_acc      = V3D(0, 0, -1.0);
  mean_gyr      = V3D(0, 0, 0);
  angvel_last     = Zero3d;
  Lidar_T_wrt_IMU = Zero3d;
  Lidar_R_wrt_IMU = Eye3d;
  last_imu_.reset(new sensor_msgs::Imu());
}

ImuProcess::~ImuProcess() {}

void ImuProcess::Reset() 
{
  // ROS_WARN("Reset ImuProcess");
  mean_acc      = V3D(0, 0, -1.0);
  mean_gyr      = V3D(0, 0, 0);
  angvel_last       = Zero3d;
  imu_need_init_    = true;
  start_timestamp_  = -1;
  init_iter_num     = 1;
  v_imu_.clear();
  IMUpose.clear();
  imu_pose_reset_before_.clear();
  last_lidar_end_time_ = -1.0;
  last_imu_.reset(new sensor_msgs::Imu());
  cur_pcl_un_.reset(new PointCloudXYZI());
}

void ImuProcess::set_extrinsic(const MD(4,4) &T)
{
  Lidar_T_wrt_IMU = T.block<3,1>(0,3);
  Lidar_R_wrt_IMU = T.block<3,3>(0,0);
}

void ImuProcess::set_extrinsic(const V3D &transl)
{
  Lidar_T_wrt_IMU = transl;
  Lidar_R_wrt_IMU.setIdentity();
}

void ImuProcess::set_extrinsic(const V3D &transl, const M3D &rot)
{
  Lidar_T_wrt_IMU = transl;
  Lidar_R_wrt_IMU = rot;
}

void ImuProcess::set_gyr_cov(const V3D &scaler)
{
  cov_gyr_scale = scaler;
}

void ImuProcess::set_acc_cov(const V3D &scaler)
{
  cov_acc_scale = scaler;
}

void ImuProcess::set_gyr_bias_cov(const V3D &b_g)
{
  cov_bias_gyr = b_g;
}

void ImuProcess::set_acc_bias_cov(const V3D &b_a)
{
  cov_bias_acc = b_a;
}

void ImuProcess::IMU_init(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, int &N)
{
  /** 1. initializing the gravity, gyro bias, acc and gyro covariance
   ** 2. normalize the acceleration measurenments to unit gravity **/
  
  V3D cur_acc, cur_gyr;
  
  if (b_first_frame_)
  {
    Reset();
    N = 1;
    b_first_frame_ = false;
    const auto &imu_acc = meas.imu.front()->linear_acceleration;
    const auto &gyr_acc = meas.imu.front()->angular_velocity;
    mean_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    mean_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;
    first_lidar_time = meas.lidar_beg_time;
  }

  for (const auto &imu : meas.imu)
  {
    const auto &imu_acc = imu->linear_acceleration;
    const auto &gyr_acc = imu->angular_velocity;
    cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

    mean_acc      += (cur_acc - mean_acc) / N;
    mean_gyr      += (cur_gyr - mean_gyr) / N;

    cov_acc = cov_acc * (N - 1.0) / N + (cur_acc - mean_acc).cwiseProduct(cur_acc - mean_acc) * (N - 1.0) / (N * N);
    cov_gyr = cov_gyr * (N - 1.0) / N + (cur_gyr - mean_gyr).cwiseProduct(cur_gyr - mean_gyr) * (N - 1.0) / (N * N);

    // cout<<"acc norm: "<<cur_acc.norm()<<" "<<mean_acc.norm()<<endl;

    N ++;
  }
  state_ikfom init_state = kf_state.get_x();
  init_state.grav = S2(- mean_acc / mean_acc.norm() * G_m_s2);
  
  //state_inout.rot = Eye3d; // Exp(mean_acc.cross(V3D(0, 0, -1 / scale_gravity)));
  init_state.bg  = mean_gyr;
  init_state.offset_T_L_I = Lidar_T_wrt_IMU;
  init_state.offset_R_L_I = Lidar_R_wrt_IMU;
  kf_state.change_x(init_state);

  esekfom::esekf<state_ikfom, 12, input_ikfom>::cov init_P = kf_state.get_P();
  init_P.setIdentity();
  init_P(6,6) = init_P(7,7) = init_P(8,8) = 0.00001;
  init_P(9,9) = init_P(10,10) = init_P(11,11) = 0.00001;
  init_P(15,15) = init_P(16,16) = init_P(17,17) = 0.0001;
  init_P(18,18) = init_P(19,19) = init_P(20,20) = 0.001;
  init_P(21,21) = init_P(22,22) = 0.00001; 
  kf_state.change_P(init_P);
  last_imu_ = meas.imu.back();

}

void ImuProcess::UndistortPcl(
    const MeasureGroup &meas, IkfomFilter &kf_state, PointCloudXYZI &pcl_out,
    const std::vector<TimedStateUpdate> &timed_updates)
{
  const double kTimeEpsilon = 1e-9;

  auto v_imu = meas.imu;
  v_imu.push_front(last_imu_);

  double pcl_beg_time = meas.lidar_beg_time;
  double pcl_end_time = meas.lidar_end_time;
  if (lidar_type == MARSIM)
  {
    pcl_beg_time = last_lidar_end_time_;
    pcl_end_time = meas.lidar_beg_time;
  }

  pcl_out = *(meas.lidar);
  sort(pcl_out.points.begin(), pcl_out.points.end(), time_list);

  std::vector<TimedStateUpdate> updates = timed_updates;
  std::stable_sort(
      updates.begin(), updates.end(),
      [](const TimedStateUpdate &lhs, const TimedStateUpdate &rhs)
      {
        return lhs.timestamp < rhs.timestamp;
      });

  IMUpose.clear();
  imu_pose_reset_before_.clear();

  Q.block<3, 3>(0, 0).diagonal() = cov_gyr;
  Q.block<3, 3>(3, 3).diagonal() = cov_acc;
  Q.block<3, 3>(6, 6).diagonal() = cov_bias_gyr;
  Q.block<3, 3>(9, 9).diagonal() = cov_bias_acc;

  double filter_time = last_lidar_end_time_;
  if (!std::isfinite(filter_time) || filter_time < 0.0)
  {
    filter_time = v_imu.front()->header.stamp.toSec();
  }

  size_t update_index = 0;
  while (update_index < updates.size() &&
         updates[update_index].timestamp < filter_time - kTimeEpsilon)
  {
    ++update_index;
  }

  input_ikfom last_input;
  last_input.acc = Zero3d;
  last_input.gyro = Zero3d;
  bool has_input = false;

  auto inputFromSample = [&](const sensor_msgs::ImuConstPtr &imu)
  {
    input_ikfom sample_input;
    sample_input.gyro << imu->angular_velocity.x,
                         imu->angular_velocity.y,
                         imu->angular_velocity.z;
    sample_input.acc << imu->linear_acceleration.x,
                        imu->linear_acceleration.y,
                        imu->linear_acceleration.z;
    sample_input.acc *= G_m_s2 / mean_acc.norm();
    return sample_input;
  };

  auto interpolatedInput = [&](const sensor_msgs::ImuConstPtr &head,
                               const sensor_msgs::ImuConstPtr &tail,
                               const double timestamp,
                               const input_ikfom &fallback)
  {
    if (!head || !tail)
    {
      return fallback;
    }

    const double head_time = head->header.stamp.toSec();
    const double tail_time = tail->header.stamp.toSec();
    const double sample_dt = tail_time - head_time;
    if (sample_dt <= kTimeEpsilon)
    {
      return fallback;
    }

    const double alpha = std::max(0.0, std::min(1.0, (timestamp - head_time) / sample_dt));
    input_ikfom event_input;
    event_input.gyro <<
        (1.0 - alpha) * head->angular_velocity.x + alpha * tail->angular_velocity.x,
        (1.0 - alpha) * head->angular_velocity.y + alpha * tail->angular_velocity.y,
        (1.0 - alpha) * head->angular_velocity.z + alpha * tail->angular_velocity.z;
    event_input.acc <<
        (1.0 - alpha) * head->linear_acceleration.x + alpha * tail->linear_acceleration.x,
        (1.0 - alpha) * head->linear_acceleration.y + alpha * tail->linear_acceleration.y,
        (1.0 - alpha) * head->linear_acceleration.z + alpha * tail->linear_acceleration.z;
    event_input.acc *= G_m_s2 / mean_acc.norm();
    return event_input;
  };

  auto appendPose = [&](const input_ikfom &pose_input, const bool reset_before)
  {
    if (filter_time < pcl_beg_time - kTimeEpsilon ||
        filter_time > pcl_end_time + kTimeEpsilon)
    {
      return;
    }

    state_ikfom state = kf_state.get_x();
    angvel_last = pose_input.gyro - state.bg;
    acc_s_last = state.rot * (pose_input.acc - state.ba);
    for (int axis = 0; axis < 3; ++axis)
    {
      acc_s_last[axis] += state.grav[axis];
    }

    const double offset = std::max(
        0.0, std::min(pcl_end_time - pcl_beg_time, filter_time - pcl_beg_time));
    Pose6D pose = set_pose6d(offset, acc_s_last, angvel_last, state.vel, state.pos,
                            state.rot.toRotationMatrix());

    if (reset_before && !IMUpose.empty() &&
        std::fabs(IMUpose.back().offset_time - offset) <= kTimeEpsilon &&
        imu_pose_reset_before_.back())
    {
      IMUpose.back() = pose;
      return;
    }

    if (!reset_before && !IMUpose.empty() &&
        std::fabs(IMUpose.back().offset_time - offset) <= kTimeEpsilon)
    {
      return;
    }

    IMUpose.push_back(pose);
    imu_pose_reset_before_.push_back(reset_before ? 1U : 0U);
  };

  auto advanceInterval = [&](const double interval_end,
                             const input_ikfom &propagation_input,
                             const sensor_msgs::ImuConstPtr &head,
                             const sensor_msgs::ImuConstPtr &tail)
  {
    if (IMUpose.empty() && filter_time >= pcl_beg_time - kTimeEpsilon &&
        filter_time <= pcl_end_time + kTimeEpsilon)
    {
      appendPose(propagation_input, false);
    }

    auto applyUpdatesAtCurrentTime = [&]()
    {
      while (update_index < updates.size() &&
             updates[update_index].timestamp <= filter_time + kTimeEpsilon)
      {
        const TimedStateUpdate &update = updates[update_index];
        if (update.timestamp >= filter_time - kTimeEpsilon && update.apply)
        {
          const input_ikfom event_input =
              interpolatedInput(head, tail, filter_time, propagation_input);
          if (update.apply(kf_state, event_input))
          {
            // Keep both the prior and posterior knots at the RTK timestamp.
            appendPose(event_input, true);
          }
        }
        ++update_index;
      }
    };

    applyUpdatesAtCurrentTime();
    while (filter_time < interval_end - kTimeEpsilon)
    {
      double next_time = interval_end;
      if (filter_time < pcl_beg_time - kTimeEpsilon && pcl_beg_time < next_time)
      {
        next_time = pcl_beg_time;
      }
      if (update_index < updates.size() &&
          updates[update_index].timestamp > filter_time + kTimeEpsilon &&
          updates[update_index].timestamp < next_time + kTimeEpsilon)
      {
        next_time = std::min(next_time, updates[update_index].timestamp);
      }

      double dt = next_time - filter_time;
      if (dt > kTimeEpsilon)
      {
        kf_state.predict(dt, Q, propagation_input);
        filter_time = next_time;
        appendPose(propagation_input, false);
      }
      else
      {
        filter_time = next_time;
      }
      applyUpdatesAtCurrentTime();
    }
  };

  for (auto it_imu = v_imu.begin();
       it_imu < v_imu.end() - 1 && filter_time < pcl_end_time - kTimeEpsilon;
       ++it_imu)
  {
    const sensor_msgs::ImuConstPtr &head = *it_imu;
    const sensor_msgs::ImuConstPtr &tail = *(it_imu + 1);
    const double tail_time = tail->header.stamp.toSec();
    if (tail_time <= filter_time + kTimeEpsilon)
    {
      continue;
    }

    const input_ikfom head_input = inputFromSample(head);
    const input_ikfom tail_input = inputFromSample(tail);
    input_ikfom propagation_input;
    propagation_input.acc = 0.5 * (head_input.acc + tail_input.acc);
    propagation_input.gyro = 0.5 * (head_input.gyro + tail_input.gyro);
    last_input = propagation_input;
    has_input = true;

    advanceInterval(std::min(tail_time, pcl_end_time), propagation_input, head, tail);
  }

  if (!has_input)
  {
    last_input = inputFromSample(v_imu.back());
    has_input = true;
  }
  if (filter_time < pcl_end_time - kTimeEpsilon)
  {
    advanceInterval(pcl_end_time, last_input, sensor_msgs::ImuConstPtr(),
                    sensor_msgs::ImuConstPtr());
  }

  if (IMUpose.empty())
  {
    filter_time = pcl_end_time;
    appendPose(last_input, false);
  }

  state_ikfom imu_state = kf_state.get_x();
  last_imu_ = meas.imu.back();
  last_lidar_end_time_ = pcl_end_time;

  if (pcl_out.empty() || lidar_type == MARSIM || IMUpose.size() < 2)
  {
    return;
  }

  V3D angvel_avr, acc_imu, vel_imu, pos_imu;
  M3D R_imu;
  std::ptrdiff_t point_index = static_cast<std::ptrdiff_t>(pcl_out.points.size()) - 1;

  for (size_t pose_index = IMUpose.size() - 1;
       pose_index > 0 && point_index >= 0;
       --pose_index)
  {
    if (imu_pose_reset_before_[pose_index])
    {
      // A filter update is discontinuous; it is not an IMU integration segment.
      continue;
    }

    const Pose6D &head = IMUpose[pose_index - 1];
    const Pose6D &tail = IMUpose[pose_index];
    if (tail.offset_time <= head.offset_time + kTimeEpsilon)
    {
      continue;
    }

    R_imu << MAT_FROM_ARRAY(head.rot);
    vel_imu << VEC_FROM_ARRAY(head.vel);
    pos_imu << VEC_FROM_ARRAY(head.pos);
    acc_imu << VEC_FROM_ARRAY(tail.acc);
    angvel_avr << VEC_FROM_ARRAY(tail.gyr);

    const bool include_head = (pose_index == 1) || imu_pose_reset_before_[pose_index - 1];
    while (point_index >= 0)
    {
      PointType &point = pcl_out.points[static_cast<size_t>(point_index)];
      const double point_time = point.curvature / 1000.0;
      const bool after_head = point_time > head.offset_time + kTimeEpsilon;
      const bool on_included_head =
          include_head && point_time >= head.offset_time - kTimeEpsilon;
      if (!after_head && !on_included_head)
      {
        break;
      }

      const double dt = point_time - head.offset_time;
      const M3D R_i(R_imu * Exp(angvel_avr, dt));
      const V3D P_i(point.x, point.y, point.z);
      const V3D T_ei(
          pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - imu_state.pos);
      const V3D P_compensate =
          imu_state.offset_R_L_I.conjugate() *
          (imu_state.rot.conjugate() *
               (R_i * (imu_state.offset_R_L_I * P_i + imu_state.offset_T_L_I) + T_ei) -
           imu_state.offset_T_L_I);

      point.x = P_compensate(0);
      point.y = P_compensate(1);
      point.z = P_compensate(2);
      --point_index;
    }
  }
}

void ImuProcess::Process(
    const MeasureGroup &meas, IkfomFilter &kf_state, PointCloudXYZI::Ptr cur_pcl_un_)
{
  static const std::vector<TimedStateUpdate> no_timed_updates;
  Process(meas, kf_state, cur_pcl_un_, no_timed_updates);
}

void ImuProcess::Process(
    const MeasureGroup &meas, IkfomFilter &kf_state, PointCloudXYZI::Ptr cur_pcl_un_,
    const std::vector<TimedStateUpdate> &timed_updates)
{
  double t1,t2,t3;
  t1 = omp_get_wtime();

  if(meas.imu.empty()) {return;};
  ROS_ASSERT(meas.lidar != nullptr);

  if (imu_need_init_)
  {
    /// The very first lidar frame
    IMU_init(meas, kf_state, init_iter_num);

    imu_need_init_ = true;
    
    last_imu_   = meas.imu.back();
    last_lidar_end_time_ = meas.lidar_end_time;

    state_ikfom imu_state = kf_state.get_x();
    if (init_iter_num > MAX_INI_COUNT)
    {
      cov_acc *= pow(G_m_s2 / mean_acc.norm(), 2);
      imu_need_init_ = false;

      cov_acc = cov_acc_scale;
      cov_gyr = cov_gyr_scale;
      ROS_INFO("IMU Initial Done");
      // ROS_INFO("IMU Initial Done: Gravity: %.4f %.4f %.4f %.4f; state.bias_g: %.4f %.4f %.4f; acc covarience: %.8f %.8f %.8f; gry covarience: %.8f %.8f %.8f",\
      //          imu_state.grav[0], imu_state.grav[1], imu_state.grav[2], mean_acc.norm(), cov_bias_gyr[0], cov_bias_gyr[1], cov_bias_gyr[2], cov_acc[0], cov_acc[1], cov_acc[2], cov_gyr[0], cov_gyr[1], cov_gyr[2]);
      fout_imu.open(DEBUG_FILE_DIR("imu.txt"),ios::out);
    }

    return;
  }

  UndistortPcl(meas, kf_state, *cur_pcl_un_, timed_updates);

  t2 = omp_get_wtime();
  t3 = omp_get_wtime();
  
  // cout<<"[ IMU Process ]: Time: "<<t3 - t1<<endl;
}
