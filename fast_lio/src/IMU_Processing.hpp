#include <algorithm>
#include <cmath>
#include <math.h>
#include <deque>
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

const bool time_list(PointType &x, PointType &y) {return (x.curvature < y.curvature);};

/// *************IMU Process and undistortion
class ImuProcess
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

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
  // 配置基于静止重力的安装倾角估计及初始化数据验收阈值。
  void set_mount_alignment(bool enabled, double duration, double expected_acc_norm,
                           double acc_norm_tolerance, double max_gyr_norm,
                           double max_acc_std, double max_gyr_std,
                           double max_mount_deg);
  Eigen::Matrix<double, 12, 12> Q;
  void Process(const MeasureGroup &meas,  esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI::Ptr pcl_un_);

  ofstream fout_imu;
  V3D cov_acc;
  V3D cov_gyr;
  V3D cov_acc_scale;
  V3D cov_gyr_scale;
  V3D cov_bias_gyr;
  V3D cov_bias_acc;
  double first_lidar_time;
  int lidar_type;

  // R_B_I：雷达/IMU坐标系 I 到水平无人机机体坐标系 B 的固定旋转。
  Quaterniond mount_tilt = Quaterniond::Identity();
  bool gravity_align_mount_en = true;
  // 静止窗口通过验收后置位，定位节点发布结果后清除；重新初始化后可再次发布。
  bool mount_alignment_pending = false;

 private:
  void IMU_init(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, int &N);
  void UndistortPcl(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI &pcl_in_out);
  // 验收失败时丢弃当前静止窗口，从最新 IMU 时刻重新累计。
  void reject_init_window(const MeasureGroup &meas);

  PointCloudXYZI::Ptr cur_pcl_un_;
  sensor_msgs::ImuConstPtr last_imu_;
  deque<sensor_msgs::ImuConstPtr> v_imu_;
  vector<Pose6D> IMUpose;
  vector<M3D>    v_rot_pcl_;
  M3D Lidar_R_wrt_IMU;
  V3D Lidar_T_wrt_IMU;
  V3D mean_acc;
  V3D mean_gyr;
  // Welford 在线方差算法的 M2 累计量，避免长时间采样时的数值消减误差。
  V3D acc_m2_;
  V3D gyr_m2_;
  V3D angvel_last;
  V3D acc_s_last = Zero3d;
  double start_timestamp_;
  // 使用 IMU 时间戳而不是处理批次数，保证初始化窗口是真实的时间长度。
  double init_start_time_ = -1.0;
  double last_lidar_end_time_ = -1.0;
  // 加速度相关阈值使用驱动发布的原始单位；MID360 当前静止模长约为 1 g。
  double imu_init_duration_ = 10.0;
  double imu_init_expected_acc_norm_ = 1.0;
  double imu_init_acc_norm_tolerance_ = 0.1;
  double imu_init_max_gyr_norm_ = 0.05;
  double imu_init_max_acc_std_ = 0.2;
  double imu_init_max_gyr_std_ = 0.02;
  // 重力自适应估计的安装倾角上限(度)：超过视为开机姿态异常(斜坡/手持)而非安装角。
  double gravity_align_max_mount_deg_ = 30.0;
  int    init_iter_num = 0;
  bool   b_first_frame_ = true;
  bool   imu_need_init_ = true;
};

ImuProcess::ImuProcess(): b_first_frame_(true), imu_need_init_(true), start_timestamp_(-1)
{
  init_iter_num = 0;
  Q = process_noise_cov();
  cov_acc       = Zero3d;
  cov_gyr       = Zero3d;
  cov_bias_gyr  = V3D(0.0001, 0.0001, 0.0001);
  cov_bias_acc  = V3D(0.0001, 0.0001, 0.0001);
  mean_acc      = Zero3d;
  mean_gyr      = V3D(0, 0, 0);
  acc_m2_       = Zero3d;
  gyr_m2_       = Zero3d;
  angvel_last     = Zero3d;
  acc_s_last      = Zero3d;
  last_lidar_end_time_ = -1.0;
  Lidar_T_wrt_IMU = Zero3d;
  Lidar_R_wrt_IMU = Eye3d;
  last_imu_.reset(new sensor_msgs::Imu());
}

ImuProcess::~ImuProcess() {}

void ImuProcess::Reset()
{
  mean_acc      = Zero3d;
  mean_gyr      = V3D(0, 0, 0);
  cov_acc       = Zero3d;
  cov_gyr       = Zero3d;
  acc_m2_       = Zero3d;
  gyr_m2_       = Zero3d;
  angvel_last       = Zero3d;
  acc_s_last        = Zero3d;
  last_lidar_end_time_ = -1.0;
  imu_need_init_    = true;
  start_timestamp_  = -1;
  init_iter_num = 0;
  init_start_time_ = -1.0;
  // 重新初始化时清除上一次保存的安装姿态，防止旧补偿继续生效。

  mount_tilt.setIdentity();
  mount_alignment_pending = false;
  v_imu_.clear();
  IMUpose.clear();
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

void ImuProcess::set_mount_alignment(bool enabled, double duration, double expected_acc_norm,
                                     double acc_norm_tolerance, double max_gyr_norm,
                                     double max_acc_std, double max_gyr_std,
                                     double max_mount_deg)
{
  gravity_align_mount_en = enabled;
  imu_init_duration_ = duration;
  imu_init_expected_acc_norm_ = expected_acc_norm;
  imu_init_acc_norm_tolerance_ = acc_norm_tolerance;
  imu_init_max_gyr_norm_ = max_gyr_norm;
  imu_init_max_acc_std_ = max_acc_std;
  imu_init_max_gyr_std_ = max_gyr_std;
  gravity_align_max_mount_deg_ = max_mount_deg;
}

void ImuProcess::IMU_init(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, int &N)
{
  /** 1. initializing the gravity, gyro bias, acc and gyro covariance
   ** 2. normalize the acceleration measurenments to unit gravity **/
  
  V3D cur_acc, cur_gyr;
  
  if (b_first_frame_)
  {
    Reset();
    b_first_frame_ = false;
    first_lidar_time = meas.lidar_beg_time;
    init_start_time_ = meas.imu.front()->header.stamp.toSec();
  }
  else if (init_start_time_ < 0.0)
  {
    // 支持运行时 Reset：下一批有效 IMU 重新建立完整初始化窗口的时间起点。
    init_start_time_ = meas.imu.front()->header.stamp.toSec();
  }

  for (const auto &imu : meas.imu)
  {
    const auto &imu_acc = imu->linear_acceleration;
    const auto &gyr_acc = imu->angular_velocity;
    cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

    // 在线累计均值和逐轴样本方差
    ++N;
    const V3D acc_delta = cur_acc - mean_acc;
    const V3D gyr_delta = cur_gyr - mean_gyr;
    mean_acc += acc_delta / N;
    mean_gyr += gyr_delta / N;
    acc_m2_ += acc_delta.cwiseProduct(cur_acc - mean_acc);
    gyr_m2_ += gyr_delta.cwiseProduct(cur_gyr - mean_gyr);
  }
  if (N > 1)
  {
    cov_acc = acc_m2_ / (N - 1.0);
    cov_gyr = gyr_m2_ / (N - 1.0);
  }
  state_ikfom init_state = kf_state.get_x();

  const double mean_acc_norm = mean_acc.norm();
  // 初始化窗口未通过最终验收前也会刷新 EKF 初值，先阻止零向量或 NaN 进入归一化。
  if (!mean_acc.allFinite() || !mean_gyr.allFinite() ||
      !std::isfinite(mean_acc_norm) || mean_acc_norm < 1e-6)
  {
    mount_tilt.setIdentity();
    last_imu_ = meas.imu.back();
    return;
  }

  if (gravity_align_mount_en)
  {
    // 由静止重力方向估计安装 roll/pitch，并用欧拉分解 R_B_I = Ry(pitch)·Rx(roll) 调平。
    // 相比 FromTwoVectors 的最小旋转，欧拉分解严格保持雷达朝向(yaw)，
    // 避免 roll 与 pitch 并存时机体 x 轴产生几度的偏航扭转。
    // 静止比力竖直向上；roll 左倾为正，pitch 前倾(下压)为正。
    const double roll_rad  = std::atan2(mean_acc.y(), mean_acc.z());
    const double pitch_rad = std::atan2(-mean_acc.x(), std::hypot(mean_acc.y(), mean_acc.z()));
    mount_tilt = Quaterniond(AngleAxisd(pitch_rad, V3D::UnitY()) *
                             AngleAxisd(roll_rad,  V3D::UnitX()));
    mount_tilt.normalize();
    init_state.rot = mount_tilt;
    init_state.grav = S2(0, 0, -G_m_s2);
  }
  else
  {
    mount_tilt.setIdentity();
    init_state.grav = S2(-mean_acc.normalized() * G_m_s2);
  }

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

void ImuProcess::UndistortPcl(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI &pcl_out)
{
  /*** add the imu of the last frame-tail to the of current frame-head ***/
  auto v_imu = meas.imu;
  v_imu.push_front(last_imu_);
  const double &imu_end_time = v_imu.back()->header.stamp.toSec();

  double pcl_beg_time = meas.lidar_beg_time;
  double pcl_end_time = meas.lidar_end_time;

  // 第一次去畸变以前没有上一帧结束时刻，以当前帧起点作为积分边界。
  if (last_lidar_end_time_ < 0.0)
  {
    last_lidar_end_time_ = pcl_beg_time;
  }

    if (lidar_type == MARSIM) {
        pcl_beg_time = last_lidar_end_time_;
        pcl_end_time = meas.lidar_beg_time;
    }

    /*** sort point clouds by offset time ***/
  pcl_out = *(meas.lidar);
  sort(pcl_out.points.begin(), pcl_out.points.end(), time_list);

  /*** Initialize IMU pose ***/
  state_ikfom imu_state = kf_state.get_x();
  IMUpose.clear();
  IMUpose.push_back(set_pose6d(0.0, acc_s_last, angvel_last, imu_state.vel, imu_state.pos, imu_state.rot.toRotationMatrix()));

  /*** forward propagation at each imu point ***/
  V3D angvel_avr, acc_avr, acc_imu, vel_imu, pos_imu;
  M3D R_imu;

  double dt = 0;

  input_ikfom in;
  // 即使异常时间戳导致积分循环没有执行，也避免末端预测读取未初始化输入。
  in.acc = mean_acc * G_m_s2 / mean_acc.norm();
  in.gyro = mean_gyr;
  for (auto it_imu = v_imu.begin(); it_imu < (v_imu.end() - 1); it_imu++)
  {
    auto &&head = *(it_imu);
    auto &&tail = *(it_imu + 1);
    
    if (tail->header.stamp.toSec() < last_lidar_end_time_)    continue;
    
    angvel_avr<<0.5 * (head->angular_velocity.x + tail->angular_velocity.x),
                0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
                0.5 * (head->angular_velocity.z + tail->angular_velocity.z);
    acc_avr   <<0.5 * (head->linear_acceleration.x + tail->linear_acceleration.x),
                0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y),
                0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z);

    // fout_imu << setw(10) << head->header.stamp.toSec() - first_lidar_time << " " << angvel_avr.transpose() << " " << acc_avr.transpose() << endl;

    acc_avr     = acc_avr * G_m_s2 / mean_acc.norm(); // - state_inout.ba;

    if(head->header.stamp.toSec() < last_lidar_end_time_)
    {
      dt = tail->header.stamp.toSec() - last_lidar_end_time_;
      // dt = tail->header.stamp.toSec() - pcl_beg_time;
    }
    else
    {
      dt = tail->header.stamp.toSec() - head->header.stamp.toSec();
    }
    
    in.acc = acc_avr;
    in.gyro = angvel_avr;
    Q.block<3, 3>(0, 0).diagonal() = cov_gyr;
    Q.block<3, 3>(3, 3).diagonal() = cov_acc;
    Q.block<3, 3>(6, 6).diagonal() = cov_bias_gyr;
    Q.block<3, 3>(9, 9).diagonal() = cov_bias_acc;
    kf_state.predict(dt, Q, in);

    /* save the poses at each IMU measurements */
    imu_state = kf_state.get_x();
    angvel_last = angvel_avr - imu_state.bg;
    acc_s_last  = imu_state.rot * (acc_avr - imu_state.ba);
    for(int i=0; i<3; i++)
    {
      acc_s_last[i] += imu_state.grav[i];
    }
    double &&offs_t = tail->header.stamp.toSec() - pcl_beg_time;
    IMUpose.push_back(set_pose6d(offs_t, acc_s_last, angvel_last, imu_state.vel, imu_state.pos, imu_state.rot.toRotationMatrix()));
  }

  /*** calculated the pos and attitude prediction at the frame-end ***/
  double note = pcl_end_time > imu_end_time ? 1.0 : -1.0;
  dt = note * (pcl_end_time - imu_end_time);
  kf_state.predict(dt, Q, in);
  
  imu_state = kf_state.get_x();
  last_imu_ = meas.imu.back();
  last_lidar_end_time_ = pcl_end_time;

  /*** undistort each lidar point (backward propagation) ***/
  if (pcl_out.points.begin() == pcl_out.points.end()) return;

  if(lidar_type != MARSIM){
      auto it_pcl = pcl_out.points.end() - 1;
      for (auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp--)
      {
          auto head = it_kp - 1;
          auto tail = it_kp;
          R_imu<<MAT_FROM_ARRAY(head->rot);
          vel_imu<<VEC_FROM_ARRAY(head->vel);
          pos_imu<<VEC_FROM_ARRAY(head->pos);
          acc_imu<<VEC_FROM_ARRAY(tail->acc);
          angvel_avr<<VEC_FROM_ARRAY(tail->gyr);

          for(; it_pcl->curvature / double(1000) > head->offset_time; it_pcl --)
          {
              dt = it_pcl->curvature / double(1000) - head->offset_time;

              /* Transform to the 'end' frame, using only the rotation
               * Note: Compensation direction is INVERSE of Frame's moving direction
               * So if we want to compensate a point at timestamp-i to the frame-e
               * P_compensate = R_imu_e ^ T * (R_i * P_i + T_ei) where T_ei is represented in global frame */
              M3D R_i(R_imu * Exp(angvel_avr, dt));

              V3D P_i(it_pcl->x, it_pcl->y, it_pcl->z);
              V3D T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - imu_state.pos);
              V3D P_compensate = imu_state.offset_R_L_I.conjugate() * (imu_state.rot.conjugate() * (R_i * (imu_state.offset_R_L_I * P_i + imu_state.offset_T_L_I) + T_ei) - imu_state.offset_T_L_I);// not accurate!

              // save Undistorted points and their rotation
              it_pcl->x = P_compensate(0);
              it_pcl->y = P_compensate(1);
              it_pcl->z = P_compensate(2);

              if (it_pcl == pcl_out.points.begin()) break;
          }
      }
  }
}

void ImuProcess::reject_init_window(const MeasureGroup &meas)
{
  init_iter_num = 0;
  mean_acc = Zero3d;
  mean_gyr = Zero3d;
  cov_acc = Zero3d;
  cov_gyr = Zero3d;
  acc_m2_ = Zero3d;
  gyr_m2_ = Zero3d;
  init_start_time_ = meas.imu.back()->header.stamp.toSec();
  mount_tilt.setIdentity();
  mount_alignment_pending = false;
}

void ImuProcess::Process(const MeasureGroup &meas,  esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI::Ptr cur_pcl_un_)
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

    // 只有累计满配置时长后才验收
    const double init_elapsed = meas.imu.back()->header.stamp.toSec() - init_start_time_;
    if (init_elapsed >= imu_init_duration_)
    {
      const V3D acc_std = cov_acc.cwiseMax(0.0).cwiseSqrt();
      const V3D gyr_std = cov_gyr.cwiseMax(0.0).cwiseSqrt();
      // 均值检测持续运动或单位异常，标准差检测触碰、振动和往复晃动。
      const bool stationary = mean_acc.allFinite() && mean_gyr.allFinite() &&
                              acc_std.allFinite() && gyr_std.allFinite() &&
                              mean_acc.norm() > 1e-6 &&
                              std::abs(mean_acc.norm() - imu_init_expected_acc_norm_) <= imu_init_acc_norm_tolerance_ &&
                              mean_gyr.norm() <= imu_init_max_gyr_norm_ &&
                              acc_std.maxCoeff() <= imu_init_max_acc_std_ &&
                              gyr_std.maxCoeff() <= imu_init_max_gyr_std_;
      if (!stationary)
      {
        ROS_WARN("IMU initialization rejected: keep still for %.1f s (|acc|=%.3f, |gyr|=%.4f, max acc std=%.4f, max gyr std=%.4f)",
                 imu_init_duration_, mean_acc.norm(), mean_gyr.norm(), acc_std.maxCoeff(), gyr_std.maxCoeff());
        // 丢弃整个不可信窗口，从当前时刻重新累计一个完整的静止窗口。
        reject_init_window(meas);
        return;
      }

      // 从验收通过的 mount_tilt 提取安装角欧拉值，用于门槛检查和日志。
      const M3D mount_rotation = mount_tilt.toRotationMatrix();
      const double mount_roll = std::atan2(mount_rotation(2, 1), mount_rotation(2, 2)) * 180.0 / M_PI;
      // 夹紧 asin 输入，避免旋转矩阵浮点误差使日志出现 NaN。
      const double pitch_sin = std::max(-1.0, std::min(1.0, -mount_rotation(2, 0)));
      const double mount_pitch = std::asin(pitch_sin) * 180.0 / M_PI;
      // 安装角合理性门槛：过大的估计倾角通常来自斜坡停放或手持开机而非真实安装角，
      // 烘焙进 mount_tilt 会让后续所有姿态持续偏差。
      if (gravity_align_mount_en &&
          (std::abs(mount_roll) > gravity_align_max_mount_deg_ ||
           std::abs(mount_pitch) > gravity_align_max_mount_deg_))
      {
        ROS_WARN("IMU initialization rejected: estimated mount RPY(deg)=[%.2f, %.2f] exceeds limit %.1f deg; "
                 "power on at nominal vehicle attitude or raise the limit",
                 mount_roll, mount_pitch, gravity_align_max_mount_deg_);
        reject_init_window(meas);
        return;
      }

      imu_need_init_ = false;
      mount_alignment_pending = true;
      // 保存初始化完成帧的结束时刻，首次正常去畸变从真实上一帧边界开始传播。
      last_lidar_end_time_ = meas.lidar_end_time;

      cov_acc = cov_acc_scale;
      cov_gyr = cov_gyr_scale;
      const double mount_yaw = std::atan2(mount_rotation(1, 0), mount_rotation(0, 0)) * 180.0 / M_PI;
      ROS_INFO("IMU initialization accepted after %.1f s: |acc|=%.3f, |gyr|=%.4f, mount RPY(deg)=[%.2f, %.2f, %.2f]",
               init_elapsed, mean_acc.norm(), mean_gyr.norm(), mount_roll, mount_pitch, mount_yaw);
      fout_imu.open(DEBUG_FILE_DIR("imu.txt"),ios::out);
    }

    return;
  }

  UndistortPcl(meas, kf_state, *cur_pcl_un_);

  t2 = omp_get_wtime();
  t3 = omp_get_wtime();
}
