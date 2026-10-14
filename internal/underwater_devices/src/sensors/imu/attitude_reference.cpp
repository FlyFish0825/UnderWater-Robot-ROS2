// ==============================================================================
// internal/underwater_devices/src/sensors/imu/attitude_reference.cpp
// ==============================================================================
// 姿态参考数学运算与采集状态机实现：
// - 四元数归一化、共轭逆、乘法与 RPY 互转 (右手 ZYX 顺序)
// - 相对基准参考姿态求解 (reference_attitude)
// - 静止姿态采集状态机 (ReferenceCapture)：角速度模长门限与姿态离散度双重判据
// ==============================================================================
#include "underwater_devices/attitude_reference.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace underwater::devices {

namespace {

constexpr double pi = 3.14159265358979323846;
constexpr double radians = pi / 180.0;  // 角度转弧度系数

template <std::size_t N>
bool finite(const std::array<double, N> &a) {
  return std::all_of(a.begin(), a.end(), [](double x) { return std::isfinite(x); });
}

double dot(const Quaternion &a, const Quaternion &b) {
  double value = 0;
  for (std::size_t i = 0; i < 4; ++i) {
    value += a[i] * b[i];
  }
  return value;
}

void validate_target(const Vector3 &target) {
  // 目标以度为单位：roll/yaw ±180°、pitch ±90°，越界或非有限值一律拒绝
  if (!finite(target) || std::abs(target[0]) > 180 || std::abs(target[1]) > 90 ||
      std::abs(target[2]) > 180) {
    throw std::invalid_argument("target RPY must be finite degrees: roll/yaw +/-180, pitch +/-90");
  }
}

}  // namespace

// 四元数归一化：范数下限 1e-12，避免除零产生坏解
Quaternion normalized_quaternion(const Quaternion &q) {
  const double norm = std::sqrt(dot(q, q));
  if (!finite(q) || !std::isfinite(norm) || norm < 1e-12) {
    throw std::invalid_argument("invalid quaternion for normalization");
  }
  return {q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm};
}

// 四元数逆运算：q⁻¹ = [w, -x, -y, -z] / |q|²
Quaternion quaternion_inverse(const Quaternion &q) {
  const double norm_sq = dot(q, q);
  if (!finite(q) || !std::isfinite(norm_sq) || norm_sq < 1e-12) {
    throw std::invalid_argument("invalid quaternion for inversion");
  }
  return {q[0] / norm_sq, -q[1] / norm_sq, -q[2] / norm_sq, -q[3] / norm_sq};
}

// 四元数乘法：Hamilton 约定 (a * b)
Quaternion quaternion_product(const Quaternion &a, const Quaternion &b) {
  if (!finite(a) || !finite(b)) {
    throw std::invalid_argument("invalid quaternion for product");
  }
  return {
      a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
      a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
      a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
      a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]};
}

// 欧拉角 (roll, pitch, yaw) 转四元数 (右手 ZYX 旋转顺序：Rz(yaw) * Ry(pitch) * Rx(roll))
Quaternion rpy_quaternion(const Vector3 &rpy) {
  if (!finite(rpy)) {
    throw std::invalid_argument("invalid RPY for quaternion conversion");
  }
  const double cr = std::cos(rpy[0] * 0.5);
  const double sr = std::sin(rpy[0] * 0.5);
  const double cp = std::cos(rpy[1] * 0.5);
  const double sp = std::sin(rpy[1] * 0.5);
  const double cy = std::cos(rpy[2] * 0.5);
  const double sy = std::sin(rpy[2] * 0.5);

  return {
      cr * cp * cy + sr * sp * sy,
      sr * cp * cy - cr * sp * sy,
      cr * sp * cy + sr * cp * sy,
      cr * cp * sy - sr * sp * cy};
}

// 四元数转欧拉角 (roll, pitch, yaw)：单位为弧度
Vector3 quaternion_rpy(const Quaternion &q) {
  const auto n = normalized_quaternion(q);
  const double w = n[0], x = n[1], y = n[2], z = n[3];

  // roll (x 轴旋转)
  const double sinr_cosp = 2.0 * (w * x + y * z);
  const double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
  const double roll = std::atan2(sinr_cosp, cosr_cosp);

  // pitch (y 轴旋转，带 ±1 奇异点钳位)
  const double sinp = 2.0 * (w * y - z * x);
  const double pitch = (std::abs(sinp) >= 1.0) ? std::copysign(pi / 2.0, sinp)
                                               : std::asin(sinp);

  // yaw (z 轴旋转)
  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  const double yaw = std::atan2(siny_cosp, cosy_cosp);

  return {roll, pitch, yaw};
}

// 计算两姿态四元数之间的角距离 (弧度，范围 [0, π])，自动处理 q 与 -q 双重覆盖
double quaternion_distance(const Quaternion &a, const Quaternion &b) {
  const auto na = normalized_quaternion(a);
  const auto nb = normalized_quaternion(b);
  const double d = std::clamp(std::abs(dot(na, nb)), 0.0, 1.0);
  return 2.0 * std::acos(d);
}

// 计算相对基准姿态：q_rel = q_target * (q_baseline⁻¹ * q_current)
Quaternion reference_attitude(const AttitudeReference &reference, const Vector3 &rpy_rad) {
  validate_target(reference.target_rpy_deg);
  const Vector3 target_rad{
      reference.target_rpy_deg[0] * radians,
      reference.target_rpy_deg[1] * radians,
      reference.target_rpy_deg[2] * radians};

  const auto q_target = rpy_quaternion(target_rad);
  const auto q_current = rpy_quaternion(rpy_rad);
  const auto q_baseline_inv = quaternion_inverse(reference.baseline);

  // 姿态增量 Delta = q_baseline⁻¹ * q_current
  const auto delta = quaternion_product(q_baseline_inv, q_current);
  return normalized_quaternion(quaternion_product(q_target, delta));
}

ReferenceCapture::ReferenceCapture(CaptureLimits limits)
    : limits_(limits) {}

void ReferenceCapture::start(
    const Vector3 &target_deg,
    std::uint32_t samples,
    double timeout_s,
    double now) {
  validate_target(target_deg);
  if (samples < 10 || samples > 2000 || !std::isfinite(timeout_s) || timeout_s < 1.0 || timeout_s > 60.0) {
    throw std::invalid_argument("invalid capture parameters: samples must be 10..2000, timeout 1..60s");
  }

  busy_ = true;
  requested_ = samples;
  deadline_ = now + timeout_s;
  count_ = 0;
  sum_ = {0, 0, 0, 0};
  target_ = target_deg;
  candidate_.reset();
  connection_.reset();
  detail_ = "capturing attitude reference; hold the robot stationary";
}

bool ReferenceCapture::accept_connection(std::uint64_t connection) {
  if (!connection_) {
    connection_ = connection;
    return true;
  }
  if (*connection_ != connection) {
    cancel("serial connection changed during capture");
    return false;
  }
  return true;
}

void ReferenceCapture::raw(const Vector3 &gyro, bool valid, std::uint64_t connection, double now) {
  if (!busy_ || !accept_connection(connection)) {
    return;
  }
  raw_valid_ = valid && finite(gyro);
  raw_connection_ = connection;
  raw_at_ = now;
  gyro_norm_ = raw_valid_ ? std::sqrt(gyro[0] * gyro[0] + gyro[1] * gyro[1] + gyro[2] * gyro[2])
                          : 0.0;
}

void ReferenceCapture::attitude(
    const Vector3 &rpy,
    bool valid,
    std::uint64_t connection,
    const std::string &frame_id,
    double now) {
  if (!busy_ || !accept_connection(connection)) {
    return;
  }
  if (frame_id.empty()) {
    cancel("missing frame_id in attitude sample");
    return;
  }
  if (!frame_id_.empty() && frame_id_ != frame_id) {
    cancel("frame_id changed during capture");
    return;
  }
  frame_id_ = frame_id;

  // 陀螺仪角速度有效性与静止门限检查
  if (!raw_valid_ || now - raw_at_ > limits_.raw_timeout_s) {
    cancel("raw gyro data unavailable or stale during capture");
    return;
  }
  if (gyro_norm_ > limits_.gyro_limit_rad_s) {
    cancel("motion detected during calibration: angular velocity exceeded limit");
    return;
  }
  if (!valid || !finite(rpy)) {
    cancel("invalid or non-finite attitude sample received");
    return;
  }

  const auto q = rpy_quaternion(rpy);

  // 首个样本作为基准点
  if (count_ == 0) {
    first_ = q;
  } else {
    // 检查姿态样本离散度
    const double distance_deg = quaternion_distance(first_, q) / radians;
    if (distance_deg > limits_.spread_limit_deg) {
      cancel("attitude spread exceeded limit: robot moved during capture");
      return;
    }
  }

  // 四元数平均：对齐符号避免正负半球对冲
  const double sign = (dot(first_, q) >= 0.0) ? 1.0 : -1.0;
  for (std::size_t i = 0; i < 4; ++i) {
    sum_[i] += sign * q[i];
  }

  ++count_;
  last_sample_at_ = now;

  // 达到目标采样数，生成候选结果
  if (count_ >= requested_) {
    busy_ = false;
    AttitudeReference candidate;
    candidate.baseline = normalized_quaternion(sum_);
    candidate.target_rpy_deg = target_;
    candidate.frame_id = frame_id_;
    candidate.samples = count_;
    candidate_ = candidate;
    detail_ = "attitude reference capture completed successfully";
  }
}

void ReferenceCapture::tick(double now) {
  if (busy_ && now >= deadline_) {
    cancel("attitude reference capture timed out");
  }
}

void ReferenceCapture::cancel(const std::string &reason) {
  if (busy_) {
    busy_ = false;
    detail_ = reason;
    candidate_.reset();
    count_ = 0;
  }
}

std::optional<AttitudeReference> ReferenceCapture::take_candidate() {
  auto candidate = candidate_;
  candidate_.reset();
  return candidate;
}

}  // namespace underwater::devices
