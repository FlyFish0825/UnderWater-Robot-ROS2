// 姿态参考的数学实现：四元数工具、相对姿态换算，以及静止采集状态机。
// 欧拉角统一采用右手 ZYX 顺序（Rz(yaw)*Ry(pitch)*Rx(roll)），输入单位为弧度。
// 采集期间用角速度模长与姿态离散度双重判据确认“静止”，样本做符号对齐后再平均。
#include "underwater_devices/attitude_reference.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace underwater::devices {
namespace {
constexpr double pi = 3.14159265358979323846;
constexpr double radians = pi / 180.0;  // 度转弧度系数
template<std::size_t N> bool finite(const std::array<double, N> &a) {
  return std::all_of(a.begin(), a.end(), [](double x) { return std::isfinite(x); });
}
double dot(const Quaternion &a, const Quaternion &b) {
  double value = 0;
  for (std::size_t i = 0; i < 4; ++i) value += a[i] * b[i];
  return value;
}
void validate_target(const Vector3 &target) {
  // 目标以度为单位：roll/yaw ±180、pitch ±90，越界或非有限值一律拒绝。
  if (!finite(target) || std::abs(target[0]) > 180 || std::abs(target[1]) > 90 ||
      std::abs(target[2]) > 180)
    throw std::invalid_argument("target RPY must be finite degrees: roll/yaw +/-180, pitch +/-90");
}
}
Quaternion normalized_quaternion(const Quaternion &q) {
  const double norm = std::sqrt(dot(q, q));
  if (!finite(q) || !std::isfinite(norm) || norm < 1e-12)  // 范数下限 1e-12，避免除零产生坏姿态
    throw std::invalid_argument("invalid quaternion");
  Quaternion out = q;
  for (auto &v : out) v /= norm;
  return out;
}
Quaternion quaternion_product(const Quaternion &a, const Quaternion &b) {
  return {a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3],
          a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2],
          a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1],
          a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0]};
}
Quaternion quaternion_inverse(const Quaternion &q) {
  auto out = normalized_quaternion(q);
  for (std::size_t i = 1; i < 4; ++i) out[i] = -out[i];  // 单位四元数的逆即共轭：虚部取反
  return out;
}
Quaternion rpy_quaternion(const Vector3 &rpy) {
  if (!finite(rpy)) throw std::invalid_argument("non-finite Euler angles");
  const double cr=std::cos(rpy[0]/2), sr=std::sin(rpy[0]/2);
  const double cp=std::cos(rpy[1]/2), sp=std::sin(rpy[1]/2);
  const double cy=std::cos(rpy[2]/2), sy=std::sin(rpy[2]/2);
  // Right-handed ZYX: Rz(yaw) * Ry(pitch) * Rx(roll).
  return {cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy,
          cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy};
}
Vector3 quaternion_rpy(const Quaternion &q) {
  const auto n=normalized_quaternion(q);
  const auto w=n[0], x=n[1], y=n[2], z=n[3];
  // 标准 ZYX 欧拉角提取，返回 roll/pitch/yaw (rad)；pitch 用 asin 并钳位避免 NaN。
  return {std::atan2(2*(w*x+y*z),1-2*(x*x+y*y)),
          std::asin(std::clamp(2*(w*y-z*x),-1.0,1.0)),
          std::atan2(2*(w*z+x*y),1-2*(y*y+z*z))};
}
double quaternion_distance(const Quaternion &a, const Quaternion &b) {
  const auto an=normalized_quaternion(a), bn=normalized_quaternion(b);
  return 2*std::acos(std::clamp(std::abs(dot(an,bn)),0.0,1.0));  // 取 |dot| 使 q 与 -q 等价，结果 0..π
}
Quaternion reference_attitude(const AttitudeReference &reference, const Vector3 &rpy) {
  // 相对姿态：Q_target * Q_baseline⁻¹ * Q_now，把当前姿态换算到以基准为原点的参考系。
  validate_target(reference.target_rpy_deg);
  auto target=reference.target_rpy_deg;
  for (auto &v : target) v *= radians;
  return normalized_quaternion(quaternion_product(rpy_quaternion(target),
    quaternion_product(quaternion_inverse(reference.baseline),rpy_quaternion(rpy))));
}
ReferenceCapture::ReferenceCapture(CaptureLimits limits) : limits_(limits) {
  // 参数范围收紧校验，防止误配置导致采集永远失败或过于宽松。
  if (!std::isfinite(limits.gyro_limit_rad_s) || limits.gyro_limit_rad_s<=0 ||
      !std::isfinite(limits.spread_limit_deg) || limits.spread_limit_deg<=0 || limits.spread_limit_deg>10 ||
      !std::isfinite(limits.raw_timeout_s) || limits.raw_timeout_s<=0 ||
      !std::isfinite(limits.min_duration_s) || limits.min_duration_s<0.5 || limits.min_duration_s>10)
    throw std::invalid_argument("invalid stationary capture limits");
}
void ReferenceCapture::start(const Vector3 &target, std::uint32_t samples, double timeout, double now) {
  if (busy_) throw std::invalid_argument("calibration already in progress");
  validate_target(target);
  // 采样数限制 10..2000；超时必须大于最短采集时长且不超过 120 s。
  if (samples<10 || samples>2000 || !std::isfinite(timeout) || timeout<=limits_.min_duration_s || timeout>120)
    throw std::invalid_argument("samples must be 10..2000; timeout must exceed capture duration and be <=120 s");
  target_=target; requested_=samples; deadline_=now+timeout; busy_=true; count_=0;
  sum_.fill(0); connection_.reset(); frame_id_.clear(); candidate_.reset();
  // Require a raw sample arriving after the explicit request.
  raw_valid_=false; detail_="collecting: keep the robot stationary";
}
bool ReferenceCapture::accept_connection(std::uint64_t connection) {
  if (connection_ && *connection_!=connection) { cancel("connection changed during calibration"); return false; }  // 采集期间串口重连即作废
  connection_=connection; return true;
}
void ReferenceCapture::raw(const Vector3 &gyro, bool valid, std::uint64_t connection, double now) {
  raw_valid_=valid && finite(gyro); raw_at_=now; raw_connection_=connection;
  gyro_norm_=raw_valid_?std::hypot(gyro[0],gyro[1],gyro[2]):0;  // 角速度模长 (rad/s)
  if (!busy_) return;
  tick(now);
  if (!busy_ || !accept_connection(connection)) return;
  if (!raw_valid_) cancel("raw IMU became invalid during calibration");
  else if (gyro_norm_>limits_.gyro_limit_rad_s) cancel("robot is rotating; keep still and retry");
}
void ReferenceCapture::attitude(const Vector3 &rpy, bool valid, std::uint64_t connection,
                               const std::string &frame, double now) {
  if (!busy_) return;
  tick(now);
  if (!busy_ || !accept_connection(connection)) return;
  if (!valid || !finite(rpy)) { cancel("attitude became invalid during calibration"); return; }
  // 姿态样本必须有同期的新鲜原始陀螺数据，否则无法确认静止。
  if (!raw_valid_ || raw_connection_!=connection || now-raw_at_>limits_.raw_timeout_s) {
    detail_="waiting for fresh raw gyro to check stationarity"; return;
  }
  if (gyro_norm_>limits_.gyro_limit_rad_s) { cancel("robot is rotating; keep still and retry"); return; }
  const auto q=rpy_quaternion(rpy);
  if (count_ && (frame!=frame_id_ || quaternion_distance(first_,q)>limits_.spread_limit_deg*radians)) {  // 离散度上限按度给出，换算成弧度比较
    cancel("attitude moved or frame changed; keep still and retry"); return;
  }
  // Space accepted samples so a buffered burst cannot constitute a stationary capture.
  if (count_ && now-last_sample_at_+1e-9<limits_.min_duration_s/(requested_-1)) return;  // 最小采样间隔 = 最短时长/(样本数-1)
  if (!count_) { first_=q; frame_id_=frame; }
  const double sign=dot(first_,q)<0?-1:1;  // 与首样本对齐符号，避免 q 与 -q 在累加时相互抵消
  for (std::size_t i=0; i<4; ++i) sum_[i]+=sign*q[i];
  ++count_; last_sample_at_=now; detail_="collecting: keep the robot stationary";
  if (count_==requested_) {
    AttitudeReference result; result.baseline=normalized_quaternion(sum_);  // 累加后归一化得到平均姿态
    result.target_rpy_deg=target_; result.frame_id=frame_id_; result.samples=count_;
    candidate_=result; busy_=false; detail_="capture complete; pending persistent save";  // 仅产出候选，等待调用方持久化
  }
}
void ReferenceCapture::tick(double now) {
  if (busy_ && now>=deadline_) cancel("calibration timed out; previous reference preserved");
}
void ReferenceCapture::cancel(const std::string &reason) {
  busy_=false; candidate_.reset(); detail_=reason;  // 取消时丢弃候选，旧基准不受影响
}
std::optional<AttitudeReference> ReferenceCapture::take_candidate() {
  auto result=candidate_; candidate_.reset(); return result;
}
}  // namespace underwater::devices
