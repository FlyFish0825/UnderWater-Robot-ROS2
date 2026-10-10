// 姿态参考（attitude reference）纯计算与采集接口：四元数工具、相对姿态换算，
// 以及与 ROS 无关的静止姿态采集状态机 ReferenceCapture。
// 约定：四元数顺序为 w,x,y,z；角度默认弧度，带 _deg 后缀的为度。
// 采集完成后必须先成功持久化，调用方才能启用新基准。
#pragma once
#include <array>
#include <cstdint>
#include <optional>
#include <string>

namespace underwater::devices {
using Vector3 = std::array<double, 3>;  // 三分量向量，单位随上下文
using Quaternion = std::array<double, 4>;  // w, x, y, z
// 以下四元数函数均按 wxyz 约定；输入非法（NaN 或范数过小）时抛 std::invalid_argument。
Quaternion rpy_quaternion(const Vector3 &rpy_rad);
Quaternion quaternion_product(const Quaternion &a, const Quaternion &b);
Quaternion quaternion_inverse(const Quaternion &q);
Quaternion normalized_quaternion(const Quaternion &q);
Vector3 quaternion_rpy(const Quaternion &q);
double quaternion_distance(const Quaternion &a, const Quaternion &b);  // 两姿态夹角(rad)，已处理 q 与 -q 等价

// 已保存的静止基准：baseline 为采集期间的平均姿态四元数，
// target_rpy_deg 声明该基准位姿在用户参考系中的期望角度（度），用于相对姿态换算。
struct AttitudeReference {
  Quaternion baseline{1, 0, 0, 0};
  Vector3 target_rpy_deg{};
  std::string frame_id;
  std::string saved_at;  // UTC ISO-8601 字符串
  std::uint32_t samples = 0;
};
Quaternion reference_attitude(const AttitudeReference &reference, const Vector3 &rpy_rad);  // 由当前 RPY 求相对基准的参考姿态
// 参考文件读写：load 校验失败时抛异常，save 用同目录临时文件+fsync+rename 原子替换。
std::optional<AttitudeReference> load_attitude_reference(const std::string &path,
                                                       const std::string &frame_id);
void save_attitude_reference(const std::string &path, const AttitudeReference &reference);
void clear_attitude_reference(const std::string &path);

// 静止采集判据：角速度上限、姿态离散度上限、原始数据超时、最短采集时长。
struct CaptureLimits {
  double gyro_limit_rad_s = 0.05;  // 判定“静止”的角速度模长上限 (rad/s)
  double spread_limit_deg = 1.0;  // 采集期间姿态偏离首个样本的上限 (deg)
  double raw_timeout_s = 0.5;  // 原始陀螺数据的最大陈旧时间 (s)
  double min_duration_s = 1.0;  // 采集最短持续时间 (s)
};

// Pure C++ collector. Caller supplies monotonic seconds; never blocks a ROS callback.
// Completion produces a candidate; the caller must save successfully before activating it.
class ReferenceCapture {
public:
  explicit ReferenceCapture(CaptureLimits limits = {});
  void start(const Vector3 &target_deg, std::uint32_t samples, double timeout_s, double now);  // now 为调用方的单调秒
  void raw(const Vector3 &gyro, bool valid, std::uint64_t connection, double now);  // 原始角速度样本，单位 rad/s
  void attitude(const Vector3 &rpy, bool valid, std::uint64_t connection,
                const std::string &frame_id, double now);  // 姿态样本，单位 rad
  void tick(double now);  // 超时检查，需由调用方周期性驱动
  void cancel(const std::string &reason);
  bool busy() const { return busy_; }
  std::uint32_t count() const { return count_; }
  std::uint32_t requested() const { return requested_; }
  const std::string &detail() const { return detail_; }
  std::optional<AttitudeReference> take_candidate();  // 取出候选并清空；未完成时返回 nullopt
private:
  bool accept_connection(std::uint64_t connection);  // 连接 ID 变化即取消本次采集
  CaptureLimits limits_;
  bool busy_ = false;
  double deadline_ = 0, last_sample_at_ = 0, raw_at_ = 0;  // 均为单调秒
  bool raw_valid_ = false;
  std::uint64_t raw_connection_ = 0;
  double gyro_norm_ = 0;  // 最近一次角速度模长 (rad/s)
  std::optional<std::uint64_t> connection_;
  std::uint32_t count_ = 0, requested_ = 0;
  Quaternion first_{}, sum_{};  // sum_ 为符号对齐后的四元数累加，用于平均值
  Vector3 target_{};
  std::string frame_id_, detail_ = "idle";
  std::optional<AttitudeReference> candidate_;
};
}  // namespace underwater::devices
