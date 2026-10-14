// 姿态参考（attitude reference）纯计算与采集接口：四元数工具、相对姿态换算，
// 以及与 ROS 无关的静止姿态采集状态机 ReferenceCapture。
// 约定：四元数顺序为 w,x,y,z；角度默认弧度，带 _deg 后缀的为度。
// 采集完成后必须先成功持久化，调用方才能启用新基准。
// 初学者导读（underwater_devices / attitude_reference.hpp）
// 所属：IMU 姿态参考标定。devices 节点订阅 gateway/frames，把 0x81 姿态帧交给
//   ImuCalibration，本文件提供其中的数学与静止采集逻辑。
// 谁调用谁：imu_calibration.hpp/cpp 包含本头文件并使用 ReferenceCapture 与
//   reference_attitude；测试 attitude_reference_test.cpp 直接调用这些纯函数。
// 本文件是纯 C++（不含 ROS），因此可以脱离节点单独单元测试；ROS 服务/话题的封装在
//   旁边的 imu_calibration.* 中。
// C++ 概念：struct 是默认 public 的数据集合；std::array<double,4> 是固定长度数组；
//   std::optional<T> 表示“可能有、也可能没有 T”（例如未标定时的基准）；
//   explicit 构造函数禁止隐式类型转换；成员函数后的 const 表示调用它不会修改对象。
#pragma once
#include <array>
#include <cstdint>
#include <optional>
#include <string>

namespace underwater::devices {
using Vector3 = std::array<double, 3>;  // 三分量向量，单位随上下文
using Quaternion = std::array<double, 4>;  // w, x, y, z
// 四元数：用 4 个数 (w,x,y,z) 表示三维旋转，可避免欧拉角的万向节死锁问题。
// 以下四元数函数均按 wxyz 约定；输入非法（NaN 或范数过小）时抛 std::invalid_argument。
Quaternion rpy_quaternion(const Vector3 &rpy_rad);
Quaternion quaternion_product(const Quaternion &a, const Quaternion &b);
Quaternion quaternion_inverse(const Quaternion &q);
Quaternion normalized_quaternion(const Quaternion &q);
Vector3 quaternion_rpy(const Quaternion &q);
double quaternion_distance(const Quaternion &a, const Quaternion &b);  // 两姿态夹角(rad)，已处理 q 与 -q 等价

// AttitudeReference：一次标定产出的“基准姿态”记录，可以被保存到文件。
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

// CaptureLimits：判定“机器人是否静止、采集是否有效”的阈值集合。
// 静止采集判据：角速度上限、姿态离散度上限、原始数据超时、最短采集时长。
struct CaptureLimits {
  double gyro_limit_rad_s = 0.05;  // 判定“静止”的角速度模长上限 (rad/s)
  double spread_limit_deg = 1.0;  // 采集期间姿态偏离首个样本的上限 (deg)
  double raw_timeout_s = 0.5;  // 原始陀螺数据的最大陈旧时间 (s)
  double min_duration_s = 1.0;  // 采集最短持续时间 (s)
};

// ReferenceCapture：与 ROS 无关的静止采集状态机。它按“原始角速度帧 + 姿态帧”
// 两个输入流推进；调用方喂 start/raw/attitude/tick，完成后用 take_candidate() 取结果。
// 所有时间戳都由调用方以单调秒给出，这样它内部不需要时钟、也不会阻塞 ROS 回调。
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
