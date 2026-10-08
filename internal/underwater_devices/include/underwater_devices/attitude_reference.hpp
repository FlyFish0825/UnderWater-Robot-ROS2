#pragma once
#include <array>
#include <cstdint>
#include <optional>
#include <string>

namespace underwater::devices {
using Vector3 = std::array<double, 3>;
using Quaternion = std::array<double, 4>;  // w, x, y, z
Quaternion rpy_quaternion(const Vector3 &rpy_rad);
Quaternion quaternion_product(const Quaternion &a, const Quaternion &b);
Quaternion quaternion_inverse(const Quaternion &q);
Quaternion normalized_quaternion(const Quaternion &q);
Vector3 quaternion_rpy(const Quaternion &q);
double quaternion_distance(const Quaternion &a, const Quaternion &b);

struct AttitudeReference {
  Quaternion baseline{1, 0, 0, 0};
  Vector3 target_rpy_deg{};
  std::string frame_id;
  std::string saved_at;
  std::uint32_t samples = 0;
};
Quaternion reference_attitude(const AttitudeReference &reference, const Vector3 &rpy_rad);
std::optional<AttitudeReference> load_attitude_reference(const std::string &path,
                                                       const std::string &frame_id);
void save_attitude_reference(const std::string &path, const AttitudeReference &reference);
void clear_attitude_reference(const std::string &path);

struct CaptureLimits {
  double gyro_limit_rad_s = 0.05;
  double spread_limit_deg = 1.0;
  double raw_timeout_s = 0.5;
  double min_duration_s = 1.0;
};

// Pure C++ collector. Caller supplies monotonic seconds; never blocks a ROS callback.
// Completion produces a candidate; the caller must save successfully before activating it.
class ReferenceCapture {
public:
  explicit ReferenceCapture(CaptureLimits limits = {});
  void start(const Vector3 &target_deg, std::uint32_t samples, double timeout_s, double now);
  void raw(const Vector3 &gyro, bool valid, std::uint64_t connection, double now);
  void attitude(const Vector3 &rpy, bool valid, std::uint64_t connection,
                const std::string &frame_id, double now);
  void tick(double now);
  void cancel(const std::string &reason);
  bool busy() const { return busy_; }
  std::uint32_t count() const { return count_; }
  std::uint32_t requested() const { return requested_; }
  const std::string &detail() const { return detail_; }
  std::optional<AttitudeReference> take_candidate();
private:
  bool accept_connection(std::uint64_t connection);
  CaptureLimits limits_;
  bool busy_ = false;
  double deadline_ = 0, last_sample_at_ = 0, raw_at_ = 0;
  bool raw_valid_ = false;
  std::uint64_t raw_connection_ = 0;
  double gyro_norm_ = 0;
  std::optional<std::uint64_t> connection_;
  std::uint32_t count_ = 0, requested_ = 0;
  Quaternion first_{}, sum_{};
  Vector3 target_{};
  std::string frame_id_, detail_ = "idle";
  std::optional<AttitudeReference> candidate_;
};
}  // namespace underwater::devices
