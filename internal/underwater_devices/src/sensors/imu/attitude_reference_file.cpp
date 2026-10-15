// ==============================================================================
// internal/underwater_devices/src/sensors/imu/attitude_reference_file.cpp
// ==============================================================================
// 姿态参考文件的持久化：文本格式，首行 "UNDERWATER_IMU_REFERENCE 1"。
// 读取时逐字段校验并检查 frame_id 与当前 imu_frame_id 一致，失败即抛异常。
// 写入采用同目录临时文件 + fsync + rename 的原子替换，并用 flock 防止并发写坏文件。
// ==============================================================================
// 初学者导读（underwater_devices / sensors/imu/attitude_reference_file.cpp）
// 所属：姿态参考的“存盘/读盘”实现，纯 C++ + POSIX 文件 API，没有 ROS。
// 谁调用谁：imu_calibration.cpp 在标定完成时调用 save_attitude_reference()，
//   启动时调用 load_attitude_reference()，清除标定时调用 clear_attitude_reference()。
// 安全设计：写入采用“临时文件 + fsync + rename”原子替换，并加 flock 文件锁，
//   避免掉电或并发写导致标定文件只写了一半而损坏。
// C++/系统概念：std::filesystem 用于路径与文件操作；O_WRONLY/O_CREAT 等是 open() 的
//   系统调用标志；fsync 把数据真正刷入磁盘；同目录 rename 是原子操作。
#include "underwater_devices/attitude_reference.hpp"

#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <sys/file.h>
#include <unistd.h>

namespace underwater::devices {

namespace fs = std::filesystem;

namespace {

void validate(const AttitudeReference &r) {
  (void)reference_attitude(r, {0, 0, 0});  // 顺带校验 target_rpy_deg 是否在合法角度范围内
  double norm = 0;
  for (double v : r.baseline) {
    norm += v * v;
  }
  // baseline 必须近似单位四元数(容差 1e-6)；采样数限 10..2000；字符串长度设上限。
  if (std::abs(norm - 1.0) > 1e-6 || r.samples < 10 || r.samples > 2000 ||
      r.frame_id.empty() || r.frame_id.size() > 128 ||
      r.saved_at.empty() || r.saved_at.size() > 64) {
    throw std::runtime_error("invalid calibration file fields");
  }
}

void regular_or_absent(const fs::path &p) {
  const auto status = fs::symlink_status(p);
  if (fs::exists(status) && !fs::is_regular_file(status)) {
    // 拒绝目录与符号链接，防止被诱导写到别处
    throw std::runtime_error(
        "calibration path must be a regular file, not a directory or symlink");
  }
}

struct FileLock {
  int fd = -1;
  explicit FileLock(const std::string &path) {
    // <path>.lock 以 0600 创建并用 LOCK_EX|LOCK_NB 排他；已被占用则直接失败。
    fd = ::open((path + ".lock").c_str(), O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW, 0600);
    if (fd < 0) {
      throw std::runtime_error("cannot open calibration lock: " + std::string(std::strerror(errno)));
    }
    if (::flock(fd, LOCK_EX | LOCK_NB) != 0) {
      ::close(fd);
      fd = -1;
      throw std::runtime_error("calibration file is busy");
    }
  }
  ~FileLock() {
    if (fd >= 0) {
      ::close(fd);
    }
  }
};

}  // namespace

std::optional<AttitudeReference> load_attitude_reference(
    const std::string &path,
    const std::string &frame) {
  regular_or_absent(path);
  if (!fs::exists(path)) {
    return std::nullopt;  // 文件不存在表示尚未标定，不算错误
  }
  if (fs::file_size(path) > 4096) {
    throw std::runtime_error("calibration file is too large");  // 超过 4 KiB 视为损坏
  }
  std::ifstream in(path);
  if (!in) {
    throw std::runtime_error("cannot read calibration file");
  }

  std::string key;
  std::string source;
  int version = 0;
  AttitudeReference r;

  if (!(in >> key >> version) || key != "UNDERWATER_IMU_REFERENCE" || version != 1) {
    throw std::runtime_error("unsupported calibration file version");  // 仅接受 version=1
  }
  if (!(in >> key >> std::quoted(source)) || key != "source" || source != "euler_zyx_relative") {
    throw std::runtime_error("unsupported calibration orientation convention");  // 仅接受 ZYX 相对欧拉角约定
  }
  if (!(in >> key >> std::quoted(r.frame_id)) || key != "frame") {
    throw std::runtime_error("missing calibration frame");
  }
  if (!(in >> key >> r.baseline[0] >> r.baseline[1] >> r.baseline[2] >> r.baseline[3]) ||
      key != "baseline_wxyz") {
    throw std::runtime_error("invalid calibration baseline");  // 基准四元数按 w x y z 顺序
  }
  if (!(in >> key >> r.target_rpy_deg[0] >> r.target_rpy_deg[1] >> r.target_rpy_deg[2]) ||
      key != "target_rpy_deg") {
    throw std::runtime_error("invalid calibration target");  // 目标角，单位度
  }
  if (!(in >> key >> r.samples) || key != "samples") {
    throw std::runtime_error("invalid calibration sample count");
  }
  if (!(in >> key >> std::quoted(r.saved_at)) || key != "saved_at") {
    throw std::runtime_error("missing calibration date");
  }
  if (in >> key) {
    throw std::runtime_error("unexpected trailing calibration data");  // 多出的字段视为文件被篡改
  }

  validate(r);
  if (r.frame_id != frame) {
    throw std::runtime_error("saved calibration frame differs from imu_frame_id");  // 坐标系不匹配则拒绝加载
  }
  return r;
}

void save_attitude_reference(const std::string &path, const AttitudeReference &r) {
  validate(r);
  const fs::path p(path);
  if (!p.is_absolute()) {
    throw std::invalid_argument("calibration path must be absolute");
  }
  fs::create_directories(p.parent_path());
  FileLock lock(path);
  regular_or_absent(p);

  std::ostringstream out;
  out << std::setprecision(17);  // 17 位有效数字保证 double 往返无损
  out << "UNDERWATER_IMU_REFERENCE 1\nsource \"euler_zyx_relative\"\nframe "
      << std::quoted(r.frame_id) << "\nbaseline_wxyz";
  for (double v : r.baseline) {
    out << ' ' << v;
  }
  out << "\ntarget_rpy_deg";
  for (double v : r.target_rpy_deg) {
    out << ' ' << v;
  }
  out << "\nsamples " << r.samples << "\nsaved_at " << std::quoted(r.saved_at) << '\n';

  std::string temp = path + ".tmp.XXXXXX";
  int fd = ::mkstemp(temp.data());
  if (fd < 0) {
    throw std::runtime_error("cannot create calibration temporary file");
  }
  try {
    const auto data = out.str();
    std::size_t offset = 0;
    while (offset < data.size()) {
      const auto n = ::write(fd, data.data() + offset, data.size() - offset);
      if (n < 0 && errno == EINTR) {
        continue;  // 被信号中断则重试
      }
      if (n <= 0) {
        throw std::runtime_error("calibration write failed");
      }
      offset += static_cast<std::size_t>(n);
    }
    if (::fsync(fd) != 0) {
      throw std::runtime_error("calibration flush failed");  // 落盘后再改名，断电不会留下半截文件
    }
    const int result = ::close(fd);
    fd = -1;
    if (result != 0) {
      throw std::runtime_error("calibration close failed");
    }
    // Same-directory POSIX rename: failure leaves the previous active file untouched.
    fs::rename(temp, p);
  } catch (...) {
    if (fd >= 0) {
      ::close(fd);
    }
    ::unlink(temp.c_str());
    throw;  // 失败时清理临时文件，旧文件保持有效
  }
}

void clear_attitude_reference(const std::string &path) {
  const fs::path p(path);
  if (!p.is_absolute()) {
    throw std::invalid_argument("calibration path must be absolute");
  }
  if (!fs::exists(p.parent_path())) {
    return;  // 目录不存在说明本来就没有参考文件
  }
  FileLock lock(path);
  regular_or_absent(p);
  fs::remove(p);
}

}  // namespace underwater::devices
