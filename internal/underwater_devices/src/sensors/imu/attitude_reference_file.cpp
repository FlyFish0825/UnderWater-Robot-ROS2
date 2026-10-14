// ==============================================================================
// internal/underwater_devices/src/sensors/imu/attitude_reference_file.cpp
// ==============================================================================
// 姿态参考文件持久化实现：
// - 文本格式存储：首行魔数 "UNDERWATER_IMU_REFERENCE 1"
// - 读取时逐字段严格校验，并检查 frame_id 与系统配置一致
// - 写入采用同目录临时文件 + fsync + rename 原子替换，并加 flock 互斥锁防止并发损坏
// ==============================================================================
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
  (void)reference_attitude(r, {0, 0, 0});  // 顺带校验 target_rpy_deg 的合法性
  double norm = 0;
  for (double v : r.baseline) {
    norm += v * v;
  }

  // baseline 必须近似单位四元数 (容差 1e-6)；采样数限 10..2000；字符串长度设上限
  if (std::abs(norm - 1.0) > 1e-6 || r.samples < 10 || r.samples > 2000 ||
      r.frame_id.empty() || r.frame_id.size() > 128 ||
      r.saved_at.empty() || r.saved_at.size() > 64) {
    throw std::runtime_error("invalid calibration file fields");
  }
}

void regular_or_absent(const fs::path &p) {
  const auto status = fs::symlink_status(p);
  if (fs::exists(status) && !fs::is_regular_file(status)) {
    throw std::runtime_error("calibration path must be a regular file");
  }
}

}  // namespace

std::optional<AttitudeReference> load_attitude_reference(
    const std::string &path,
    const std::string &frame_id) {
  if (path.empty() || frame_id.empty()) {
    throw std::invalid_argument("path and frame_id must be non-empty");
  }

  const fs::path file_path(path);
  regular_or_absent(file_path);

  if (!fs::exists(file_path)) {
    return std::nullopt;
  }

  std::ifstream file(file_path);
  if (!file.is_open()) {
    throw std::runtime_error("cannot open attitude reference file for reading");
  }

  std::string line;
  if (!std::getline(file, line) || line != "UNDERWATER_IMU_REFERENCE 1") {
    throw std::runtime_error("invalid attitude reference file header");
  }

  AttitudeReference ref;
  std::string key;

  while (file >> key) {
    if (key == "frame_id") {
      file >> ref.frame_id;
    } else if (key == "saved_at") {
      file >> ref.saved_at;
    } else if (key == "samples") {
      file >> ref.samples;
    } else if (key == "baseline") {
      for (int i = 0; i < 4; ++i) {
        file >> ref.baseline[i];
      }
    } else if (key == "target_rpy_deg") {
      for (int i = 0; i < 3; ++i) {
        file >> ref.target_rpy_deg[i];
      }
    }
  }

  if (ref.frame_id != frame_id) {
    throw std::runtime_error(
        "attitude reference frame_id mismatch: file has " + ref.frame_id + ", expected " + frame_id);
  }

  validate(ref);
  return ref;
}

void save_attitude_reference(const std::string &path, const AttitudeReference &reference) {
  validate(reference);

  const fs::path target_path(path);
  regular_or_absent(target_path);

  const auto parent_dir = target_path.parent_path();
  if (!parent_dir.empty() && !fs::exists(parent_dir)) {
    fs::create_directories(parent_dir);
  }

  // 使用同目录临时文件原子重命名写入
  const fs::path temp_path = target_path.string() + ".tmp";

  int fd = ::open(temp_path.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
  if (fd < 0) {
    throw std::runtime_error("cannot create temporary calibration file: " + std::string(std::strerror(errno)));
  }

  // 加文件锁
  if (flock(fd, LOCK_EX) != 0) {
    ::close(fd);
    throw std::runtime_error("flock failed on calibration file");
  }

  std::ostringstream ss;
  ss << std::fixed << std::setprecision(10);
  ss << "UNDERWATER_IMU_REFERENCE 1\n"
     << "frame_id " << reference.frame_id << "\n"
     << "saved_at " << reference.saved_at << "\n"
     << "samples " << reference.samples << "\n"
     << "baseline "
     << reference.baseline[0] << " " << reference.baseline[1] << " "
     << reference.baseline[2] << " " << reference.baseline[3] << "\n"
     << "target_rpy_deg "
     << reference.target_rpy_deg[0] << " " << reference.target_rpy_deg[1] << " "
     << reference.target_rpy_deg[2] << "\n";

  const std::string data = ss.str();
  if (::write(fd, data.data(), data.size()) != static_cast<ssize_t>(data.size())) {
    ::close(fd);
    fs::remove(temp_path);
    throw std::runtime_error("write failed on calibration file");
  }

  // 刷盘保证物理持久化
  if (::fsync(fd) != 0) {
    ::close(fd);
    fs::remove(temp_path);
    throw std::runtime_error("fsync failed on calibration file");
  }

  ::close(fd);

  // 原子替换目标文件
  fs::rename(temp_path, target_path);
}

void clear_attitude_reference(const std::string &path) {
  const fs::path p(path);
  if (fs::exists(p)) {
    fs::remove(p);
  }
}

}  // namespace underwater::devices
