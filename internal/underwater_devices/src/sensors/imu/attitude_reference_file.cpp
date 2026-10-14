// ==============================================================================
// internal/underwater_devices/src/sensors/imu/attitude_reference_file.cpp
// ==============================================================================
// 姿态参考文件持久化实现：
// - 文本格式存储：首行魔数 "UNDERWATER_IMU_REFERENCE 1"
// - 读取时逐字段严格校验，并检查 frame_id 与系统配置一致
// - 写入采用同目录临时文件 + fsync + rename 原子替换，并加 flock 互斥锁防止并发损坏
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

// 给 std::filesystem 起别名，方便书写路径。
namespace fs = std::filesystem;

// 匿名命名空间：本文件私有的辅助函数。
namespace {

// validate()：检查一份参考数据是否自洽（目标角合法、基准近似单位四元数、字段范围合理）。
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

// 只允许普通文件或“不存在”，防止把软链接/目录当作标定文件。
void regular_or_absent(const fs::path &p) {
  const auto status = fs::symlink_status(p);
  if (fs::exists(status) && !fs::is_regular_file(status)) {
    throw std::runtime_error("calibration path must be a regular file");
  }
}

}  // namespace

// load_attitude_reference()：读取并解析文件；文件不存在返回 nullopt，解析失败抛异常。
// frame_id 必须与调用方一致，否则拒绝。
std::optional<AttitudeReference> load_attitude_reference(
    const std::string &path,
    const std::string &frame_id) {
  if (path.empty() || frame_id.empty()) {
    throw std::invalid_argument("path and frame_id must be non-empty");
  }

  // 先检查路径类型，避免读到非普通文件。
  const fs::path file_path(path);
  regular_or_absent(file_path);

  if (!fs::exists(file_path)) {
    return std::nullopt;
  }

  std::ifstream file(file_path);
  if (!file.is_open()) {
    throw std::runtime_error("cannot open attitude reference file for reading");
  }

  // 第一行必须是魔数，用来识别文件格式与版本。
  std::string line;
  if (!std::getline(file, line) || line != "UNDERWATER_IMU_REFERENCE 1") {
    throw std::runtime_error("invalid attitude reference file header");
  }

  AttitudeReference ref;
  std::string key;

  // 逐字段读取 key value 形式的文本。
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

// save_attitude_reference()：把参考数据原子地写入文件。
void save_attitude_reference(const std::string &path, const AttitudeReference &reference) {
  validate(reference);

  const fs::path target_path(path);
  regular_or_absent(target_path);

  const auto parent_dir = target_path.parent_path();
  if (!parent_dir.empty() && !fs::exists(parent_dir)) {
    fs::create_directories(parent_dir);
  }

  // 临时文件与目标同目录，确保 rename 在同一文件系统上，从而具备原子性。
  // 使用同目录临时文件原子重命名写入
  const fs::path temp_path = target_path.string() + ".tmp";

  int fd = ::open(temp_path.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
  if (fd < 0) {
    throw std::runtime_error("cannot create temporary calibration file: " + std::string(std::strerror(errno)));
  }

  // flock 排他锁：同一时刻只允许一个进程写该文件。
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

  // fsync 强制刷盘：返回成功才认为数据已经持久化。
  // 刷盘保证物理持久化
  if (::fsync(fd) != 0) {
    ::close(fd);
    fs::remove(temp_path);
    throw std::runtime_error("fsync failed on calibration file");
  }

  ::close(fd);

  // rename 原子替换：即使此前中途崩溃，原目标文件内容也保持不变。
  // 原子替换目标文件
  fs::rename(temp_path, target_path);
}

// clear_attitude_reference()：删除标定文件；文件不存在也不报错（幂等）。
void clear_attitude_reference(const std::string &path) {
  const fs::path p(path);
  if (fs::exists(p)) {
    fs::remove(p);
  }
}

}  // namespace underwater::devices
