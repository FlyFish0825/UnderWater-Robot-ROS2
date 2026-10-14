// ==============================================================================
// internal/underwater_control/src/allocation/allocator.cpp
// ==============================================================================
// 推力分配实现：把期望六维 wrench 交给 ThrustAllocationSolver 求解八桨推力，
// 再按线性 RPM 曲线量化成各 CAN 节点的目标转速，并给出死区/饱和标志。
// 构造阶段会校验几何、极性与 RPM 边界；求解前拒绝 NaN/超大输入，避免输出无意义指令。
// ==============================================================================
#include "underwater_control/allocator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace underwater::control {

Allocator::Allocator(Config c)
    : config_(std::move(c)) {
  // 校验电机 RPM 的物理能力范围 (必须在 1000..6000 之间)
  if (config_.min_rpm < 1000 || config_.max_rpm > 6000 || config_.min_rpm > config_.max_rpm) {
    throw std::invalid_argument("RPM bounds must be within 1000..6000");
  }

  // 手动起步转速必须位于合法转速区间内
  if (config_.manual_start_rpm < config_.min_rpm || config_.manual_start_rpm > config_.max_rpm) {
    throw std::invalid_argument("manual_start_rpm must be within min_rpm..max_rpm");
  }

  // 校验手动输入轴符号必须为 +1 或 -1
  for (int sign : config_.manual_axis_signs) {
    if (sign != 1 && sign != -1) {
      throw std::invalid_argument("manual_axis_signs must be +/-1");
    }
  }

  // 计算八个推进器安装位置的三维几何中心（质心参考点）
  for (const auto &position : config_.positions) {
    for (int j = 0; j < 3; ++j) {
      if (!std::isfinite(position[j])) {
        throw std::invalid_argument("invalid thruster position");
      }
      wrench_origin_[j] += position[j] / 8.0;  // 力矩参考点取八个安装点的算术平均
    }
  }

  unsigned mask = 0;
  for (std::size_t i = 0; i < 8; ++i) {
    const int id = config_.node_ids[i];
    const int direction = config_.motor_direction[i];

    // node_ids 必须是 1..8 的全排列，防止两桨映射到同一 CAN 节点
    if (id < 1 || id > 8 || (mask & (1u << (id - 1))) || (direction != 1 && direction != -1)) {
      throw std::invalid_argument(
          "node_ids must be a permutation of 1..8 and directions must be +/-1");
    }
    mask |= (1u << (id - 1));

    auto &a = config_.axes[i];
    const auto &p = config_.positions[i];

    // 推进器方向向量归一化
    const double norm = std::hypot(a[0], a[1], a[2]);
    if (!std::isfinite(norm) || norm < 1e-9) {
      throw std::invalid_argument("invalid thruster axis");
    }
    for (auto &x : a) {
      x /= norm;  // 轴向归一化：求解器按 单位方向 × 推力 计算力与力矩
    }

    // 向二次规划求解器注册各推进器几何位姿（使用相对几何质心的相对坐标）
    if (!solver_.addThruster(
            {p[0] - wrench_origin_[0], p[1] - wrench_origin_[1], p[2] - wrench_origin_[2]},
            {a[0], a[1], a[2]},
            config_.nominal_max_thrust_n[i])) {
      throw std::invalid_argument("invalid thruster geometry or nominal force");
    }
  }

  // 配置权重并初始化 OSQP / QDLDL 二次规划求解器
  if (!solver_.setWrenchWeights(config_.weights) ||
      !solver_.setEffortWeight(config_.effort_weight) ||
      !solver_.initialize()) {
    throw std::runtime_error("thrust allocator initialization failed");
  }
}

Wrench Allocator::wrench_of(const std::array<double, 8> &force) const {
  Wrench w{};
  for (std::size_t i = 0; i < 8; ++i) {
    const auto &a = config_.axes[i];
    const auto &position = config_.positions[i];
    const Vec3 p{
        position[0] - wrench_origin_[0],
        position[1] - wrench_origin_[1],
        position[2] - wrench_origin_[2]};

    // 线性合力：F = sum(轴向单位向量 * 推力)
    for (int j = 0; j < 3; ++j) {
      w[j] += a[j] * force[i];
    }

    // 合力矩：M = r × F，相对几何质心进行叉乘计算
    w[3] += (p[1] * a[2] - p[2] * a[1]) * force[i];
    w[4] += (p[2] * a[0] - p[0] * a[2]) * force[i];
    w[5] += (p[0] * a[1] - p[1] * a[0]) * force[i];
  }
  return w;
}

bool Allocator::solve(const Wrench &desired, Result &result) {
  result = {};

  // 校验输入数值的合法性：拒绝 NaN、无穷大或超大异常数值
  for (double v : desired) {
    if (!std::isfinite(v) || std::abs(v) > 1e6) {
      return false;
    }
  }
  result.effective_request = desired;

  // 全零请求直接作为明确的停机指令输出，不保留求解器热启动残差
  bool zero = true;
  for (double v : desired) {
    zero &= (v == 0.0);
  }
  if (zero) {
    return true;
  }

  // 执行二次规划求解
  if (!solver_.solveThrust(desired, result.thrust)) {
    return false;
  }

  finish(result);
  return true;
}

bool Allocator::solve_manual(const Wrench &levels, Result &result) {
  result = {};
  double level = 0.0;

  // 校验手动档位范围在 [-100, 100] 之间
  for (double v : levels) {
    if (!std::isfinite(v) || std::abs(v) > 100.0) {
      return false;
    }
    level = std::max(level, std::abs(v));
  }

  // 低于第一档 (1.0) 视为机械停机（过滤浮点噪声与死区抖动）
  if (level < 1.0) {
    return true;
  }

  // 按最大档位幅值归一化为单位方向向量，保持各轴之间的运动协调比例
  Wrench direction{};
  for (int j = 0; j < 6; ++j) {
    direction[j] = config_.manual_axis_signs[j] * levels[j] / level;
  }

  // 二次规划求解方向向量的推力分布
  if (!solver_.solveThrust(direction, result.thrust)) {
    return false;
  }

  // 计算推进器通道换算后的最大 RPM 峰值
  double peak = 0.0;
  for (std::size_t i = 0; i < 8; ++i) {
    peak = std::max(
        peak,
        std::abs(result.thrust[i]) / config_.nominal_max_thrust_n[i] * config_.max_rpm);
  }
  if (!std::isfinite(peak) || peak < 1e-9) {
    result = {};
    return false;
  }

  // 滤除求解器数值噪声：低于峰值 1e-5 的微弱通道直接置零，关键补偿通道保留其原有比例
  constexpr double noise_ratio = 1e-5;
  double minimum = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < 8; ++i) {
    const double rpm =
        std::abs(result.thrust[i]) / config_.nominal_max_thrust_n[i] * config_.max_rpm;
    if (!std::isfinite(rpm)) {
      result = {};
      return false;
    }
    if (rpm < peak * noise_ratio) {
      result.thrust[i] = 0.0;
    } else {
      minimum = std::min(minimum, rpm);  // 记录参与工作的最小有效通道 RPM
    }
  }

  // 计算起跳峰值：保证最小通道也能越过电机机械死区 (+1 RPM 安全余量)
  const double start_peak = std::max(
      static_cast<double>(config_.manual_start_rpm),
      (config_.min_rpm + 1.0) * peak / minimum);

  // 若起跳峰值超限，则安全拒绝
  if (!std::isfinite(start_peak) || start_peak > config_.max_rpm) {
    result = {};
    return false;
  }

  // 线性映射：档位 1 对应 start_peak，档位 100 对应 max_rpm
  const double target = start_peak + (config_.max_rpm - start_peak) * (level - 1.0) / 99.0;
  const double gain = target / peak;

  for (auto &f : result.thrust) {
    f *= gain;
  }
  for (int j = 0; j < 6; ++j) {
    result.effective_request[j] = direction[j] * gain;
  }

  finish(result);
  return true;
}

void Allocator::finish(Result &result) const {
  std::array<double, 8> modeled{};

  for (std::size_t i = 0; i < 8; ++i) {
    // 线性映射推力到电机转速
    const auto raw = linear_rpm(
        result.thrust[i],
        config_.nominal_max_thrust_n[i],
        config_.min_rpm,
        config_.max_rpm,
        result.deadband,
        result.saturated);

    // 按 CAN 节点号映射到输出数组，并施加电机转向符号修正
    result.rpm[config_.node_ids[i] - 1] = raw * config_.motor_direction[i];

    // 反向建模死区/饱和截断后的实际输出等效推力
    modeled[i] = static_cast<double>(raw) / config_.max_rpm * config_.nominal_max_thrust_n[i];
  }

  // 计算实际求解推力合力与量化截断后的等效物理力矩
  result.allocated = wrench_of(result.thrust);
  result.modeled_rpm = wrench_of(modeled);
}

}  // namespace underwater::control
