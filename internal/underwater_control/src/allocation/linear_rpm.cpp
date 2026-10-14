// ==============================================================================
// internal/underwater_control/src/allocation/linear_rpm.cpp
// ==============================================================================
// 推力到转速的线性映射：rpm = thrust / nominal_max * max_rpm。
// - 绝对值小于 minimum 视为进入电机物理死区，返回 0 并在有非零期望时置位 deadband 标志；
// - 超出 maximum 时置位 saturated 标志并按正负极性钳位到 ±maximum。
// ==============================================================================
// ------------------------------------------------------------------------------
// 【初学者导读】本文件实现单个推进器的“推力→转速”线性折算函数 linear_rpm。
// 所属：internal/underwater_control 的分配子模块；被 allocator.cpp 的 finish() 调用。
// 不涉及 ROS 2 话题或节点，是纯数学函数。
// 名词白话：RPM＝每分钟转数；死区(deadband)＝转速太低电机其实不转，只能置零；
//           饱和(saturated)＝想要的转速超过电机上限，只能削顶到最大值。
// C++ 概念：引用参数 bool & 让函数能“回传”额外结果；std::int16_t 是 16 位整数，
//           足够存 ±6000 这种量级；std::lround 做四舍五入。
// ------------------------------------------------------------------------------
#include "underwater_control/allocator.hpp"

// algorithm 提供 std::clamp 钳位；cmath 提供绝对值/四舍五入；stdexcept 提供异常。
#include <algorithm>
#include <cmath>
#include <stdexcept>

// 实现 underwater::control 中声明的 linear_rpm。
namespace underwater::control {

// linear_rpm：force 为单桨推力(N)，nominal 为额定推力(N)，minimum/maximum 为 RPM 边界。
// deadband/saturated 是“累积式”标志：一旦某通道触发就被置 true，由调用者负责清零。
std::int16_t linear_rpm(
    double force,
    double nominal,
    int minimum,
    int maximum,
    bool &deadband,
    bool &saturated) {
  // 校验参数有效性及转速物理边界
  if (!std::isfinite(force) || !std::isfinite(nominal) || nominal <= 0.0 ||
      minimum < 1000 || maximum > 6000 || minimum > maximum) {
    throw std::invalid_argument("invalid temporary RPM curve parameters");
  }

  // 理想线性换算
  const double rpm = force / nominal * maximum;

  // 死区判定：低于最低转速门限时停转置零
  if (std::abs(rpm) < minimum) {
    deadband |= (std::abs(rpm) > 1e-6);  // 大于 1e-6 的非零微小请求标记为死区损失
    return 0;
  }

  // 饱和检测与数值钳位
  saturated |= (std::abs(rpm) > maximum);
  const double clamped_rpm = std::clamp(rpm, -static_cast<double>(maximum), static_cast<double>(maximum));

  // 四舍五入为整型转速
  return static_cast<std::int16_t>(std::lround(clamped_rpm));
}

}  // namespace underwater::control
