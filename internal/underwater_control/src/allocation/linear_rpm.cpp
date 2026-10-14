// ==============================================================================
// internal/underwater_control/src/allocation/linear_rpm.cpp
// ==============================================================================
// 推力到转速的线性映射：rpm = thrust / nominal_max * max_rpm。
// - 绝对值小于 minimum 视为进入电机物理死区，返回 0 并在有非零期望时置位 deadband 标志；
// - 超出 maximum 时置位 saturated 标志并按正负极性钳位到 ±maximum。
// ==============================================================================
#include "underwater_control/allocator.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace underwater::control {

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
