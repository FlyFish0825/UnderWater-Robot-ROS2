// 推力到转速的线性映射：rpm = thrust/nominal_max*max_rpm。
// 绝对值小于 minimum 视为死区，返回 0 且有非零请求时置 deadband；
// 超出 maximum 时置 saturated 并钳位到 ±maximum。
#include "underwater_control/allocator.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace underwater::control {
std::int16_t linear_rpm(double force,double nominal,int minimum,int maximum,bool & deadband,bool & saturated) {
  if(!std::isfinite(force)||!std::isfinite(nominal)||nominal<=0 ||
     minimum<1000 || maximum>6000 || minimum>maximum)  // 与分配器一致的 RPM 合法区间
    throw std::invalid_argument("invalid temporary RPM curve");
  const double rpm=force/nominal*maximum;
  if(std::abs(rpm)<minimum) { deadband|=std::abs(rpm)>1e-6; return 0; }  // 死区内停转；1e-6 以下视为零请求，不算死区
  saturated|=std::abs(rpm)>maximum;
  return static_cast<std::int16_t>(std::lround(std::clamp(rpm,-double(maximum),double(maximum))));  // 四舍五入并钳位
}
}
