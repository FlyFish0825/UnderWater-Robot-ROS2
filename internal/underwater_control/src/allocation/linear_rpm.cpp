#include "underwater_control/allocator.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace underwater::control {
std::int16_t linear_rpm(double force,double nominal,int minimum,int maximum,bool & deadband,bool & saturated) {
  if(!std::isfinite(force)||!std::isfinite(nominal)||nominal<=0 ||
     minimum<1000 || maximum>6000 || minimum>maximum)
    throw std::invalid_argument("invalid temporary RPM curve");
  const double rpm=force/nominal*maximum;
  if(std::abs(rpm)<minimum) { deadband|=std::abs(rpm)>1e-6; return 0; }
  saturated|=std::abs(rpm)>maximum;
  return static_cast<std::int16_t>(std::lround(std::clamp(rpm,-double(maximum),double(maximum))));
}
}
