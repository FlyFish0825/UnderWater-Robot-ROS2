#include "underwater_control/allocator.hpp"
#include <cmath>
#include <stdexcept>
#include <utility>
#include <algorithm>
#include <limits>
namespace underwater::control {
Allocator::Allocator(Config c):config_(std::move(c)) {
  if(config_.min_rpm<1000 || config_.max_rpm>6000 || config_.min_rpm>config_.max_rpm)
    throw std::invalid_argument("RPM bounds must be within 1000..6000");
  if(config_.manual_start_rpm<config_.min_rpm || config_.manual_start_rpm>config_.max_rpm)
    throw std::invalid_argument("manual_start_rpm must be within min_rpm..max_rpm");
  for(int sign:config_.manual_axis_signs)
    if(sign!=1 && sign!=-1) throw std::invalid_argument("manual_axis_signs must be +/-1");
  for(const auto & position:config_.positions)for(int j=0;j<3;++j) {
    if(!std::isfinite(position[j]))throw std::invalid_argument("invalid thruster position");
    wrench_origin_[j]+=position[j]/8.0;
  }
  unsigned mask=0;
  for(std::size_t i=0;i<8;++i) {
    const int id=config_.node_ids[i],direction=config_.motor_direction[i];
    if(id<1 || id>8 || (mask&(1u<<(id-1))) || (direction!=1 && direction!=-1))
      throw std::invalid_argument("node_ids must be a permutation of 1..8 and directions must be +/-1");
    mask|=1u<<(id-1);
    auto & a=config_.axes[i]; const auto & p=config_.positions[i];
    const double norm=std::hypot(a[0],a[1],a[2]);
    if(!std::isfinite(norm)||norm<1e-9) throw std::invalid_argument("invalid thruster axis");
    for(auto & x:a) x/=norm;
    if(!solver_.addThruster({p[0]-wrench_origin_[0],p[1]-wrench_origin_[1],p[2]-wrench_origin_[2]},
        {a[0],a[1],a[2]},config_.nominal_max_thrust_n[i]))
      throw std::invalid_argument("invalid thruster geometry or nominal force");
  }
  if(!solver_.setWrenchWeights(config_.weights) || !solver_.setEffortWeight(config_.effort_weight) ||
     !solver_.initialize()) throw std::runtime_error("thrust allocator initialization failed");
}
Wrench Allocator::wrench_of(const std::array<double,8> & force) const {
  Wrench w{};
  for(std::size_t i=0;i<8;++i) {
    const auto & a=config_.axes[i];const auto & position=config_.positions[i];
    const Vec3 p{position[0]-wrench_origin_[0],position[1]-wrench_origin_[1],position[2]-wrench_origin_[2]};
    for(int j=0;j<3;++j) w[j]+=a[j]*force[i];
    w[3]+=(p[1]*a[2]-p[2]*a[1])*force[i];
    w[4]+=(p[2]*a[0]-p[0]*a[2])*force[i];
    w[5]+=(p[0]*a[1]-p[1]*a[0])*force[i];
  }
  return w;
}
bool Allocator::solve(const Wrench & desired,Result & result) {
  result={};
  for(double v:desired) if(!std::isfinite(v)||std::abs(v)>1e6) return false;
  result.effective_request=desired;
  // Exact zero is an exact stop, without retaining solver warm-start residue.
  bool zero=true; for(double v:desired) zero&=v==0.0;
  if(zero) return true;
  if(!solver_.solveThrust(desired,result.thrust)) return false;
  finish(result);
  return true;
}
bool Allocator::solve_manual(const Wrench & levels,Result & result) {
  result={};
  double level=0;
  for(double v:levels) {
    if(!std::isfinite(v)||std::abs(v)>100) return false;
    level=std::max(level,std::abs(v));
  }
  // Values below the first keyboard step are stop, including numerical noise.
  if(level<1) return true;
  Wrench direction{};
  for(int j=0;j<6;++j)direction[j]=config_.manual_axis_signs[j]*levels[j]/level;
  if(!solver_.solveThrust(direction,result.thrust)) return false;
  double peak=0;
  for(std::size_t i=0;i<8;++i)
    peak=std::max(peak,std::abs(result.thrust[i])/config_.nominal_max_thrust_n[i]*config_.max_rpm);
  if(!std::isfinite(peak)||peak<1e-9) {result={};return false;}
  // Remove only solver noise. Required small compensation keeps its original ratio.
  constexpr double noise_ratio=1e-5;
  double minimum=std::numeric_limits<double>::infinity();
  for(std::size_t i=0;i<8;++i) {
    const double rpm=std::abs(result.thrust[i])/config_.nominal_max_thrust_n[i]*config_.max_rpm;
    if(!std::isfinite(rpm)) {result={};return false;}
    if(rpm<peak*noise_ratio) result.thrust[i]=0;
    else minimum=std::min(minimum,rpm);
  }
  // One RPM of margin avoids losing a required channel at the deadband boundary.
  const double start_peak=std::max(double(config_.manual_start_rpm),
    (config_.min_rpm+1.0)*peak/minimum);
  // Never silently clip one channel or discard compensation to fit the speed range.
  if(!std::isfinite(start_peak)||start_peak>config_.max_rpm) {result={};return false;}
  const double target=start_peak+(config_.max_rpm-start_peak)*(level-1)/99.0;
  const double gain=target/peak;
  for(auto & f:result.thrust)f*=gain;
  for(int j=0;j<6;++j)result.effective_request[j]=direction[j]*gain;
  // The same gain keeps all required force/torque ratios and clears the RPM deadband.
  finish(result);
  return true;
}
void Allocator::finish(Result & result) const {
  std::array<double,8> modeled{};
  for(std::size_t i=0;i<8;++i) {
    const auto raw=linear_rpm(result.thrust[i],config_.nominal_max_thrust_n[i],
      config_.min_rpm,config_.max_rpm,result.deadband,result.saturated);
    result.rpm[config_.node_ids[i]-1]=raw*config_.motor_direction[i];
    modeled[i]=double(raw)/config_.max_rpm*config_.nominal_max_thrust_n[i];
  }
  result.allocated=wrench_of(result.thrust); result.modeled_rpm=wrench_of(modeled);
}
}
