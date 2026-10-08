#pragma once
#include <ThrustAllocationSolver.hpp>
#include <array>
#include <cstdint>
namespace underwater::control {
using Vec3=std::array<double,3>;
using Wrench=std::array<double,6>;
struct Config {
  // Authoritative geometry: F:/file/BaiduSyncdisk/Project/OSQP/main.cpp.
  std::array<Vec3,8> positions{{
    {0.23016,-0.18359,-0.040532},{0.23016,0.18359,-0.040532},
    {-0.25591,-0.18359,-0.040532},{-0.25591,0.18359,-0.040532},
    {0.25239,-0.26163,0},{0.25239,0.26163,0},
    {-0.27087,-0.26163,0},{-0.27087,0.26163,0}}};
  std::array<Vec3,8> axes{{
    {-0.70711,-0.70711,0},{-0.70711,0.70711,0},
    {0.70711,-0.70711,0},{0.70711,0.70711,0},
    {0,0,1},{0,0,1},{0,0,1},{0,0,1}}};
  std::array<double,8> nominal_max_thrust_n{500,500,500,500,500,500,500,500};
  std::array<int,8> node_ids{1,2,3,4,5,6,7,8};
  std::array<int,8> motor_direction{1,1,1,1,1,1,1,1};
  Wrench weights{1,1,1,1,1,1};
  double effort_weight=1e-6;
  int min_rpm=1000,max_rpm=6000;
  int manual_start_rpm=1200;
};
struct Result {
  std::array<double,8> thrust{};
  std::array<std::int16_t,8> rpm{}; // CAN node order.
  Wrench allocated{},modeled_rpm{},effective_request{};
  bool deadband=false,saturated=false;
};
std::int16_t linear_rpm(double thrust_n,double nominal_max_n,int min_rpm,int max_rpm,
  bool & deadband,bool & saturated);
class Allocator {
public:
  explicit Allocator(Config config);
  bool solve(const Wrench &,Result &);
  // Keyboard levels in [-100,100], separate from the physical-wrench API.
  bool solve_manual(const Wrench & levels,Result &);
  const Config & config() const { return config_; }
private:
  Wrench wrench_of(const std::array<double,8> &) const;
  void finish(Result &) const;
  Config config_;
  ThrustAllocationSolver solver_;
};
}
