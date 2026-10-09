#include "underwater_control/allocator.hpp"
#include <current_robot_8thruster_config.hpp>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <algorithm>
using namespace underwater::control;
int checks=0;
void check(bool value) { ++checks; if(!value) throw std::runtime_error("check "+std::to_string(checks)); }
// Independent physical fixture: URDF positions plus operator-confirmed positive-RPM thrust.
Wrench physical_wrench(const std::array<std::int16_t,8> & rpm) {
  const double diagonal=std::sqrt(0.5);
  const std::array<Vec3,8> positive_rpm_axes{{
    {-diagonal,-diagonal,0},{diagonal,-diagonal,0},
    {-diagonal,diagonal,0},{diagonal,diagonal,0},
    {0,0,1},{0,0,1},{0,0,-1},{0,0,-1}}};
  Wrench w{};
  Vec3 center{};
  for(const auto & t:current_rov::kThrusters) {
    center[0]+=t.position_m.x/8.0;center[1]+=t.position_m.y/8.0;center[2]+=t.position_m.z/8.0;
  }
  for(std::size_t i=0;i<8;++i) {
    const auto & position=current_rov::kThrusters[i].position_m;
    const Vec3 p{position.x-center[0],position.y-center[1],position.z-center[2]};
    const auto & a=positive_rpm_axes[i];
    const double force=double(rpm[i])/12.0; // Existing provisional 6000 RPM <-> 500 N model.
    for(int j=0;j<3;++j) w[j]+=a[j]*force;
    w[3]+=(p[1]*a[2]-p[2]*a[1])*force;
    w[4]+=(p[2]*a[0]-p[0]*a[2])*force;
    w[5]+=(p[0]*a[1]-p[1]*a[0])*force;
  }
  return w;
}
void check_physical_mapping(const Result & result) {
  const auto w=physical_wrench(result.rpm);
  for(int j=0;j<6;++j) check(std::abs(w[j]-result.modeled_rpm[j])<1e-6);
}
int main() {
 try {
  Config cfg; Allocator allocator(cfg); Result r;
  check(std::abs(allocator.wrench_origin()[0]+0.012873)<1e-12);
  check(std::abs(allocator.wrench_origin()[1])<1e-12);
  check(std::abs(allocator.wrench_origin()[2]+0.040532)<1e-12);
  for(std::size_t i=0;i<8;++i) {
    const auto & p=current_rov::kThrusters[i].position_m;
    check(cfg.positions[i]==Vec3{p.x,p.y,p.z});
  }
  for(int axis=0;axis<6;++axis) for(int sign:{-1,1}) {
    Wrench w{}; w[axis]=sign*(axis<3?100.0:10.0);
    check(allocator.solve(w,r));
    for(int j=0;j<6;++j) check(std::abs(r.allocated[j]-w[j])<0.1);
    for(auto rpm:r.rpm) check(rpm==0 || (std::abs(int(rpm))>=1000 && std::abs(int(rpm))<=6000));
    check_physical_mapping(r);
  }
  check(allocator.solve({0,0,400,0,0,0},r));
  for(int i=0;i<4;++i) check(r.rpm[i]==0);
  for(int i=4;i<6;++i) check(r.rpm[i]>=1000 && r.rpm[i]<=6000);
  for(int i=6;i<8;++i) check(r.rpm[i]<=-1000 && r.rpm[i]>=-6000);
  check_physical_mapping(r);
  std::cout<<"Fz=400 proposed RPM:";
  for(auto rpm:r.rpm) std::cout<<" "<<rpm; std::cout<<"\n";
  check(allocator.solve({0,0,500,0,0,0},r));
  for(int i=0;i<4;++i) check(r.rpm[i]==0);
  for(int i=4;i<6;++i) check(r.rpm[i]>=1000 && r.rpm[i]<=6000);
  for(int i=6;i<8;++i) check(r.rpm[i]<=-1000 && r.rpm[i]>=-6000);
  check_physical_mapping(r);
  std::cout<<"Fz=500 proposed RPM:";
  for(auto rpm:r.rpm) std::cout<<" "<<rpm; std::cout<<"\n";
  check(allocator.solve({0,0,50,0,0,0},r));
  for(auto rpm:r.rpm) check(rpm==0);
  check(r.deadband);
  check(allocator.solve({1e6,-1e6,1e6,0,0,0},r));
  for(auto rpm:r.rpm) check(std::abs(int(rpm))<=6000);
  check(allocator.solve({},r)); for(auto rpm:r.rpm) check(rpm==0);
  Wrench bad{}; bad[0]=std::numeric_limits<double>::quiet_NaN(); check(!allocator.solve(bad,r));
  bool dead=false,sat=false;
  check(linear_rpm(500,500,1000,6000,dead,sat)==6000);
  check(linear_rpm(-500,500,1000,6000,dead,sat)==-6000);
  check(linear_rpm(80,500,1000,6000,dead,sat)==0 && dead);
  check(linear_rpm(100,500,1000,6000,dead,sat)==1200);
  check(linear_rpm(600,500,1000,6000,dead,sat)==6000 && sat);
  cfg.motor_direction[4]=-1; cfg.node_ids={8,7,6,5,4,3,2,1};
  Allocator mapped(cfg); check(mapped.solve({0,0,400,0,0,0},r)); check(r.rpm[3]<=-1000);
  Config manual_cfg; Allocator manual(manual_cfg);
  const std::array<int,6> expected_axis_signs{-1,1,1,-1,1,-1};
  const std::array<std::array<int,8>,6> expected_motor_signs{{
    {1,-1,1,-1,0,0,0,0},{-1,-1,1,1,0,0,0,0},{0,0,0,0,1,1,-1,-1},
    {0,0,0,0,1,-1,-1,1},{0,0,0,0,-1,-1,-1,-1},{1,1,1,1,0,0,0,0}}};
  for(int axis=0;axis<6;++axis) for(int sign:{-1,1}) {
    Wrench levels{};levels[axis]=sign;
    check(manual.solve_manual(levels,r));
    int running=0,peak=0;
    for(std::size_t i=0;i<8;++i) {
      const int rpm=r.rpm[i];if(rpm)++running;peak=std::max(peak,std::abs(rpm));
      check(rpm==0||(std::abs(rpm)>=1000&&std::abs(rpm)<=6000));
      if(expected_motor_signs[axis][i])check(sign*expected_motor_signs[axis][i]*rpm>0);
    }
    check(running==4);check(peak==1200);check(!r.deadband);
    for(std::size_t i=0;i<8;++i)if(!expected_motor_signs[axis][i])check(r.rpm[i]==0);
    check_physical_mapping(r);
    const auto actual=physical_wrench(r.rpm);
    check(expected_axis_signs[axis]*sign*actual[axis]>0);
    check(expected_axis_signs[axis]*sign*r.effective_request[axis]>0);
    for(int j=0;j<6;++j) if(j!=axis)check(std::abs(actual[j])<(j<3?0.5:0.1));
    std::cout<<"manual axis="<<axis<<" sign="<<sign<<" RPM:";for(auto rpm:r.rpm)std::cout<<" "<<rpm;
    std::cout<<" modeled:";for(auto value:actual)std::cout<<" "<<value;std::cout<<"\n";
  }
  int previous=0;
  for(int level=1;level<=100;++level) {
    check(manual.solve_manual({double(level),0,0,0,0,0},r));
    int peak=0;for(auto rpm:r.rpm){peak=std::max(peak,std::abs(int(rpm)));check(rpm==0||(std::abs(int(rpm))>=1000&&std::abs(int(rpm))<=6000));}
    check(peak>=previous);previous=peak;
  }
  check(previous==6000);
  check(manual.solve_manual({},r));for(auto rpm:r.rpm)check(rpm==0);
  check(manual.solve_manual({1e-8,0,0,0,0,0},r));for(auto rpm:r.rpm)check(rpm==0);
  check(!manual.solve_manual({101,0,0,0,0,0},r));
  check(!manual.solve_manual(bad,r));
  check(manual.solve_manual({0,0,1,1,0,0},r));check_physical_mapping(r);check(!r.deadband);
  for(auto rpm:r.rpm)check(rpm==0||(std::abs(int(rpm))>=1000&&std::abs(int(rpm))<=6000));
  // A direction whose weakest required channel cannot clear deadband below 6000 must stop.
  check(!manual.solve_manual({1,-1,1,-1,1,-1},r));for(auto rpm:r.rpm)check(rpm==0);
  check(manual.solve({1,0,0,0,0,0},r));for(auto rpm:r.rpm)check(rpm==0);
  // Operator directions follow the explicit motor sign table; physical NED remains unchanged.
  check(manual.solve_manual({0,0,10,0,0,0},r));
  check(r.rpm[4]>0 && r.rpm[5]>0 && r.rpm[6]<0 && r.rpm[7]<0);
  check(r.modeled_rpm[2]>0);check_physical_mapping(r);
  check(manual.solve({0,0,500,0,0,0},r));check(r.modeled_rpm[2]>0);
  Config original_manual;original_manual.manual_axis_signs.fill(1);
  Allocator original(original_manual);
  for(int axis=0;axis<6;++axis) {
    Wrench levels{};levels[axis]=10;
    check(original.solve_manual(levels,r));check(r.modeled_rpm[axis]>0);
  }
  Config invalid_manual;invalid_manual.manual_axis_signs[2]=0;
  bool rejected=false;try { Allocator invalid(invalid_manual); } catch(const std::invalid_argument &) { rejected=true; }
  check(rejected);
  // Translating the CAD origin must not change allocation about the thruster centroid.
  Config translated;
  for(auto & p:translated.positions) {p[0]+=2;p[1]-=3;p[2]+=0.7;}
  Allocator shifted(translated);check(shifted.solve_manual({0,0,1,0,0,0},r));
  check(r.rpm==std::array<std::int16_t,8>{0,0,0,0,1200,1200,-1200,-1200});
  check_physical_mapping(r);
  std::cout<<"PASS "<<checks<<" allocation checks\n"; return 0;
 } catch(const std::exception & e) { std::cerr<<e.what()<<"\n"; return 1; }
}
