#include "underwater_control/allocator.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <algorithm>
using namespace underwater::control;
int checks=0;
void check(bool value) { ++checks; if(!value) throw std::runtime_error("check "+std::to_string(checks)); }
int main() {
 try {
  Config cfg; Allocator allocator(cfg); Result r;
  check(cfg.positions[4][0]==0.25239 && cfg.positions[7][1]==0.26163);
  for(int axis=0;axis<6;++axis) for(int sign:{-1,1}) {
    Wrench w{}; w[axis]=sign*(axis<3?100.0:10.0);
    check(allocator.solve(w,r));
    for(int j=0;j<6;++j) check(std::abs(r.allocated[j]-w[j])<0.1);
    for(auto rpm:r.rpm) check(rpm==0 || (std::abs(int(rpm))>=1000 && std::abs(int(rpm))<=6000));
  }
  check(allocator.solve({0,0,400,0,0,0},r));
  for(int i=0;i<4;++i) check(r.rpm[i]==0);
  for(int i=4;i<8;++i) check(r.rpm[i]>=1000 && r.rpm[i]<=6000);
  std::cout<<"Fz=400 proposed RPM:";
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
  for(int axis=0;axis<6;++axis) for(int sign:{-1,1}) {
    Wrench levels{};levels[axis]=sign;
    check(manual.solve_manual(levels,r));
    int running=0,peak=0;
    for(auto rpm:r.rpm){if(rpm)++running;peak=std::max(peak,std::abs(int(rpm)));check(rpm==0||(std::abs(int(rpm))>=1000&&std::abs(int(rpm))<=1200));}
    check(running==4);check(peak==1200);
    std::cout<<"manual axis="<<axis<<" sign="<<sign<<" RPM:";for(auto rpm:r.rpm)std::cout<<" "<<rpm;std::cout<<"\n";
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
  check(manual.solve_manual({1,-1,1,-1,1,-1},r));
  for(auto rpm:r.rpm)check(rpm==0||(std::abs(int(rpm))>=1000&&std::abs(int(rpm))<=1200));
  check(manual.solve({1,0,0,0,0,0},r));for(auto rpm:r.rpm)check(rpm==0);
  std::cout<<"PASS "<<checks<<" allocation checks\n"; return 0;
 } catch(const std::exception & e) { std::cerr<<e.what()<<"\n"; return 1; }
}
