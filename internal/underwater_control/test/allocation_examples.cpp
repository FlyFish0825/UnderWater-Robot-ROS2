#include "underwater_control/allocator.hpp"
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
using namespace underwater::control;
template<class T> void print_array(const T & values) {
  std::cout<<'[';
  for(std::size_t i=0;i<values.size();++i) {if(i)std::cout<<',';std::cout<<values[i];}
  std::cout<<']';
}
void example(const char * mode,int axis,double value) {
  Allocator allocator(Config{});Result r;Wrench input{};input[axis]=value;
  const bool manual=std::string(mode)=="manual";
  if(!(manual?allocator.solve_manual(input,r):allocator.solve(input,r)))
    throw std::runtime_error("example solve failed");
  std::cout<<"{\"mode\":\""<<mode<<"\",\"axis\":"<<axis<<",\"value\":"<<value<<",\"rpm\":";
  print_array(r.rpm);std::cout<<",\"thrust_n\":";print_array(r.thrust);
  std::cout<<",\"effective_request\":";print_array(r.effective_request);
  std::cout<<",\"modeled_wrench\":";print_array(r.modeled_rpm);
  std::cout<<",\"deadband\":"<<(r.deadband?"true":"false")<<"}\n";
}
int main() {
  try {
    std::cout<<std::setprecision(10);
    for(double level:{1.0,10.0})for(int axis=0;axis<6;++axis)for(int sign:{1,-1})
      example("manual",axis,sign*level);
    for(int axis=0;axis<6;++axis)for(int sign:{1,-1})
      example("physical",axis,sign*(axis<3?500.0:axis==5?150.0:50.0));
  } catch(const std::exception & e) {std::cerr<<e.what()<<'\n';return 1;}
}
