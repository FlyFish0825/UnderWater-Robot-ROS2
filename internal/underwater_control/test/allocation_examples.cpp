// ==============================================================================
// internal/underwater_control/test/allocation_examples.cpp
// ==============================================================================
// 推力分配典型案例输出与验证工具：
// 演示物理 wrench 模式与手动档位模式下，不同轴向输入求解出的八路电机转速 (RPM)
// ==============================================================================
#include "underwater_control/allocator.hpp"

#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

using namespace underwater::control;

template <class T>
void print_array(const T &values) {
  std::cout << '[';
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i) {
      std::cout << ',';
    }
    std::cout << values[i];
  }
  std::cout << ']';
}

void example(const char *mode, int axis, double value) {
  Allocator allocator(Config{});
  Result r;
  Wrench input{};
  input[axis] = value;

  const bool manual = (std::string(mode) == "manual");
  const bool ok = manual ? allocator.solve_manual(input, r) : allocator.solve(input, r);
  if (!ok) {
    throw std::runtime_error("example failed");
  }

  std::cout << std::fixed << std::setprecision(2);
  std::cout << mode << " axis " << axis << " = " << value << " -> rpm ";
  print_array(r.rpm);
  std::cout << "\n";
}

int main() {
  try {
    // 物理力/力矩测试案例 (NED 坐标系：0:Fx, 1:Fy, 2:Fz, 5:Mz)
    example("physical", 0, 100.0);
    example("physical", 1, 100.0);
    example("physical", 2, 100.0);
    example("physical", 5, 50.0);

    // 手动档位测试案例
    example("manual", 0, 50.0);
    example("manual", 1, 50.0);
    example("manual", 2, 50.0);
    example("manual", 5, 50.0);
  } catch (const std::exception &e) {
    std::cerr << "Error: " << e.what() << "\n";
    return 1;
  }
  return 0;
}
