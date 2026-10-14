// ==============================================================================
// internal/underwater_control/test/allocation_examples.cpp
// ==============================================================================
// 推力分配典型案例输出与验证工具：
// 演示物理 wrench 模式与手动档位模式下，不同轴向输入求解出的八路电机转速 (RPM)
// ==============================================================================
// ------------------------------------------------------------------------------
// 【初学者导读】这是一个“示例输出”小程序，不属于 ROS 2 节点，也没有话题通信。
// 用途：直接调用 Allocator，演示物理 wrench 模式与手动档位模式下，
//       单轴输入会得到怎样的 8 路 RPM，便于人工对照与教学。
// 相关文件：同目录的 allocation_test.cpp 做自动断言，本文件只做打印演示。
// 涉及的 C++ 概念：
//   模板 template<class T>＝对类型参数化的函数，这里让 print_array 能打印任意定长数组；
//   std::fixed/setprecision＝把浮点数按固定小数位打印。
// ------------------------------------------------------------------------------
#include "underwater_control/allocator.hpp"

// 标准库：格式化输出、输入输出、异常、字符串。
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

// 直接使用 underwater::control 中的类型，省去命名空间前缀。
using namespace underwater::control;

// print_array：通用打印函数，把形如 std::array 的序列打印成 [a,b,c] 形式。
// 模板参数 T 可以是任意带 size() 与下标运算符的容器。
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

// example：跑一个用例。mode 为 "manual" 时走手动档位接口，否则走物理 wrench 接口；
// axis 是 0..5 的轴编号，value 是该轴数值。失败时抛异常，由 main 统一处理。
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

// main：依次运行 4 个物理用例和 4 个手动用例，只打印结果、不做断言。
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
