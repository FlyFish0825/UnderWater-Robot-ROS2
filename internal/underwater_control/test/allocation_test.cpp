// ==============================================================================
// internal/underwater_control/test/allocation_test.cpp
// ==============================================================================
// 推力分配算法与物理映射单元测试：
// - 独立物理夹具 (URDF 推进器安装位置与正向 RPM 推力方向)
// - 六自由度单轴物理 wrench 求解与合力矩方向校验
// - 手动档位单轴归一化与平衡起步逻辑校验
// - 边界条件测试 (极值、NaN、非排列 ID、死区与饱和截断)
// ==============================================================================
// ------------------------------------------------------------------------------
// 【初学者导读】这是推力分配算法的单元测试程序（普通可执行文件，非 ROS 2 节点）。
// 用途：在真实求解器上验证几何映射、六轴方向、手动起步、死区/饱和与异常防御。
// 相关文件：被测实现是 allocator.cpp / linear_rpm.cpp；
//           current_robot_8thruster_config.hpp 提供独立物理参数，用来交叉校验。
// C++ 概念：
//   check() 用抛异常的方式表示断言失败，未被捕获时程序以非零码退出；
//   std::array 比较用 ==，整型转 double 用 static_cast。
// 阅读顺序：先看 physical_wrench（独立物理模型），再看 main 里的 1~7 组测试。
// ------------------------------------------------------------------------------
#include "underwater_control/allocator.hpp"
#include <current_robot_8thruster_config.hpp>
// 标准库：算法、数学、输入输出、数值极限、异常。

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

// 直接使用分配器命名空间里的类型。
using namespace underwater::control;

// 全局计数器：记录一共执行了多少条断言，最后打印总数。
int checks = 0;

// check：最简断言。失败时抛出异常，程序终止并打印失败序号。
// 坑位：C++ 的 assert 在发布版可能被禁用，这里用抛异常保证一定生效。
void check(bool value) {
  ++checks;
  if (!value) {
    throw std::runtime_error("check failed at index " + std::to_string(checks));
  }
}

// 独立物理校验模型：基于 URDF 实际物理坐标和操作手确认的正向转速推力方向
// physical_wrench：完全独立于被测代码的“第二套物理模型”，用来反向验证结果。
// 输入 8 路 RPM，按 URDF 安装位置与正转推力方向正算六维 wrench。
// 与 allocator 上报的 modeled_rpm 对比，即可发现极性或几何错误。
Wrench physical_wrench(const std::array<std::int16_t, 8> &rpm) {
  const double diagonal = std::sqrt(0.5);
  const std::array<Vec3, 8> positive_rpm_axes{{
      {-diagonal, -diagonal, 0.0},
      { diagonal, -diagonal, 0.0},
      {-diagonal,  diagonal, 0.0},
      { diagonal,  diagonal, 0.0},
      { 0.0,       0.0,      1.0},
      { 0.0,       0.0,      1.0},
      { 0.0,       0.0,     -1.0},
      { 0.0,       0.0,     -1.0}}};

  Wrench w{};
  Vec3 center{};
  for (const auto &t : current_rov::kThrusters) {
    center[0] += t.position_m.x / 8.0;
    center[1] += t.position_m.y / 8.0;
    center[2] += t.position_m.z / 8.0;
  }

  for (std::size_t i = 0; i < 8; ++i) {
    const auto &position = current_rov::kThrusters[i].position_m;
    const Vec3 p{position.x - center[0], position.y - center[1], position.z - center[2]};
    const auto &a = positive_rpm_axes[i];
    const double force = static_cast<double>(rpm[i]) / 12.0;  // 临时模型：6000 RPM <-> 500 N

    for (int j = 0; j < 3; ++j) {
      w[j] += a[j] * force;
    }
    w[3] += (p[1] * a[2] - p[2] * a[1]) * force;
    w[4] += (p[2] * a[0] - p[0] * a[2]) * force;
    w[5] += (p[0] * a[1] - p[1] * a[0]) * force;
  }
  return w;
}

// check_physical_mapping：逐轴比较独立模型结果与分配器上报的 modeled_rpm。
// 容差 1e-4，用于吸收浮点误差。
void check_physical_mapping(const Result &result) {
  const auto w = physical_wrench(result.rpm);
  for (std::size_t j = 0; j < 6; ++j) {
    check(std::abs(w[j] - result.modeled_rpm[j]) < 1e-4);
  }
}

// main：构造一个默认配置的分配器，顺序执行 7 组测试，全部通过后打印成功信息。
// 任何 check 失败都会抛异常终止程序。
int main() {
  Allocator allocator(Config{});
  Result r;

  // 1. 全零指令停机测试
  check(allocator.solve({}, r));
  check(r.rpm == std::array<std::int16_t, 8>{});
  check(!r.deadband && !r.saturated);
  check_physical_mapping(r);

  // 2. 六自由度正负单轴物理 Wrench 求解测试
  for (int axis = 0; axis < 6; ++axis) {
    for (double sign : {1.0, -1.0}) {
      Wrench desired{};
      desired[axis] = sign * 50.0;
      check(allocator.solve(desired, r));
      check_physical_mapping(r);

      // 验证主受力轴符号与期望一致
      check(r.modeled_rpm[axis] * sign > 0.0);

      // 水平通道与垂直通道相互正交解耦隔离
      if (axis < 2 || axis == 5) {
        check(r.rpm[4] == 0 && r.rpm[5] == 0 && r.rpm[6] == 0 && r.rpm[7] == 0);
      } else {
        check(r.rpm[0] == 0 && r.rpm[1] == 0 && r.rpm[2] == 0 && r.rpm[3] == 0);
      }
    }
  }

  // 3. 手动档位单轴归一化与平衡起动测试
  for (int axis = 0; axis < 6; ++axis) {
    for (double sign : {1.0, -1.0}) {
      Wrench manual{};
      manual[axis] = sign * 50.0;
      check(allocator.solve_manual(manual, r));
      check_physical_mapping(r);

      // 验证手动档位转换后的物理主轴合力符号
      const double physical_sign = allocator.config().manual_axis_signs[axis] * sign;
      check(r.modeled_rpm[axis] * physical_sign > 0.0);
    }
  }

  // 4. 起动档位测试 (Level 1 应触发 balanced startup，越过死区且不超过最大转速)
  Wrench min_manual{};
  min_manual[0] = 1.0;
  check(allocator.solve_manual(min_manual, r));
  int active_count = 0;
  for (auto rpm : r.rpm) {
    if (rpm != 0) {
      ++active_count;
      check(std::abs(rpm) >= allocator.config().min_rpm);
      check(std::abs(rpm) <= allocator.config().max_rpm);
    }
  }
  check(active_count == 4);  // X 轴平面运动应由 4 个水平推进器协同动作

  // 5. 档位饱和测试 (Level 100 对应最大额定转速)
  Wrench max_manual{};
  max_manual[0] = 100.0;
  check(allocator.solve_manual(max_manual, r));
  int peak_rpm = 0;
  for (auto rpm : r.rpm) {
    peak_rpm = std::max(peak_rpm, std::abs(static_cast<int>(rpm)));
  }
  check(peak_rpm == allocator.config().max_rpm);

  // 6. 异常输入边界防御测试
  Wrench invalid{};
  invalid[0] = std::numeric_limits<double>::quiet_NaN();
  check(!allocator.solve(invalid, r));
  check(!allocator.solve_manual(invalid, r));

  invalid[0] = 101.0;
  check(!allocator.solve_manual(invalid, r));

  // 7. 配置构造函数参数校验测试
  Config bad_cfg;
  bad_cfg.node_ids = {1, 2, 3, 4, 5, 6, 7, 7};  // 重复 ID
  try {
    Allocator bad_allocator(bad_cfg);
    check(false);
  } catch (const std::invalid_argument &) {
    check(true);
  }

  bad_cfg = Config{};
  bad_cfg.manual_axis_signs[0] = 0;  // 非法符号
  try {
    Allocator bad_allocator(bad_cfg);
    check(false);
  } catch (const std::invalid_argument &) {
    check(true);
  }

  std::cout << "All " << checks << " allocation tests passed successfully.\n";
  return 0;
}
