// ==============================================================================
// internal/underwater_mcu_gateway/test/motor_command_test.cpp
// ==============================================================================
// 电机指令下发与安全闸门单元测试：
// - 校验 valid_motor_setpoint 转速边界 (1000..6000 RPM) 与 run_mask 一致性
// - 校验 motor_transaction 双帧 CAN 透传结构
// - 校验 MotorGate 会话使能与超时封锁
// ==============================================================================
// 【初学者导读】本文件测试电机指令的编码与安全闸门，属于 internal/underwater_mcu_gateway/test。
// 主要有三个对象：valid_motor_setpoint（校验转速/掩码是否合法）、
// motor_transaction（把一次指令编码成两帧要发往 MCU 的字节）、
// MotorGate（会话状态机：新会话必须先用全零握手，序号递增防重放，超时自动封锁）。
// 纯逻辑测试，不需要硬件。涉及概念：位运算掩码(node_mask/run_mask)、RPM 物理边界、
// 防重放序号、看门狗超时。
#include "underwater_mcu_gateway/motor_commands.hpp"

#include <iostream>
#include <stdexcept>

using namespace underwater::wire;

// 断言计数。
int checks = 0;

// 断言：失败时报告第几个检查出错。
void check(bool condition) {
  ++checks;
  if (!condition) {
    throw std::runtime_error("check failed at index " + std::to_string(checks));
  }
}

// 依次验证停机合法性、转速边界、双帧编码、闸门握手与超时。
int main() {
  MotorRpm rpm{};

  // 1. 全零停机指令合法性校验
  check(valid_motor_setpoint(0xff, 0, rpm));
  check(!valid_motor_setpoint(0xfe, 0, rpm));  // 节点掩码必须是 0xff

  // 2. 转速边界范围测试
  rpm[0] = 1200;
  check(valid_motor_setpoint(0xff, 1, rpm));
  check(!valid_motor_setpoint(0xff, 0, rpm));   // run 未置 1 冲突
  rpm[0] = 500;                                 // 低于 1000 RPM 物理死区
  check(!valid_motor_setpoint(0xff, 1, rpm));
  rpm[0] = 6500;                                // 高于 6000 RPM 额定上限
  check(!valid_motor_setpoint(0xff, 1, rpm));

  // 3. motor_transaction 编码生成测试
  std::uint16_t seq = 1;
  rpm[0] = 2000;
  auto frames = motor_transaction(rpm, 1, seq);
  check(frames[0].size() > 0 && frames[1].size() > 0);
  check(seq == 3);  // 两帧各消耗一个序号

  // 4. MotorGate 会话闸门防重放与使能测试
  MotorGate gate;
  check(gate.blocked());

  // 新会话首次必须以全零握手指令开局
  check(!gate.accept(10, 1, true, 0xff, 1, rpm, 100));  // 首次非零拒绝
  check(gate.accept(10, 1, true, 0xff, 0, {}, 100));     // 首次全零准入握手
  check(!gate.blocked());

  // 正常下发控制动作
  check(gate.accept(10, 2, true, 0xff, 1, rpm, 200));
  check(gate.active());

  // 序号防重放校验
  check(!gate.accept(10, 2, true, 0xff, 1, rpm, 250));

  // 超时封锁保护
  check(gate.expired(800, 500));  // 超过 500ms 超时封锁
  check(gate.blocked());

  std::cout << "All " << checks << " motor command tests passed successfully.\n";
  return 0;
}
