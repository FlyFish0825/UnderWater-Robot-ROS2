// ==============================================================================
// internal/underwater_mcu_gateway/include/underwater_mcu_gateway/motor_commands.hpp
// ==============================================================================
// 电机指令编码与下发闸门：
// - encode_motor_vector 生成 0x55 下行 CAN FD 帧 (CAN ID 0x100)
//   SPEED_VECTOR(0x10) 与 RUN_VECTOR(0x11) 必须成对发送。
// - MotorGate 负责会话 ID 与序号校验去重并执行使能防线：换会话后必须以全零握手起步。
// ==============================================================================
// 初学者导读：
// 1) 所属：internal/underwater_mcu_gateway。负责生成下发给推进器的 CAN FD 串口帧，
//    并用 MotorGate 做下发前的安全检查（防重放、防旧会话、防未使能乱动）。
// 2) 交互：网关节点从 ROS 电机话题拿到 8 路转速 -> motor_transaction() 编码 ->
//    MotorGate::accept() 准入 -> 写串口 -> STM32 -> 推进器。
// 3) ROS 2 概念：本文件是纯编码/策略代码，不含 ROS API；ROS 侧电机话题在网关节点中订阅。
// 4) C++ 概念：std::array<T,N> 是固定长度数组（这里 8 个推进器、2 帧事务）；
//    using 起别名；std::uint16_t& 引用参数可用于原地自增序号。
#pragma once

#include "underwater_mcu_gateway/protocol.hpp"
#include <array>
#include <cstdint>

namespace underwater::wire {

// 8 个推进器的目标转速 (RPM)，0 表示停止
// 8 路推进器目标转速的固定数组；下标即通道号，0 表示停转。
using MotorRpm = std::array<std::int16_t, 8>;

// 校验节点掩码、run 启停位与转速的一致性：
// 非零转速必须在 1000..6000 RPM 之间且对应的 run 掩码位置 1
// 设定点合法性检查：节点掩码必须 0xff，非零转速必须在 1000..6000 RPM，
// 且 run 掩码的每一位必须与该通道"转速是否非零"严格一致。
bool valid_motor_setpoint(
    std::uint8_t node_mask,
    std::uint8_t run_mask,
    const MotorRpm &rpm);

// 组装一帧 0x55 下行 CAN FD 帧 (含 CRC8 校验)
// 编码一帧 0x55 下行 CAN FD 帧（含 CRC8）；command 只允许 0x10(转速向量) / 0x11(启停向量)。
// 参数 rpm 为 8 路转速，sequence 为本帧流水号。
Bytes encode_motor_vector(
    std::uint8_t command,
    std::uint8_t node_mask,
    std::uint8_t run_mask,
    const MotorRpm &rpm,
    std::uint16_t sequence);

// 生成双帧事务 (SPEED_VECTOR 与 RUN_VECTOR)，seq 原地自增递增
// 生成一次完整的两帧事务；注意 seq 是引用，函数返回后已自增 2，保证下一帧序号连续。
std::array<Bytes, 2> motor_transaction(
    const MotorRpm &rpm,
    std::uint8_t run_mask,
    std::uint16_t &sequence);

// 电机下发执行安全看门闸门
// 电机下发闸门：所有电机指令必须通过 accept() 才允许写串口。
// 三道防线：会话非递减、序号严格递增、未武装只放行全零。
class MotorGate {
public:
  // 校验并准入电机指令：session 必须非递减、sequence 严格递增、未使能仅放行全零停机
// 准入一帧电机指令：返回 true 才可下发。新会话第一次必须是转速全零的握手指令。
  bool accept(
      std::uint64_t session,
      std::uint64_t sequence,
      bool armed,
      std::uint8_t nodes,
      std::uint8_t run,
      const MotorRpm &rpm,
      std::uint64_t now_ms);

  // 检查是否超时 (超过 timeout_ms 未收新指令则自动封锁)
// 看门狗：超过 timeout_ms 没有新指令就封闸，防止失控后还在输出旧转速。
  bool expired(std::uint64_t now_ms, std::uint64_t timeout_ms);

  // 复位闸门 (记录 retired_session 防止旧会话延迟零包篡改状态)
// 复位闸门并记录 retired_session_：旧会话的延迟零包到达时会因 session 已退休而被拒绝。
  void reset() {
    if (session_ > retired_session_) {
      retired_session_ = session_;
    }
    session_ = 0;
    sequence_ = 0;
    last_ms_ = 0;
    blocked_ = true;
    active_ = false;
  }

  // 立即封锁闸门
// 立即封锁：不改会话信息，只是拒绝后续动作。
  void block() {
    blocked_ = true;
    active_ = false;
  }

  // 状态属性查询
  std::uint64_t session() const { return session_; }
  bool blocked() const { return blocked_; }
  bool active() const { return active_; }

private:
// 闸门内部状态：session_ 当前会话；sequence_ 已接受的最大序号；
// retired_session_ 已作废会话；blocked_ 是否封锁；active_ 是否正在输出非零转速。
  std::uint64_t session_ = 0;
  std::uint64_t sequence_ = 0;
  std::uint64_t last_ms_ = 0;
  std::uint64_t retired_session_ = 0;
  bool blocked_ = true;
  bool active_ = false;
};

}  // namespace underwater::wire
