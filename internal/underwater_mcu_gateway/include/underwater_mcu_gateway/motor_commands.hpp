// ==============================================================================
// internal/underwater_mcu_gateway/include/underwater_mcu_gateway/motor_commands.hpp
// ==============================================================================
// 电机指令编码与下发闸门：
// - encode_motor_vector 生成 0x55 下行 CAN FD 帧 (CAN ID 0x100)
//   SPEED_VECTOR(0x10) 与 RUN_VECTOR(0x11) 必须成对发送。
// - MotorGate 负责会话 ID 与序号校验去重并执行使能防线：换会话后必须以全零握手起步。
// ==============================================================================
#pragma once

#include "underwater_mcu_gateway/protocol.hpp"
#include <array>
#include <cstdint>

namespace underwater::wire {

// 8 个推进器的目标转速 (RPM)，0 表示停止
using MotorRpm = std::array<std::int16_t, 8>;

// 校验节点掩码、run 启停位与转速的一致性：
// 非零转速必须在 1000..6000 RPM 之间且对应的 run 掩码位置 1
bool valid_motor_setpoint(
    std::uint8_t node_mask,
    std::uint8_t run_mask,
    const MotorRpm &rpm);

// 组装一帧 0x55 下行 CAN FD 帧 (含 CRC8 校验)
Bytes encode_motor_vector(
    std::uint8_t command,
    std::uint8_t node_mask,
    std::uint8_t run_mask,
    const MotorRpm &rpm,
    std::uint16_t sequence);

// 生成双帧事务 (SPEED_VECTOR 与 RUN_VECTOR)，seq 原地自增递增
std::array<Bytes, 2> motor_transaction(
    const MotorRpm &rpm,
    std::uint8_t run_mask,
    std::uint16_t &sequence);

// 电机下发执行安全看门闸门
class MotorGate {
public:
  // 校验并准入电机指令：session 必须非递减、sequence 严格递增、未使能仅放行全零停机
  bool accept(
      std::uint64_t session,
      std::uint64_t sequence,
      bool armed,
      std::uint8_t nodes,
      std::uint8_t run,
      const MotorRpm &rpm,
      std::uint64_t now_ms);

  // 检查是否超时 (超过 timeout_ms 未收新指令则自动封锁)
  bool expired(std::uint64_t now_ms, std::uint64_t timeout_ms);

  // 复位闸门 (记录 retired_session 防止旧会话延迟零包篡改状态)
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
  void block() {
    blocked_ = true;
    active_ = false;
  }

  // 状态属性查询
  std::uint64_t session() const { return session_; }
  bool blocked() const { return blocked_; }
  bool active() const { return active_; }

private:
  std::uint64_t session_ = 0;
  std::uint64_t sequence_ = 0;
  std::uint64_t last_ms_ = 0;
  std::uint64_t retired_session_ = 0;
  bool blocked_ = true;
  bool active_ = false;
};

}  // namespace underwater::wire
