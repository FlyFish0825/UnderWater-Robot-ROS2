// ==============================================================================
// internal/underwater_mcu_gateway/src/commands/motor_commands.cpp
// ==============================================================================
// 电机指令编码与下发闸门实现：
// 下行帧为 0x55 family 的 CAN FD 透传 (CAN ID 0x100)，两帧成对：
// - SPEED_VECTOR (0x10): 目标转速向量
// - RUN_VECTOR (0x11): 启停掩码向量
// 安全约定：
// - 非零转速必须在 1000..6000 RPM 区间，run 位与转速严格对应；
// - 未使能时只允许零指令；新会话首次必须以全零握手起跳。
// ==============================================================================
// 初学者导读：
// 1) 本文件实现 motor_commands.hpp：把 8 路转速编码成串口 CAN FD 帧，并做下发准入。
// 2) 交互：网关节点订阅 ROS 电机话题后调用 motor_transaction() 生成两帧，
//    再用 MotorGate::accept() 判断能否写串口；通过后字节经 SerialIo 发给 STM32。
// 3) ROS 2 概念：本文件无 ROS API；ROS 的电机话题在网关节点中订阅与限幅。
// 4) C++ 概念：匿名 namespace 内部函数；std::array 固定数组；std::move 转移 Bytes；
//    输入非法时用异常 std::invalid_argument 快速失败。
#include "underwater_mcu_gateway/motor_commands.hpp"

#include <cstdlib>
#include <stdexcept>

namespace underwater::wire {

namespace {

// 小端序写入 16 位无符号整数
// 小端序追加 16 位整数（低字节在前）。
void put16(Bytes &b, std::uint16_t v) {
  b.push_back(static_cast<std::uint8_t>(v & 0xff));
  b.push_back(static_cast<std::uint8_t>((v >> 8) & 0xff));
}

}  // namespace

// 校验电机设定点：非零通道转速在 1000..6000 之间，且与 run_mask 逐位一致
// 校验是编码前的第一道安全闸；本机固定使用全部 8 个节点（掩码 0xff）。
bool valid_motor_setpoint(std::uint8_t nodes, std::uint8_t run, const MotorRpm &rpm) {
  // 本水下机器人独占全部 8 个配置节点
  if (nodes != 0xff) {
    return false;
  }

  for (std::size_t i = 0; i < 8; ++i) {
    const int value = rpm[i];
    const int absolute = std::abs(value);

    // 死区外的有效转速必须在 1000..6000 RPM 之间
    if (value != 0 && (absolute < 1000 || absolute > 6000)) {
      return false;
    }

    // run 位必须与"该通道转速是否非零"严格对应
    const bool bit_active = (run & (1u << i)) != 0;
    const bool speed_active = (value != 0);
    if (bit_active != speed_active) {
      return false;
    }
  }
  return true;
}

// 组装一帧 0x55 下行 CAN FD 帧 (含 CRC8)；command 仅允许 0x10 或 0x11
Bytes encode_motor_vector(
    std::uint8_t command,
    std::uint8_t nodes,
    std::uint8_t run,
    const MotorRpm &rpm,
    std::uint16_t sequence) {
  if (command != 0x10 && command != 0x11) {
    throw std::invalid_argument("unsupported motor command");
  }
  if (nodes != 0xff) {
    throw std::invalid_argument("invalid motor node mask");
  }

  for (auto value : rpm) {
    if (value != 0 && (std::abs(static_cast<int>(value)) < 1000 || std::abs(static_cast<int>(value)) > 6000)) {
      throw std::invalid_argument("motor speed out of range");
    }
  }

  // 帧载荷：首字节 1 = CAN FD 数据帧；command, nodes, run, sequence
// data 是 CAN 帧的数据段：节点掩码、run 掩码、序号与 8 路转速。
  Bytes data{1, command, nodes, run};
  put16(data, sequence);
  put16(data, 0);

  // 各通道目标转速 (小端序)
  for (auto value : rpm) {
    put16(data, static_cast<std::uint16_t>(value));
  }

  // 构造物理数据包：帧头 0xAA 0x55，包体长 32，流水号
  Bytes packet{0xaa, 0x55, 32};
  put16(packet, sequence);

  // CAN ID 0x100, FD 格式且无 BRS
// 这 6 字节是 CAN FD 元数据：CAN ID=0x100、帧类型、DLC=24。
  packet.insert(packet.end(), {0x00, 0x01, 0x00, 0x00, 0x02, 24});
  packet.insert(packet.end(), data.begin(), data.end());

  // CRC-8 校验和 (覆盖自包长到数据体末尾)
// CRC8 覆盖"包长"字节到数据体末尾，不包含帧头 AA 55。
  packet.push_back(crc8(packet.data() + 2, packet.size() - 2));

  // 帧尾 (0x55 0xAA)
  packet.insert(packet.end(), {0x55, 0xaa});
  return packet;
}

// 生成双帧事务报文：
// 当 run=0 时，必须先发 RUN_VECTOR 停止帧，再发速度清零帧，以确保下位机固件彻底刹停
// 注意：转速帧与启停帧的顺序/正负约定直接影响下位机固件行为，不要随意调整。
std::array<Bytes, 2> motor_transaction(
    const MotorRpm &rpm,
    std::uint8_t run,
    std::uint16_t &seq) {
  if (!valid_motor_setpoint(0xff, run, rpm)) {
    throw std::invalid_argument("invalid run/RPM pair");
  }

  const MotorRpm zeros{};
  if (run == 0) {
    // 固件保护：先发送 RUN_VECTOR(0) 清除运行标志，再发 SPEED_VECTOR(0)
    auto stop = encode_motor_vector(0x11, 0xff, 0, zeros, seq++);
    auto speed_zero = encode_motor_vector(0x10, 0xff, 0, zeros, seq++);
    return {std::move(stop), std::move(speed_zero)};
  }

  auto speed = encode_motor_vector(0x10, 0xff, 0, rpm, seq++);
  auto run_frame = encode_motor_vector(0x11, 0xff, run, zeros, seq++);
  return {std::move(speed), std::move(run_frame)};
}

// 电机指令下发准入判定：
// - 会话 session 单调非递减、sequence 严格递增；
// - 未武装状态下只允许全零指令；
// - 新会话建立时，必须以全零握手帧开局，否则拒绝执行。
// 调用方：网关节点写串口前必须调用；返回 false 时字节一律不得下发。
bool MotorGate::accept(
    std::uint64_t session,
    std::uint64_t seq,
    bool armed,
    std::uint8_t nodes,
    std::uint8_t run,
    const MotorRpm &rpm,
    std::uint64_t now) {
  if (!session || !seq || !valid_motor_setpoint(nodes, run, rpm) || (!armed && run != 0)) {
    return false;
  }
  if (session <= retired_session_ || session < session_) {
    return false;
  }

  // 检测到新会话切换：必须以全零握手指令开局
  if (session > session_) {
    if (run != 0) {
      return false;
    }
    session_ = session;
    sequence_ = 0;
    blocked_ = !armed;
  }

  // 序号防重放校验
  if (seq <= sequence_) {
    return false;
  }

  // 封锁状态下拒绝动作
  if (armed && blocked_) {
    return false;
  }

  sequence_ = seq;
  last_ms_ = now;
  active_ = armed && (run != 0);

  // 若下发未武装指令，触发复位保护
  if (!armed) {
    blocked_ = true;
  }
  return true;
}

// 超过 timeout_ms 未收新指令触发封锁
// 返回 true 表示本轮刚触发封锁，调用方应补发停机帧。
bool MotorGate::expired(std::uint64_t now, std::uint64_t timeout) {
  if (!blocked_ && (now - last_ms_ >= timeout)) {
    block();
    return true;
  }
  return false;
}

}  // namespace underwater::wire
