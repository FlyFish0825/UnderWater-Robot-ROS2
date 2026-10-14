// ==============================================================================
// internal/underwater_devices/src/actuators/thrusters/motor_decoder.cpp
// ==============================================================================
// 推进器 CAN 反馈与心跳帧解码：把网关转发的 CAN 帧还原为 MotorState。
// - 反馈帧：family=0x55，CAN ID 0x201..0x208，flags 2/6，12 字节负载；
// - 心跳帧：CAN ID 0x281..0x288，8 字节负载，末字节为前 7 字节的 CRC8。
// ==============================================================================
#include "underwater_devices/adapters.hpp"
#include "underwater_mcu_gateway/protocol.hpp"

namespace underwater::devices {

bool motor_feedback(const ui::GatewayFrame &f, ui::MotorState &m) {
  if (f.family != 0x55 || f.can_id < 0x201 || f.can_id > 0x208 ||
      (f.flags != 2 && f.flags != 6) || f.payload.size() != 12 || f.payload[8] > 2) {
    return false;  // payload[8] 状态码仅 0..2 合法
  }

  const auto &b = f.payload;
  m = ui::MotorState{};
  m.header = f.header;
  m.connection_id = f.connection_id;
  m.node_id = f.can_id - 0x200;  // CAN ID 偏移即节点编号 (1..8)
  m.online = true;
  m.feedback_valid = true;
  m.device_timestamp_valid = f.device_timestamp_valid;
  m.device_timestamp_us = f.device_timestamp_us;

  // 转速解码 (16 位有符号补码，>=0x8000 为负转速)
  const auto raw_speed = wire::u16(b, 0);
  m.speed_rpm = (raw_speed < 0x8000) ? static_cast<double>(raw_speed)
                                     : static_cast<double>(raw_speed) - 65536.0;

  // 母线电流 0..10 A 线性映射到 16 位全量程
  m.estimated_bus_current_a = wire::u16(b, 2) * 10.0 / 65535.0;

  // 电压：0.01 V/LSB
  m.bus_voltage_v = wire::u16(b, 4) / 100.0;

  // 温度：-20..150 ℃
  m.mcu_temperature_c = -20.0 + wire::u16(b, 6) * 170.0 / 65535.0;

  // 状态字与位域标志
  m.state = b[8];
  m.current_calibration_done = (b[9] & 1) != 0;  // bit 0: 电流标定完成
  m.speed_loop_enabled = (b[9] & 2) != 0;        // bit 1: 速度闭环使能
  m.voltage_limited = (b[9] & 4) != 0;           // bit 2: 电压限幅生效
  m.feedback_sequence = b[10];
  return true;
}

bool motor_heartbeat(const ui::GatewayFrame &f) {
  // (flags & 7) == 0 为心跳子帧，CAN ID 0x281..0x288
  return f.family == 0x55 &&
         f.can_id >= 0x281 &&
         f.can_id <= 0x288 &&
         (f.flags & 7) == 0 &&
         f.payload.size() == 8 &&
         f.payload[0] == f.can_id - 0x280 &&
         f.payload[1] <= 2 &&
         wire::crc8(f.payload.data(), 7) == f.payload[7];  // 末字节为前 7 字节 CRC8 校验
}

}  // namespace underwater::devices
