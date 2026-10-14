// ==============================================================================
// internal/underwater_devices/include/underwater_devices/adapters.hpp
// ==============================================================================
// 设备帧解析适配层：把 MCU 网关帧 GatewayFrame 解码为各传感器/执行器状态消息。
// 返回 true 表示该帧属于本函数的 family/command 组合，且输出消息已被填充。
// 协议无法提供的物理量统一填 NaN (invalid())，而不是 0。
// ==============================================================================
#pragma once

#include <underwater_interfaces/msg/gateway_frame.hpp>
#include <underwater_interfaces/msg/imu_state.hpp>
#include <underwater_interfaces/msg/depth_state.hpp>
#include <underwater_interfaces/msg/motor_state.hpp>

#include <limits>

namespace underwater::devices {

namespace ui = underwater_interfaces::msg;

// 统一的无效浮点数标记 (NaN)
inline double invalid() {
  return std::numeric_limits<double>::quiet_NaN();
}

// 解码 IMU 传感器帧 (0x5b, target=1, command=0x80/0x81/0x85)
bool imu(const ui::GatewayFrame &frame, ui::ImuState &message);

// 解码 MS5837 深度/压力计帧 (0x5b, target=2, command=0x82)
bool depth(const ui::GatewayFrame &frame, ui::DepthState &message);

// 解码推进器电机运行状态反馈帧 (0x55, CAN ID 0x201..0x208)
bool motor_feedback(const ui::GatewayFrame &frame, ui::MotorState &message);

// 判定是否为推进器心跳子帧 (0x55, CAN ID 0x281..0x288)
bool motor_heartbeat(const ui::GatewayFrame &frame);

}  // namespace underwater::devices
