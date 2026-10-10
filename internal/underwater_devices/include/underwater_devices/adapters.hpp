// 设备帧解析适配层：把 MCU 网关帧 GatewayFrame 解码为各传感器/执行器状态消息。
// 返回 true 表示该帧属于本函数的 family/command 组合，且输出消息已被填充。
// 协议无法提供的物理量统一填 NaN（invalid()），而不是 0。
#pragma once
#include <underwater_interfaces/msg/gateway_frame.hpp>
#include <underwater_interfaces/msg/imu_state.hpp>
#include <underwater_interfaces/msg/depth_state.hpp>
#include <underwater_interfaces/msg/motor_state.hpp>
#include <limits>
namespace underwater::devices {
namespace ui=underwater_interfaces::msg;
inline double invalid() { return std::numeric_limits<double>::quiet_NaN(); }  // 无效物理量占位，调用方须先查 valid 标志
bool imu(const ui::GatewayFrame &, ui::ImuState &);  // 解析 AA5B target=1 的 IMU 帧（0x80/0x81/0x85）
bool depth(const ui::GatewayFrame &, ui::DepthState &);  // 解析 AA5B target=2 的深度/压力帧（0x82）
bool motor_feedback(const ui::GatewayFrame &, ui::MotorState &);  // 解析电机周期反馈帧（CAN 0x201..0x208）
bool motor_heartbeat(const ui::GatewayFrame &);  // 仅判定是否为合法电机心跳帧（CAN 0x281..0x288）
}
