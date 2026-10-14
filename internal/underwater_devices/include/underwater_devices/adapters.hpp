// ==============================================================================
// internal/underwater_devices/include/underwater_devices/adapters.hpp
// ==============================================================================
// 设备帧解析适配层：把 MCU 网关帧 GatewayFrame 解码为各传感器/执行器状态消息。
// 返回 true 表示该帧属于本函数的 family/command 组合，且输出消息已被填充。
// 协议无法提供的物理量统一填 NaN (invalid())，而不是 0。
// ==============================================================================
// 初学者导读（underwater_devices / adapters.hpp）
// 所属：设备解码包（internal/underwater_devices）的头文件。整条数据链是：
//   传感器 → STM32(MCU) → 串口网关(gateway) → devices 节点 → control → 推进器。
//   本文件只声明“把网关帧解码成状态消息”的纯函数，真正实现在 src/ 下的 *_decoder.cpp。
// 谁来调用：devices_node.cpp 的 DevicesNode::receive() 收到 gateway/frames 话题后，
//   依次调用这里声明的 imu()/depth()/motor_feedback()/motor_heartbeat() 来识别帧类型。
// ROS 2 概念：话题(topic) 是节点之间广播消息的通道；消息类型(如 ImuState) 由 .msg 生成。
//   本头文件不含任何 ROS 调用，只依赖生成的消息类型，因此便于单独测试。
// C++ 概念：namespace 命名空间用于避免名字冲突；inline 表示函数定义可直接写在头文件；
//   引用 & 表示“借用不拷贝”，由调用方传入已经存在的消息对象。
#pragma once

#include <underwater_interfaces/msg/gateway_frame.hpp>
#include <underwater_interfaces/msg/imu_state.hpp>
#include <underwater_interfaces/msg/depth_state.hpp>
#include <underwater_interfaces/msg/motor_state.hpp>

#include <limits>

namespace underwater::devices {

namespace ui = underwater_interfaces::msg;

// invalid() 返回 NaN（quiet_NaN）：协议拿不到的物理量统一填 NaN，而不是 0，
// 这样下游就能区分“未知”和“真实的 0”。
// 统一的无效浮点数标记 (NaN)
inline double invalid() {
  return std::numeric_limits<double>::quiet_NaN();
}

// imu()：只处理 IMU 的三类帧；返回 true 表示帧已被识别且 message 已填好，
// 返回 false 时 message 不可用，调用方不应读取。
// 解码 IMU 传感器帧 (0x5b, target=1, command=0x80/0x81/0x85)
bool imu(const ui::GatewayFrame &frame, ui::ImuState &message);

// depth()：解析 MS5837 压力/深度计帧，是“深度估计”链路的入口。
// 解码 MS5837 深度/压力计帧 (0x5b, target=2, command=0x82)
bool depth(const ui::GatewayFrame &frame, ui::DepthState &message);

// motor_feedback()：解析推进器运行反馈（转速、电流、电压、温度）。
// 解码推进器电机运行状态反馈帧 (0x55, CAN ID 0x201..0x208)
bool motor_feedback(const ui::GatewayFrame &frame, ui::MotorState &message);

// motor_heartbeat()：只判断“是不是电机心跳帧”，不填消息，用于判断电机是否在线。
// 判定是否为推进器心跳子帧 (0x55, CAN ID 0x281..0x288)
bool motor_heartbeat(const ui::GatewayFrame &frame);

}  // namespace underwater::devices
