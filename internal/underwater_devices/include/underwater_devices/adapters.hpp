#pragma once
#include <underwater_interfaces/msg/gateway_frame.hpp>
#include <underwater_interfaces/msg/imu_state.hpp>
#include <underwater_interfaces/msg/depth_state.hpp>
#include <underwater_interfaces/msg/motor_state.hpp>
#include <limits>
namespace underwater::devices {
namespace ui=underwater_interfaces::msg;
inline double invalid() { return std::numeric_limits<double>::quiet_NaN(); }
bool imu(const ui::GatewayFrame &, ui::ImuState &);
bool depth(const ui::GatewayFrame &, ui::DepthState &);
bool motor_feedback(const ui::GatewayFrame &, ui::MotorState &);
bool motor_heartbeat(const ui::GatewayFrame &);
}
