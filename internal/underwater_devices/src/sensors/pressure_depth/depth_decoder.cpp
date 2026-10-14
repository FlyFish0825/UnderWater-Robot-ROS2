// ==============================================================================
// internal/underwater_devices/src/sensors/pressure_depth/depth_decoder.cpp
// ==============================================================================
// MS5837 深度/压力帧解码：AA5B target=2、command=0x82、flags=8、32 字节负载。
// device_status 的位同时充当各物理量的有效性标志，校验失败的量一律置 NaN。
// d1/d2 为传感器原始 ADC 值，仅在 raw_valid 时有意义，不做单位换算。
// ==============================================================================
// 初学者导读（underwater_devices / sensors/pressure_depth/depth_decoder.cpp）
// 所属：MS5837 压力/深度计帧解码。上游 gateway/frames，下游是 devices 节点的
//   sensors/depth/state 等话题，供深度控制与定深使用。
// 谁调用谁：devices_node.cpp 的 receive() 调用本文件的 depth()。
// 协议：family 0x5b、target 2、command 0x82、flags 8、32 字节负载；
//   device_status 的各个 bit 是物理量有效性标志。
// C++ 概念：std::isfinite 判断浮点是否为有限值（排除 NaN/Inf）；
//   wire::f32/u32 是按小端从字节数组取数的辅助函数。
#include "underwater_devices/adapters.hpp"
#include "underwater_mcu_gateway/protocol.hpp"

#include <cmath>

namespace underwater::devices {

// depth()：解码一帧深度/压力数据；返回 false 表示这不是深度类型帧。
bool depth(const ui::GatewayFrame &f, ui::DepthState &m) {
  if (f.family != 0x5b || f.target != 2 || f.command != 0x82 || f.flags != 8 || f.payload.size() != 32) {
    return false;
  }

  m = ui::DepthState{};
  m.header = f.header;
  m.connection_id = f.connection_id;
  m.device_timestamp_us = f.device_timestamp_us;
  m.sequence = f.sequence;
  m.device_status = wire::u32(f.payload, 0);
  m.fresh = true;

  const auto s = m.device_status;
  // 状态字是 32 位无符号整数，按位拆出各物理量的有效标志。
  m.raw_valid = (s & 2) != 0;  // bit 1: 原始 ADC 数据有效

  m.pressure_pa = wire::f32(f.payload, 4);
  m.temperature_c = wire::f32(f.payload, 8);
  m.raw_depth_m = wire::f32(f.payload, 12);
  m.filtered_depth_m = wire::f32(f.payload, 16);
  m.surface_pressure_pa = wire::f32(f.payload, 20);

  // 有效性 = 状态位为 1 且数值有限，二者缺一不可。
  // 状态位与有限值综合有效性判断
  m.pressure_valid = (s & (1 << 4)) && std::isfinite(m.pressure_pa);
  m.temperature_valid = (s & (1 << 5)) && std::isfinite(m.temperature_c);
  m.depth_valid = (s & (1 << 6)) && std::isfinite(m.raw_depth_m) && std::isfinite(m.filtered_depth_m);
  m.zero_valid = (s & (1 << 7)) && std::isfinite(m.surface_pressure_pa);

  // 无效的物理量统一填 NaN。
  // 无效量规范化赋 NaN
  if (!m.pressure_valid) {
    m.pressure_pa = invalid();
  }
  if (!m.temperature_valid) {
    m.temperature_c = invalid();
  }
  if (!m.depth_valid) {
    m.raw_depth_m = invalid();
    m.filtered_depth_m = invalid();
  }
  if (!m.zero_valid) {
    m.surface_pressure_pa = invalid();
  }

  // d1/d2 是温度/压力原始 ADC 值，只有 raw_valid 时才写入。
  // 原始 ADC 采样
  if (m.raw_valid) {
    m.d1 = wire::u32(f.payload, 24);
    m.d2 = wire::u32(f.payload, 28);
  }
  return true;
}

}  // namespace underwater::devices
