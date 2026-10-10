// MS5837 深度/压力帧解码：AA5B target=2、command=0x82、flags=8、32 字节负载。
// device_status 的位同时充当各物理量的有效性标志，校验失败的量一律置 NaN。
// d1/d2 为传感器原始 ADC 值，仅在 raw_valid 时有意义，不做单位换算。
#include "underwater_devices/adapters.hpp"
#include "underwater_mcu_gateway/protocol.hpp"
#include <cmath>
namespace underwater::devices {
bool depth(const ui::GatewayFrame & f, ui::DepthState & m) {
  if(f.family!=0x5b || f.target!=2 || f.command!=0x82 || f.flags!=8 || f.payload.size()!=32) return false;
  m=ui::DepthState{}; m.header=f.header; m.connection_id=f.connection_id;
  m.device_timestamp_us=f.device_timestamp_us; m.sequence=f.sequence;
  m.device_status=wire::u32(f.payload,0); m.fresh=true;
  const auto s=m.device_status; m.raw_valid=s&2;  // bit1：原始 ADC 数据有效
  m.pressure_pa=wire::f32(f.payload,4); m.temperature_c=wire::f32(f.payload,8);  // 单位 Pa / ℃
  m.raw_depth_m=wire::f32(f.payload,12); m.filtered_depth_m=wire::f32(f.payload,16);  // 单位 m
  m.surface_pressure_pa=wire::f32(f.payload,20);  // 水面零点压力 (Pa)
  m.pressure_valid=(s&(1<<4)) && std::isfinite(m.pressure_pa);  // bit4：压力有效
  m.temperature_valid=(s&(1<<5)) && std::isfinite(m.temperature_c);  // bit5：温度有效
  m.depth_valid=(s&(1<<6)) && std::isfinite(m.raw_depth_m) && std::isfinite(m.filtered_depth_m);  // bit6：深度有效
  m.zero_valid=(s&(1<<7)) && std::isfinite(m.surface_pressure_pa);  // bit7：零点标定有效
  if(!m.pressure_valid) m.pressure_pa=invalid();
  if(!m.temperature_valid) m.temperature_c=invalid();
  if(!m.depth_valid) { m.raw_depth_m=invalid(); m.filtered_depth_m=invalid(); }
  if(!m.zero_valid) m.surface_pressure_pa=invalid();
  if(m.raw_valid) { m.d1=wire::u32(f.payload,24); m.d2=wire::u32(f.payload,28); }  // 原始 ADC 采样值
  return true;
}
}
