#include "underwater_devices/adapters.hpp"
#include "underwater_mcu_gateway/protocol.hpp"
#include <cmath>
namespace underwater::devices {
bool depth(const ui::GatewayFrame & f, ui::DepthState & m) {
  if(f.family!=0x5b || f.target!=2 || f.command!=0x82 || f.flags!=8 || f.payload.size()!=32) return false;
  m=ui::DepthState{}; m.header=f.header; m.connection_id=f.connection_id;
  m.device_timestamp_us=f.device_timestamp_us; m.sequence=f.sequence;
  m.device_status=wire::u32(f.payload,0); m.fresh=true;
  const auto s=m.device_status; m.raw_valid=s&2;
  m.pressure_pa=wire::f32(f.payload,4); m.temperature_c=wire::f32(f.payload,8);
  m.raw_depth_m=wire::f32(f.payload,12); m.filtered_depth_m=wire::f32(f.payload,16);
  m.surface_pressure_pa=wire::f32(f.payload,20);
  m.pressure_valid=(s&(1<<4)) && std::isfinite(m.pressure_pa);
  m.temperature_valid=(s&(1<<5)) && std::isfinite(m.temperature_c);
  m.depth_valid=(s&(1<<6)) && std::isfinite(m.raw_depth_m) && std::isfinite(m.filtered_depth_m);
  m.zero_valid=(s&(1<<7)) && std::isfinite(m.surface_pressure_pa);
  if(!m.pressure_valid) m.pressure_pa=invalid();
  if(!m.temperature_valid) m.temperature_c=invalid();
  if(!m.depth_valid) { m.raw_depth_m=invalid(); m.filtered_depth_m=invalid(); }
  if(!m.zero_valid) m.surface_pressure_pa=invalid();
  if(m.raw_valid) { m.d1=wire::u32(f.payload,24); m.d2=wire::u32(f.payload,28); }
  return true;
}
}
