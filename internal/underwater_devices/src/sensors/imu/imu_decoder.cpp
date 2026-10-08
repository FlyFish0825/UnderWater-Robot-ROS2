#include "underwater_devices/adapters.hpp"
#include "underwater_mcu_gateway/protocol.hpp"
#include <algorithm>
#include <cmath>
namespace underwater::devices {
bool imu(const ui::GatewayFrame & f, ui::ImuState & m) {
  if(f.family!=0x5b || f.target!=1 || f.flags!=8 ||
    !((f.command==0x80 && f.payload.size()==40)||(f.command==0x81 && f.payload.size()==32)||
      (f.command==0x85 && f.payload.size()==20))) return false;
  m=ui::ImuState{}; m.header=f.header; m.connection_id=f.connection_id;
  m.device_timestamp_us=f.device_timestamp_us; m.sequence=f.sequence;
  m.device_status=wire::u32(f.payload,0); m.group=f.command; m.fresh=true;
  m.acceleration_m_s2.fill(invalid()); m.angular_velocity_rad_s.fill(invalid());
  m.magnetic_protocol_units.fill(invalid()); m.quaternion_wxyz.fill(invalid()); m.euler_rpy_rad.fill(invalid());
  m.baro_pressure_pa=m.baro_temperature_c=m.baro_height_m=m.baro_reference_pa=invalid();
  if(f.command==0x80) {
    std::array<double,9> v;
    for(std::size_t i=0;i<v.size();++i) v[i]=wire::f32(f.payload,4+i*4);
    m.raw_valid=(m.device_status&2) && std::all_of(v.begin(),v.end(),[](double x){return std::isfinite(x);});
    if(m.raw_valid) for(std::size_t i=0;i<3;++i) {
      m.acceleration_m_s2[i]=v[i]*9.80665; m.angular_velocity_rad_s[i]=v[i+3]; m.magnetic_protocol_units[i]=v[i+6];
    }
  } else if(f.command==0x81) {
    std::array<double,7> v;
    for(std::size_t i=0;i<v.size();++i) v[i]=wire::f32(f.payload,4+i*4);
    m.quaternion_valid=(m.device_status&4) && std::all_of(v.begin(),v.begin()+4,[](double x){return std::isfinite(x);});
    const double norm=v[0]*v[0]+v[1]*v[1]+v[2]*v[2]+v[3]*v[3];
    m.quaternion_valid=m.quaternion_valid && norm>0.5 && norm<1.5;
    m.euler_valid=(m.device_status&8) && std::all_of(v.begin()+4,v.end(),[](double x){return std::isfinite(x);});
    if(m.quaternion_valid) std::copy(v.begin(),v.begin()+4,m.quaternion_wxyz.begin());
    if(m.euler_valid) std::copy(v.begin()+4,v.end(),m.euler_rpy_rad.begin());
  } else {
    // The native 0x32 order is preserved on AA5B 0x85: height, temperature, pressure, reference.
    const double height=wire::f32(f.payload,4),temperature=wire::f32(f.payload,8);
    const double pressure=wire::f32(f.payload,12),reference=wire::f32(f.payload,16);
    m.baro_pressure_valid=(m.device_status&(1u<<4)) && std::isfinite(pressure) && pressure>0;
    m.baro_temperature_valid=(m.device_status&(1u<<5)) && std::isfinite(temperature);
    m.baro_reference_valid=(m.device_status&(1u<<4)) && std::isfinite(reference) && reference>0;
    m.baro_height_valid=m.baro_pressure_valid && m.baro_reference_valid && std::isfinite(height);
    if(m.baro_pressure_valid) m.baro_pressure_pa=pressure;
    if(m.baro_temperature_valid) m.baro_temperature_c=temperature;
    if(m.baro_reference_valid) m.baro_reference_pa=reference;
    if(m.baro_height_valid) m.baro_height_m=height;
  }
  return true;
}
}
