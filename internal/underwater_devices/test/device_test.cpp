#include "underwater_devices/adapters.hpp"
#include "underwater_mcu_gateway/protocol.hpp"
#include <cmath>
#include <cstring>
#include <iostream>
#include <stdexcept>
using namespace underwater::devices;
int checks=0;
void check(bool b) { ++checks; if(!b) throw std::runtime_error("check "+std::to_string(checks)); }
void put(ui::GatewayFrame & f,std::size_t o,float value) {
  std::uint32_t bits; std::memcpy(&bits,&value,4);
  for(int i=0;i<4;++i) f.payload[o+i]=(bits>>(i*8))&255;
}
int main() {
  try {
    ui::GatewayFrame f; f.family=0x5b; f.flags=8; f.target=2; f.command=0x82;
    f.payload.resize(32); f.payload[0]=3; f.payload[1]=5;
    ui::DepthState d; check(depth(f,d)); check(d.raw_valid && !d.pressure_valid && !d.depth_valid);
    check(std::isnan(d.pressure_pa) && std::isnan(d.raw_depth_m));
    f.payload[0]=0xf3; f.payload[1]=1; // Fixed sensor data is valid without a model-confirmation bit.
    put(f,4,101325); put(f,8,20); put(f,12,1.2f); put(f,16,1.1f); put(f,20,100000);
    check(depth(f,d) && d.pressure_valid && d.depth_valid && d.zero_valid);
    put(f,4,std::numeric_limits<float>::quiet_NaN()); check(depth(f,d) && !d.pressure_valid);
    f.target=1; f.command=0x80; f.payload.assign(40,0); f.payload[0]=3;
    put(f,4,1.0f); put(f,16,-0.2f);
    ui::ImuState im; check(imu(f,im) && im.raw_valid);
    check(std::abs(im.acceleration_m_s2[0]-9.80665)<1e-8);
    check(!im.quaternion_valid && std::isnan(im.quaternion_wxyz[0]));
    f.payload[0]=1; check(imu(f,im) && !im.raw_valid && std::isnan(im.acceleration_m_s2[0]));
    check(!im.baro_pressure_valid && std::isnan(im.baro_pressure_pa));
    f.command=0x85; f.sequence=3; f.payload.assign(20,0); f.payload[0]=0x30;
    put(f,4,-12.5f); put(f,8,23.25f); put(f,12,101325); put(f,16,101500);
    check(imu(f,im) && im.group==0x85 && im.baro_pressure_valid && im.baro_temperature_valid);
    check(im.baro_pressure_pa==101325 && im.baro_temperature_c==23.25);
    check(im.baro_height_valid && im.baro_height_m==-12.5 && im.baro_reference_valid && im.baro_reference_pa==101500);
    check(!im.raw_valid && !im.quaternion_valid && std::isnan(im.acceleration_m_s2[0]));
    // ONLINE follows the raw group on the MCU, so fresh barometer data does not depend on it.
    f.payload[0]=0x20; check(imu(f,im) && !im.baro_pressure_valid && im.baro_temperature_valid);
    check(!im.baro_height_valid && std::isnan(im.baro_pressure_pa));
    f.payload[0]=0x30; put(f,12,std::numeric_limits<float>::quiet_NaN());
    check(imu(f,im) && !im.baro_pressure_valid && !im.baro_height_valid);
    put(f,12,101325); put(f,16,0); check(imu(f,im) && !im.baro_reference_valid && !im.baro_height_valid);
    f.payload.resize(19); check(!imu(f,im));
    f.payload.resize(20); f.target=2; check(!imu(f,im));
    f.family=0x55; f.flags=2; f.can_id=0x201; f.payload.assign(12,0);
    f.payload[0]=0x9c; f.payload[1]=0xff; f.payload[2]=f.payload[3]=0xff;
    f.payload[4]=0x60; f.payload[5]=9; f.payload[6]=f.payload[7]=0xff;
    ui::MotorState m; check(motor_feedback(f,m));
    check(m.speed_rpm==-100 && m.estimated_bus_current_a==10 && m.bus_voltage_v==24 && m.mcu_temperature_c==150);
    f.flags=0; check(!motor_feedback(f,m));
    f.can_id=0x281; f.payload={1,0,1,0,25,0,0,0};
    f.payload[7]=underwater::wire::crc8(f.payload.data(),7);
    check(motor_heartbeat(f)); f.payload[0]=2; check(!motor_heartbeat(f));
    std::cout<<"PASS "<<checks<<" device checks\n"; return 0;
  } catch(const std::exception & e) { std::cerr<<e.what()<<"\n"; return 1; }
}
