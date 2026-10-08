#include "underwater_devices/adapters.hpp"
#include "underwater_mcu_gateway/protocol.hpp"
namespace underwater::devices {
bool motor_feedback(const ui::GatewayFrame & f, ui::MotorState & m) {
  if(f.family!=0x55 || f.can_id<0x201 || f.can_id>0x208 ||
     (f.flags!=2 && f.flags!=6) || f.payload.size()!=12 || f.payload[8]>2) return false;
  const auto & b=f.payload;
  m=ui::MotorState{}; m.header=f.header; m.connection_id=f.connection_id; m.node_id=f.can_id-0x200;
  m.online=true; m.feedback_valid=true; m.device_timestamp_valid=f.device_timestamp_valid;
  m.device_timestamp_us=f.device_timestamp_us;
  const auto raw_speed=wire::u16(b,0);
  m.speed_rpm=raw_speed<0x8000?double(raw_speed):double(raw_speed)-65536.0;
  m.estimated_bus_current_a=wire::u16(b,2)*10.0/65535.0;
  m.bus_voltage_v=wire::u16(b,4)/100.0; m.mcu_temperature_c=-20.0+wire::u16(b,6)*170.0/65535.0;
  m.state=b[8]; m.current_calibration_done=b[9]&1; m.speed_loop_enabled=b[9]&2;
  m.voltage_limited=b[9]&4; m.feedback_sequence=b[10]; return true;
}
bool motor_heartbeat(const ui::GatewayFrame & f) {
  return f.family==0x55 && f.can_id>=0x281 && f.can_id<=0x288 && (f.flags&7)==0 &&
    f.payload.size()==8 && f.payload[0]==f.can_id-0x280 && f.payload[1]<=2 &&
    wire::crc8(f.payload.data(),7)==f.payload[7];
}
}
