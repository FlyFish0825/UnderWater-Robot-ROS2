#include "underwater_mcu_gateway/motor_commands.hpp"
#include <iostream>
#include <stdexcept>
using namespace underwater::wire;
int checks=0;
void check(bool b) { ++checks; if(!b) throw std::runtime_error("check "+std::to_string(checks)); }
int main() {
 try {
  MotorRpm rpm{1000,-6000,0,0,0,0,0,0}; std::uint16_t seq=0;
  auto pair=motor_transaction(rpm,3,seq); Frame f;
  check(pair[0].size()==38 && pair[0][2]==32);
  check(decode(pair[0],f) && f.can_id==0x100 && f.flags==2 && !f.timestamp_valid);
  check(f.payload.size()==24 && f.payload[0]==1 && f.payload[1]==0x10 && f.payload[2]==255 && f.payload[3]==0);
  check(u16(f.payload,8)==1000 && u16(f.payload,10)==std::uint16_t(-6000));
  check(decode(pair[1],f) && f.payload[1]==0x11 && f.payload[3]==3);
  auto stop=motor_transaction({},0,seq);
  check(decode(stop[0],f) && f.payload[1]==0x11 && f.payload[3]==0); // Essential explicit stop.
  check(decode(stop[1],f) && f.payload[1]==0x10);
  check(!valid_motor_setpoint(255,0,rpm));
  rpm[0]=999; check(!valid_motor_setpoint(255,3,rpm)); rpm[0]=6001; check(!valid_motor_setpoint(255,3,rpm));
  MotorGate gate; rpm={1000,0,0,0,0,0,0,0};
  check(!gate.accept(10,1,true,255,1,rpm,100)); // No zero handshake.
  check(gate.accept(10,1,true,255,0,{},100));
  check(gate.accept(10,2,true,255,1,rpm,120));
  check(!gate.accept(10,2,true,255,1,rpm,140)); // Replay cannot refresh the watchdog.
  check(!gate.expired(619,500)); check(gate.expired(620,500)); check(gate.blocked());
  check(!gate.accept(10,3,true,255,1,rpm,640)); // No automatic restart.
  check(gate.accept(11,1,true,255,0,{},660)); check(!gate.blocked());
  check(!gate.accept(10,4,false,255,0,{},680)); // Previous session cannot override new authority.
  check(gate.accept(11,2,true,255,1,rpm,700));
  check(gate.accept(11,3,false,255,0,{},720)); check(gate.blocked());
  check(!gate.accept(11,4,true,255,1,rpm,740));
  std::cout<<"PASS "<<checks<<" motor command checks\n"; return 0;
 } catch(const std::exception & e) { std::cerr<<e.what()<<"\n"; return 1; }
}
