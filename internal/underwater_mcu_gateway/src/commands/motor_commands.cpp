#include "underwater_mcu_gateway/motor_commands.hpp"
#include <cstdlib>
#include <stdexcept>
namespace underwater::wire {
static void put16(Bytes & b,std::uint16_t v) { b.push_back(v&255); b.push_back(v>>8); }
bool valid_motor_setpoint(std::uint8_t nodes,std::uint8_t run,const MotorRpm & rpm) {
  if(nodes!=0xff) return false; // This ROV owns all eight configured nodes.
  for(std::size_t i=0;i<8;++i) {
    const int value=rpm[i],absolute=std::abs(value);
    if(value!=0 && (absolute<1000 || absolute>6000)) return false;
    if(bool(run&(1u<<i))!=(value!=0)) return false;
  }
  return true;
}
Bytes encode_motor_vector(std::uint8_t command,std::uint8_t nodes,std::uint8_t run,
  const MotorRpm & rpm,std::uint16_t sequence) {
  if(command!=0x10 && command!=0x11) throw std::invalid_argument("unsupported motor command");
  if(nodes!=0xff) throw std::invalid_argument("invalid motor node mask");
  for(auto value:rpm) if(value!=0 && (std::abs(int(value))<1000 || std::abs(int(value))>6000))
    throw std::invalid_argument("motor speed out of range");
  Bytes data{1,command,nodes,run}; put16(data,sequence); put16(data,0);
  for(auto value:rpm) put16(data,static_cast<std::uint16_t>(value));
  Bytes packet{0xaa,0x55,32}; put16(packet,sequence);
  packet.insert(packet.end(),{0x00,0x01,0x00,0x00,0x02,24}); // CAN ID 0x100, FD without BRS.
  packet.insert(packet.end(),data.begin(),data.end());
  packet.push_back(crc8(packet.data()+2,packet.size()-2));
  packet.insert(packet.end(),{0x55,0xaa}); return packet; // Legacy DOWNLINK, no timestamp.
}
std::array<Bytes,2> motor_transaction(const MotorRpm & rpm,std::uint8_t run,std::uint16_t & seq) {
  if(!valid_motor_setpoint(0xff,run,rpm)) throw std::invalid_argument("invalid run/RPM pair");
  const MotorRpm zeros{};
  if(run==0) {
    // SPEED_VECTOR(run=0) alone does not stop current firmware. RUN_VECTOR must lead.
    auto stop=encode_motor_vector(0x11,0xff,0,zeros,seq++);
    return {std::move(stop),encode_motor_vector(0x10,0xff,0,zeros,seq++)};
  }
  auto speed=encode_motor_vector(0x10,0xff,0,rpm,seq++);
  return {std::move(speed),encode_motor_vector(0x11,0xff,run,zeros,seq++)};
}
bool MotorGate::accept(std::uint64_t session,std::uint64_t seq,bool armed,std::uint8_t nodes,
  std::uint8_t run,const MotorRpm & rpm,std::uint64_t now) {
  if(!session || !seq || !valid_motor_setpoint(nodes,run,rpm) || (!armed && run!=0)) return false;
  if(session<=retired_session_) return false;
  if(session<session_) return false;
  if(session>session_) {
    // Every new/restarted authority must begin with a zero command handshake.
    if(run!=0) return false;
    session_=session; sequence_=0; blocked_=!armed;
  }
  if(seq<=sequence_) return false;
  if(armed && blocked_) return false; // Expiry needs an explicitly new session.
  sequence_=seq; last_ms_=now; active_=armed && run!=0;
  if(!armed) blocked_=true;
  return true;
}
bool MotorGate::expired(std::uint64_t now,std::uint64_t timeout) {
  if(!blocked_ && now-last_ms_>=timeout) { block(); return true; }
  return false;
}
}
