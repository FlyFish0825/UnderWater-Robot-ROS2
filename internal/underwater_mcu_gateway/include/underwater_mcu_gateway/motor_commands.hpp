#pragma once
#include "underwater_mcu_gateway/protocol.hpp"
#include <array>
namespace underwater::wire {
using MotorRpm=std::array<std::int16_t,8>;
bool valid_motor_setpoint(std::uint8_t node_mask,std::uint8_t run_mask,const MotorRpm &);
Bytes encode_motor_vector(std::uint8_t command,std::uint8_t node_mask,std::uint8_t run_mask,
  const MotorRpm &,std::uint16_t sequence);
std::array<Bytes,2> motor_transaction(const MotorRpm &,std::uint8_t run_mask,std::uint16_t & sequence);
class MotorGate {
public:
  bool accept(std::uint64_t session,std::uint64_t sequence,bool armed,std::uint8_t nodes,
    std::uint8_t run,const MotorRpm &,std::uint64_t now_ms);
  bool expired(std::uint64_t now_ms,std::uint64_t timeout_ms);
  void reset() {
    // Keep a high-water mark when authority/proof or the serial link changes.
    // A delayed zero from the old session must not undo the required rearm.
    if(session_>retired_session_) retired_session_=session_;
    session_=sequence_=last_ms_=0; blocked_=true; active_=false;
  }
  void block() { blocked_=true; active_=false; }
  std::uint64_t session() const { return session_; }
  bool blocked() const { return blocked_; }
  bool active() const { return active_; }
private:
  std::uint64_t session_=0,sequence_=0,last_ms_=0;
  std::uint64_t retired_session_=0;
  bool blocked_=true,active_=false;
};
}
