// 电机指令编码与下发闸门实现。
// 下行帧为 0x55 family 的 CAN FD 透传（CAN ID 0x100），两帧成对：SPEED_VECTOR(0x10) + RUN_VECTOR(0x11)。
// 安全约定：非零转速必须在 1000..6000 RPM，run 位与转速严格对应，未使能时只允许零指令。
#include "underwater_mcu_gateway/motor_commands.hpp"
#include <cstdlib>
#include <stdexcept>
namespace underwater::wire {
static void put16(Bytes & b,std::uint16_t v) { b.push_back(v&255); b.push_back(v>>8); } // 小端写入 16 位
bool valid_motor_setpoint(std::uint8_t nodes,std::uint8_t run,const MotorRpm & rpm) {
  if(nodes!=0xff) return false; // This ROV owns all eight configured nodes.
  for(std::size_t i=0;i<8;++i) {
    const int value=rpm[i],absolute=std::abs(value);
    if(value!=0 && (absolute<1000 || absolute>6000)) return false; // 死区外必须有明确转速
    if(bool(run&(1u<<i))!=(value!=0)) return false; // run 位必须与"该轴是否非零"一致
  }
  return true;
}
// 组装一帧 0x55 下行 CAN FD 帧（含 CRC8）；command 只接受 0x10/0x11，nodes 必须为 0xff。
Bytes encode_motor_vector(std::uint8_t command,std::uint8_t nodes,std::uint8_t run,
  const MotorRpm & rpm,std::uint16_t sequence) {
  if(command!=0x10 && command!=0x11) throw std::invalid_argument("unsupported motor command");
  if(nodes!=0xff) throw std::invalid_argument("invalid motor node mask");
  for(auto value:rpm) if(value!=0 && (std::abs(int(value))<1000 || std::abs(int(value))>6000))
    throw std::invalid_argument("motor speed out of range");
  Bytes data{1,command,nodes,run}; put16(data,sequence); put16(data,0); // 首字节 1 = CAN FD 数据帧
  for(auto value:rpm) put16(data,static_cast<std::uint16_t>(value));
  Bytes packet{0xaa,0x55,32}; put16(packet,sequence);
  packet.insert(packet.end(),{0x00,0x01,0x00,0x00,0x02,24}); // CAN ID 0x100, FD without BRS.
  packet.insert(packet.end(),data.begin(),data.end());
  packet.push_back(crc8(packet.data()+2,packet.size()-2));
  packet.insert(packet.end(),{0x55,0xaa}); return packet; // Legacy DOWNLINK, no timestamp.
}
// 生成一对帧并各自占用一个序号（seq 原地递增）；run=0 时先发 RUN_VECTOR 停止再发速度帧。
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
// 电机指令唯一入口：会话单调、序号严格递增、未使能只能是零指令，否则拒绝。
bool MotorGate::accept(std::uint64_t session,std::uint64_t seq,bool armed,std::uint8_t nodes,
  std::uint8_t run,const MotorRpm & rpm,std::uint64_t now) {
  if(!session || !seq || !valid_motor_setpoint(nodes,run,rpm) || (!armed && run!=0)) return false;
  if(session<=retired_session_) return false;
  if(session<session_) return false;
  if(session>session_) { // 新会话：必须以全零指令开局
    // Every new/restarted authority must begin with a zero command handshake.
    if(run!=0) return false;
    session_=session; sequence_=0; blocked_=!armed;
  }
  if(seq<=sequence_) return false; // 重复或乱序序号一律丢弃
  if(armed && blocked_) return false; // Expiry needs an explicitly new session.
  sequence_=seq; last_ms_=now; active_=armed && run!=0;
  if(!armed) blocked_=true; // 未使能指令视为复位：之后必须换会话才能再运动
  return true;
}
// 超过 timeout 未收到新指令即阻塞（停止帧由调用方发送），再次使能需新会话。
bool MotorGate::expired(std::uint64_t now,std::uint64_t timeout) {
  if(!blocked_ && now-last_ms_>=timeout) { block(); return true; }
  return false;
}
}
