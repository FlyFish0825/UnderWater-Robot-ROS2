#include "underwater_mcu_gateway/depth_zero.hpp"
#include <cstring>
#include <iostream>
#include <stdexcept>
using namespace underwater::wire;
int checks=0;
void check(bool b) { ++checks; if(!b) throw std::runtime_error("check "+std::to_string(checks)); }
void put32(Bytes & b,std::size_t o,std::uint32_t v) { for(int i=0;i<4;++i)b[o+i]=(v>>(8*i))&255; }
void putf(Bytes & b,std::size_t o,float v) { std::uint32_t bits; std::memcpy(&bits,&v,4); put32(b,o,bits); }
Frame take(DepthZero & z,std::uint8_t cmd) {
  auto p=z.take_request(); check(bool(p)); Frame f; check(decode(*p,f));
  check(f.family==0x5b && f.target==2 && f.flags==1 && f.command==cmd); return f;
}
Frame reply(const Frame & req,Bytes payload,std::uint32_t timestamp) {
  Frame f; f.family=0x5b; f.target=2; f.flags=2; f.command=req.command+0x40;
  f.sequence=req.sequence; f.timestamp_valid=true; f.timestamp_us=timestamp; f.payload=payload; return f;
}
Frame sample(std::uint32_t seq,std::uint32_t time,bool zero) {
  Frame f; f.family=0x5b; f.target=2; f.flags=8; f.command=0x82;
  f.sequence=seq; f.timestamp_valid=true; f.timestamp_us=time; f.payload.resize(32);
  put32(f.payload,0,zero?0x1f3:0x133); // Fixed model needs no confirmation bit.
  putf(f.payload,4,101325); putf(f.payload,8,20);
  if(zero) { putf(f.payload,12,0.0f); putf(f.payload,16,0.0f); putf(f.payload,20,101325); }
  return f;
}
int main() {
 try {
  DepthZero z(100); check(!z.take_request()); check(z.start(0)); check(!z.start(1));
  auto status=take(z,2);
  z.observe(sample(1,900,false),10); check(!z.take_request()); // No sample is accepted before GET_STATUS.
  auto wrong=reply(status,Bytes(21,0),1000); ++wrong.sequence; z.observe(wrong,20); check(!z.take_request());
  z.observe(reply(status,Bytes(21,0),1000),30);
  auto invalid=sample(2,4000,false); put32(invalid.payload,0,0x103);
  z.observe(invalid,60); check(!z.take_request()); // No valid pressure.
  z.observe(sample(3,5000,false),70); auto zero=take(z,0x0c); check(zero.payload.empty());
  z.observe(reply(zero,{0},6000),80); auto read_p0=take(z,3); check(read_p0.payload==Bytes({3,1}));
  Bytes reference{0,3,1,7,4,0,0,0,0}; putf(reference,5,101325);
  z.observe(reply(read_p0,reference,7000),90);
  z.observe(sample(4,6500,true),100); check(!z.take_result());
  z.observe(sample(5,8000,true),110); auto outcome=z.take_result();
  check(outcome && outcome->success && !z.busy()); check(!z.take_request());
  z.observe(sample(6,9000,true),120); check(!z.take_request()); // No automatic re-zero.
  check(z.start(200)); status=take(z,2);
  z.observe(reply(status,Bytes(21,0),10000),210); check(!z.take_request());
  z.observe(sample(7,11000,false),220); zero=take(z,0x0c);
  auto failure=reply(zero,{8},12000); failure.flags=6; z.observe(failure,230);
  outcome=z.take_result(); check(outcome && !outcome->success); check(!z.take_request());
  check(z.start(300)); take(z,2); z.tick(2300); outcome=z.take_result();
  check(outcome && !outcome->success); check(!z.take_request());
  check(z.start(3000)); take(z,2); z.cancel("disconnected"); outcome=z.take_result();
  check(outcome && !outcome->success && !z.take_request());
  std::cout<<"PASS "<<checks<<" depth zero checks\n"; return 0;
 } catch(const std::exception & e) { std::cerr<<e.what()<<"\n"; return 1; }
}
