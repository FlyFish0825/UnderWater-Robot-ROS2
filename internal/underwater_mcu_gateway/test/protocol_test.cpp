#include "underwater_mcu_gateway/protocol.hpp"
#include <iostream>
#include <sstream>
#include <stdexcept>
using namespace underwater::wire;
int checks=0;
void check(bool b) { ++checks; if(!b) throw std::runtime_error("check "+std::to_string(checks)); }
Bytes hex(const char * s) { Bytes b; std::istringstream in(s); unsigned n; while(in>>std::hex>>n) b.push_back(n); return b; }
int main() {
  try {
    const auto heartbeat=hex("aa 58 01 01 00 00 15 00 00 00 05 00 b8 73 f2 b3 00 00 00 00 00 da 21 58 aa");
    const auto can=hex("aa 55 18 3b 01 01 02 00 00 02 0c 00 00 00 00 f3 00 bb 45 00 03 36 00 c8 06 31 00 3f 55 aa");
    const auto info=hex("aa 5b 01 01 01 01 78 56 34 12 00 00 00 00 00 00 52 d6 5b aa");
    Frame f; check(decode(can,f)); check(f.can_id==0x201 && f.payload.size()==12 && f.timestamp_us==3213000);
    check(decode(info,f)); check(f.sequence==0x12345678 && f.command==1 && f.payload.empty());
    check(decode(heartbeat,f)); check(f.sequence==21 && f.payload.size()==5);
    for(const auto & sample:{heartbeat,can,info}) {
      for(std::size_t cut=0;cut<=sample.size();++cut) {
        Parser p; auto a=p.feed(Bytes(sample.begin(),sample.begin()+cut),0);
        auto b=p.feed(Bytes(sample.begin()+cut,sample.end()),1);
        check(a.size()+b.size()==1 && p.buffered()==0);
      }
      for(std::size_t i=0;i<sample.size();++i) for(int bit=0;bit<8;++bit) {
        auto bad=sample; bad[i]^=1u<<bit; Frame untouched; untouched.sequence=99;
        check(!decode(bad,untouched)); check(untouched.sequence==99);
      }
    }
    Parser p; Bytes mixed{1,2,3};
    mixed.insert(mixed.end(),can.begin(),can.end()); mixed.insert(mixed.end(),heartbeat.begin(),heartbeat.end());
    check(p.feed(mixed,0).size()==2); check(p.counters().discarded==3);
    auto damaged=heartbeat; damaged[16]^=1; damaged.insert(damaged.end(),info.begin(),info.end());
    check(p.feed(damaged,1).size()==1 && p.counters().crc_errors==1);
    p.reset(); Bytes stuck{0xaa,0x59,1,0,0,0,0,0,0,0,0,0x10,0,0,0,0};
    stuck.insert(stuck.end(),heartbeat.begin(),heartbeat.end());
    check(p.feed(stuck,10).empty()); check(p.feed({},1011).size()==1);
    // A valid packet owns its entire payload, including nested AA markers.
    Bytes nested{0xaa,0x55,0,0,0,1,2,0,0,2,0};
    nested[10]=info.size(); nested.insert(nested.end(),info.begin(),info.end());
    nested[2]=8+info.size(); nested.push_back(crc8(nested.data()+2,nested.size()-2));
    nested.push_back(0x55); nested.push_back(0xaa);
    check(p.feed(nested,1012).size()==1);
    p.feed(Bytes(100000,0x42),1013); check(p.buffered()<=65536);
    auto noise=Bytes(60000,0x42); noise.insert(noise.end(),heartbeat.begin(),heartbeat.end());
    check(p.feed(noise,1014).size()==1 && p.buffered()==0);
    std::cout<<"PASS "<<checks<<" protocol checks\n"; return 0;
  } catch(const std::exception & e) { std::cerr<<e.what()<<"\n"; return 1; }
}
