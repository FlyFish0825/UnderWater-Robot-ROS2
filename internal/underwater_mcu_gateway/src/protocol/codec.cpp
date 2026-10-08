#include "underwater_mcu_gateway/protocol.hpp"
#include <cstring>
#include <stdexcept>
namespace underwater::wire {
std::uint16_t u16(const Bytes & b, std::size_t o) {
  return std::uint16_t(b.at(o)) | (std::uint16_t(b.at(o+1))<<8);
}
std::uint32_t u32(const Bytes & b, std::size_t o) {
  return std::uint32_t(u16(b,o)) | (std::uint32_t(u16(b,o+2))<<16);
}
float f32(const Bytes & b, std::size_t o) {
  const auto bits=u32(b,o); float value;
  static_assert(sizeof(value)==sizeof(bits), "binary32 required");
  std::memcpy(&value,&bits,sizeof(value)); return value;
}
std::uint8_t crc8(const std::uint8_t * p, std::size_t n) {
  std::uint8_t c=0;
  for(std::size_t i=0;i<n;++i) { c^=p[i];
    for(int j=0;j<8;++j) c=(c&0x80)?std::uint8_t((c<<1)^7):std::uint8_t(c<<1);
  } return c;
}
std::uint16_t crc16(const std::uint8_t * p, std::size_t n) {
  std::uint16_t c=0xffff;
  for(std::size_t i=0;i<n;++i) { c^=std::uint16_t(p[i])<<8;
    for(int j=0;j<8;++j) c=(c&0x8000)?std::uint16_t((c<<1)^0x1021):std::uint16_t(c<<1);
  } return c;
}
bool decode(const Bytes & p, Frame & output) {
  if(p.size()<14 || p[0]!=0xaa) return false;
  Frame f; f.family=p[1];
  if(f.family==0x55) {
    const auto body=p[2];
    if(body<8 || body>76 || p.size()!=std::size_t(body)+6 ||
       p[p.size()-2]!=0x55 || p.back()!=0xaa ||
       crc8(p.data()+2,body+1)!=p[body+3]) return false;
    f.sequence=u16(p,3); f.can_id=u32(p,5); f.flags=p[9];
    if(f.flags==0x80) {
      if((body!=17 && body!=21) || p[10]!=0x81) return false;
      f.command=p[10]; f.payload.assign(p.begin()+11,p.begin()+20);
      f.timestamp_valid=body==21; if(f.timestamp_valid) f.timestamp_us=u32(p,20);
    } else {
      const auto n=p[10];
      if(n>64 || (body!=8+n && body!=12+n && body!=72)) return false;
      f.payload.assign(p.begin()+11,p.begin()+11+n);
      f.timestamp_valid=body==12+n;
      if(f.timestamp_valid) f.timestamp_us=u32(p,11+n);
    }
  } else {
    if(p.size()<20 || (f.family!=0x58 && f.family!=0x59 && f.family!=0x5b)) return false;
    const auto n=u16(p,10);
    if(p.size()!=20u+n || p[2]!=1 || p[18+n]!=f.family || p[19+n]!=0xaa ||
       crc16(p.data()+1,15+n)!=u16(p,16+n)) return false;
    if(f.family==0x58 && ((n!=0 && n!=5) || p[3]!=1 || p[4]!=0 || p[5]!=0)) return false;
    if(f.family==0x59 && n>4096) return false;
    if(f.family==0x5b && (n>58 || (p[5]!=1 && p[5]!=2) ||
       (p[4]!=1 && p[4]!=2 && p[4]!=6 && p[4]!=8))) return false;
    f.command=p[3]; f.flags=p[4]; f.target=p[5]; f.sequence=u32(p,6);
    f.timestamp_valid=true; f.timestamp_us=u32(p,12);
    f.payload.assign(p.begin()+16,p.begin()+16+n);
  }
  output=std::move(f); return true;
}
}
