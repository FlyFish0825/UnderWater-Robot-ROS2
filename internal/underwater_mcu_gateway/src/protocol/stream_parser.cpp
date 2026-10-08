#include "underwater_mcu_gateway/protocol.hpp"
#include <algorithm>
namespace underwater::wire {
void Parser::reset() { buffer_.clear(); partial_=false; } // Counters are process lifetime.
std::vector<Frame> Parser::feed(const Bytes & bytes, std::uint64_t now) {
  std::vector<Frame> result;
  if(buffer_.size()+bytes.size()>65536) {
    counters_.discarded+=buffer_.size(); buffer_.clear(); partial_=false;
    if(bytes.size()>65536) { counters_.discarded+=bytes.size(); return result; }
  }
  buffer_.insert(buffer_.end(),bytes.begin(),bytes.end());
  std::size_t cursor=0;
  while(cursor<buffer_.size()) {
    const auto left=buffer_.size()-cursor;
    const auto * head=buffer_.data()+cursor;
    if(head[0]!=0xaa || (left>1 && head[1]!=0x55 &&
        head[1]!=0x58 && head[1]!=0x59 && head[1]!=0x5b)) {
      ++cursor; ++counters_.discarded; partial_=false; continue;
    }
    std::size_t total=0; bool bad_length=false;
    if(left>=3 && head[1]==0x55) {
      const auto n=head[2]; bad_length=n<8 || n>76; total=n+6;
    } else if(left>=16) {
      const auto n=u16(buffer_,cursor+10); total=n+20;
      bad_length=(head[1]==0x58 && n!=0 && n!=5) || (head[1]==0x5b && n>58) || n>4096;
    }
    if(bad_length) {
      ++counters_.shape_errors; ++counters_.discarded; ++cursor; partial_=false; continue;
    }
    if(total==0 || left<total) {
      if(!partial_) { partial_=true; partial_since_=now; }
      if(now-partial_since_>=1000) {
        ++counters_.shape_errors; ++counters_.discarded; ++cursor; partial_=false; continue;
      }
      break;
    }
    Bytes packet(head,head+total); Frame frame;
    if(!decode(packet,frame)) {
      const bool tail=packet[total-2]==packet[1] && packet.back()==0xaa;
      const bool crc=packet[1]==0x55 ?
        crc8(packet.data()+2,total-5)==packet[total-3] :
        crc16(packet.data()+1,total-5)==u16(packet,total-4);
      if(tail && !crc) ++counters_.crc_errors; else ++counters_.shape_errors;
      ++counters_.discarded; ++cursor; partial_=false; continue;
    }
    ++counters_.valid; result.push_back(std::move(frame)); cursor+=total; partial_=false;
  }
  // Compact once, so long noise prefixes are scanned in linear time.
  buffer_.erase(buffer_.begin(),buffer_.begin()+cursor);
  return result;
}
}
