#pragma once
#include "underwater_control/remote_manual.hpp"
#include <string_view>

namespace underwater::control::remote {
// ROS graph is trusted locally; network authority is checked independently by
// the gateway using a private 9002 lease, never a public status-topic token.
inline bool authorized_source(std::string_view owner, std::size_t manual_count,
    std::size_t wrench_count, bool only_remote_manual) {
  if(owner=="NANO") return manual_count+wrench_count<=1;
  return owner=="GUI_MANUAL" && manual_count==1 && wrench_count==0 && only_remote_manual;
}
class LeaseProof {
public:
  std::uint64_t begin(Millis at) { pending_=true; started_=at; return ++request_; }
  void cancel_request() { ++request_; pending_=false; }
  void clear() { ++request_; pending_=false; generation_=0; confirmed_=false; }
  bool complete(std::uint64_t request,bool valid,std::uint64_t generation,Millis at) {
    if(!pending_ || request!=request_) return false;
    pending_=false;
    if(!valid || !generation || (generation_ && generation_!=generation) || at-started_>=400) {
      clear(); return false;
    }
    generation_=generation; confirmed_=true; refreshed_=at; return true;
  }
  bool valid(Millis at) const { return confirmed_ && at>=refreshed_ && at-refreshed_<400; }
  bool pending() const { return pending_; }
  bool expired_request(Millis at) const { return pending_ && at-started_>=400; }
private:
  std::uint64_t request_=0,generation_=0;
  Millis started_=0,refreshed_=0;
  bool pending_=false,confirmed_=false;
};
} // namespace underwater::control::remote
