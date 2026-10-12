#if __has_include("underwater_control/remote_manual.hpp")
#include "underwater_control/remote_manual.hpp"
#include "underwater_control/manual_authority.hpp"
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <limits>
namespace rm = underwater::control::remote;
static int checks = 0;
#define CHECK(x) do { ++checks; if (!(x)) { std::cerr << __LINE__ << ": " #x "\n"; std::exit(1); } } while(false)
static rm::Observation ready() {
  rm::Observation o; o.gateway_ready=true; o.allocation_fresh=true;
  o.connection=42; o.allocation_session=10; o.gateway_session=10;
  return o;
}
static void enabled(rm::Session &s, rm::Observation &o) {
  s.connect(); s.observe(o,0);
  CHECK(s.handle({rm::Kind::Enable,1,0,{}},0,true).ok);
  CHECK(s.take_actions().enable);
  CHECK(s.state()==rm::State::Enabling);
  s.enable_result(true,"accepted",10);
  CHECK(s.take_actions().zero);
  // Service success alone is not the motor-session zero acknowledgement.
  CHECK(s.state()==rm::State::Enabling);
  o.allocation_session=11; o.armed=true; o.input_fresh=true; o.solver_ok=true;
  o.gateway_session=11; o.rearm_required=false;
  s.observe(o,20);
  CHECK(s.state()==rm::State::Enabled);
}
int main() {
  std::string error;
  auto c=rm::parse("INPUT 12 9007199254740993 0 -100 0 0 0 0",error);
  CHECK(c && c->session==9007199254740993ULL && c->levels[1]==-100);
  CHECK(!rm::parse("ENABLE 1",error)); // v1 cannot bypass lease authentication.
  c=rm::parse("AUTH 1 0123456789abcdef0123456789abcdef",error);
  CHECK(c && c->kind==rm::Kind::Auth);
  CHECK(!rm::parse("AUTH 1",error));
  c=rm::parse("ENABLE 1 0123456789abcdef0123456789abcdef",error);
  CHECK(c && c->lease_token=="0123456789abcdef0123456789abcdef");
  CHECK(!rm::parse("ENABLE 1 0123456789abcdef0123456789abcdeZ",error));
  {
    rm::LeaseProof proof;
    CHECK(!proof.valid(0));
    const auto first=proof.begin(0);
    CHECK(proof.pending() && !proof.valid(0));
    proof.clear();
    CHECK(!proof.complete(first,true,7,10)); // cancelled async response.
    const auto second=proof.begin(20);
    CHECK(proof.complete(second,true,7,30) && proof.valid(30));
    const auto cancelled_renewal=proof.begin(40);
    proof.cancel_request();
    CHECK(!proof.pending() && proof.valid(41));
    const auto fresh_renewal=proof.begin(42);
    CHECK(!proof.complete(cancelled_renewal,true,7,43) && proof.pending());
    CHECK(proof.complete(fresh_renewal,true,7,44));
    CHECK(proof.valid(443) && !proof.valid(444));
    const auto renewal=proof.begin(50);
    CHECK(!proof.complete(renewal,true,8,60) && !proof.valid(60)); // rotated ownership.
    const auto third=proof.begin(100);
    CHECK(proof.expired_request(500));
    proof.clear();
    CHECK(!proof.complete(third,true,8,501));
    CHECK(!rm::authorized_source("GUI",1,0,true));
    CHECK(!rm::authorized_source("GUI_MANUAL",1,0,false));
    CHECK(!rm::authorized_source("GUI_MANUAL",1,1,true));
    CHECK(rm::authorized_source("GUI_MANUAL",1,0,true));
    CHECK(rm::authorized_source("NANO",1,0,false));
    CHECK(!rm::authorized_source("NONE",0,0,false));
  }
  CHECK(rm::parse("DISABLE 18446744073709551615",error));
  for(const auto *bad:{"ENABLE 0","ENABLE -1","ENABLE +1","ENABLE 1 extra",
      "ENABLE 18446744073709551616","INPUT 1 1 1.0 0 0 0 0 0",
      "INPUT 1 1 101 0 0 0 0 0","INPUT 1 1 1 1 0 0 0 0",
      "INPUT 1 0 0 0 0 0 0 0","INPUT 1 1 nan 0 0 0 0 0","HELLO 1"})
    CHECK(!rm::parse(bad,error));
  CHECK(!rm::parse(std::string("ENABLE 1\0",9),error));
  rm::LineBuffer b; std::vector<std::string> lines;
  CHECK(b.append("ENA",lines,error)); CHECK(lines.empty());
  CHECK(b.append("BLE 1\nDISABLE 2\r\n",lines,error));
  CHECK(lines.size()==2 && lines[0]=="ENABLE 1" && lines[1]=="DISABLE 2");
  b.reset(); lines.clear(); CHECK(!b.append(std::string(1025,'x'),lines,error));
  b.reset(); CHECK(!b.append(std::string(4097,'x'),lines,error));
  b.reset(); CHECK(!b.append(std::string("ENABLE 1\0\n",10),lines,error));
  {
    rm::Session s; auto o=ready(); s.connect(); s.observe(o,0);
    s.disconnect(1); CHECK(s.state()==rm::State::Fault);
    s.clear_completed_fault(); CHECK(s.state()==rm::State::Disabled);
    s.connect(); CHECK(s.state()==rm::State::Disabled && !s.owns_publisher());
    CHECK(!s.take_actions().enable); // Reconnect clears a completed fault, never auto-starts.
    CHECK(!s.handle({rm::Kind::Enable,1,0,{}},0,false).ok);
    CHECK(!s.owns_publisher()); CHECK(!s.take_actions().enable);
    CHECK(s.handle({rm::Kind::Disable,2,0,{}},0,false).ok);
    CHECK(!s.take_actions().disable); // Never disarm somebody else's terminal session.
  }
  {
    rm::Session s; auto o=ready(); enabled(s,o);
    CHECK(s.handle({rm::Kind::Input,2,11,{25,0,0,0,0,0}},30,true).ok);
    CHECK(s.take_actions().input); CHECK(s.levels()[0]==25);
    s.tick(400); CHECK(s.state()==rm::State::Enabled);
    CHECK(!s.take_actions().input); // Timer must never refresh a cached target.
    s.tick(530); CHECK(s.state()==rm::State::Stopping);
    CHECK(s.levels()==rm::Levels{}); CHECK(s.take_actions().disable);
    s.disable_result(true,540); o.armed=false; s.observe(o,550);
    CHECK(!s.owns_publisher()); CHECK(s.state()==rm::State::Fault);
    CHECK(s.take_actions().release);
  }
  for(int reason=0;reason<7;++reason) {
    rm::Session s; auto o=ready(); enabled(s,o);
    CHECK(s.handle({rm::Kind::Input,2,11,{0,0,-1,0,0,0}},30,true).ok);
    s.take_actions();
    if(reason==0) CHECK(!s.handle({rm::Kind::Input,2,11,{}},40,true).ok);
    if(reason==1) CHECK(!s.handle({rm::Kind::Input,3,10,{}},40,true).ok);
    if(reason==2) CHECK(!s.handle({rm::Kind::Input,3,11,{1,1,0,0,0,0}},40,true).ok);
    if(reason==3) s.disconnect(40);
    if(reason==4) {o.gateway_ready=false; s.observe(o,40);}
    if(reason==5) {o.connection=43; s.observe(o,40);}
    if(reason==6) s.stop("TCP output backpressure",true,40);
    CHECK(s.state()==rm::State::Stopping); CHECK(s.levels()==rm::Levels{});
    const auto a=s.take_actions(); CHECK(a.zero && a.disable);
  }
  {
    rm::Session s; auto o=ready(); s.connect(); s.observe(o,0);
    CHECK(s.handle({rm::Kind::Enable,1,0,{}},0,true).ok); s.take_actions();
    s.disconnect(10); CHECK(s.take_actions().disable);
    s.disable_result(true,15); // A late enable must invalidate this earlier stop ACK.
    s.enable_result(true,"delayed enable",100);
    const auto a=s.take_actions(); CHECK(a.zero && a.disable);
    CHECK(s.state()==rm::State::Stopping); CHECK(s.owns_publisher());
    s.connect(); CHECK(!s.handle({rm::Kind::Enable,1,0,{}},110,true).ok);
    s.disable_result(true,120); o.armed=false; s.observe(o,130);
    CHECK(!s.owns_publisher());
  }
  {
    rm::Session s; auto o=ready(); s.connect(); s.observe(o,0);
    s.handle({rm::Kind::Enable,1,0,{}},0,true); s.take_actions();
    s.stop("enable result unknown",true,10); s.take_actions();
    s.disable_result(false,20); s.tick(520);
    CHECK(s.take_actions().disable); // Compensating stop retries even while enable outcome is unknown.
    s.disable_result(true,530); s.observe(o,540);
    CHECK(s.owns_publisher()); CHECK(s.state()==rm::State::Stopping);
  }
  {
    rm::Session s; auto o=ready(); s.connect(); s.observe(o,0);
    s.handle({rm::Kind::Enable,1,0,{}},0,true); s.take_actions(); s.disconnect(10); s.take_actions();
    const auto previous=s.disable_generation();
    s.enable_result(true,"late success",20); s.take_actions();
    s.disable_result(true,30,previous); s.observe(o,40);
    CHECK(s.owns_publisher()); CHECK(s.state()==rm::State::Stopping);
    s.disable_result(true,50,s.disable_generation()); s.observe(o,60);
    CHECK(!s.owns_publisher());
  }
  {
    rm::Session s; auto o=ready(); s.connect(); s.observe(o,0);
    s.handle({rm::Kind::Enable,1,0,{}},0,true); s.take_actions();
    s.tick(2500); CHECK(s.state()==rm::State::Stopping);
    // Keep ownership until a late response is compensated, not merely until timeout.
    CHECK(s.owns_publisher()); s.enable_result(true,"late",2600);
    CHECK(s.take_actions().disable);
    s.disable_result(false,2610); s.tick(3110); CHECK(s.take_actions().disable);
  }
  {
    rm::Session s; auto o=ready(); enabled(s,o);
    for(int i=2;i<50;++i) {
      CHECK(s.handle({rm::Kind::Input,static_cast<std::uint64_t>(i),11,{0,0,0,0,0,-100}},i*50,true).ok);
      CHECK(s.take_actions().input); s.tick(i*50+49); CHECK(s.state()==rm::State::Enabled);
    }
    CHECK(s.levels()[5]==-100);
    auto wire=rm::state_line(s,1,o);
    std::istringstream stream(wire); std::string token; int count=0;
    while(stream>>token) ++count;
    CHECK(count==24); CHECK(wire.back()=='\n');
    CHECK(rm::result_line({12,true,11,""})=="RESULT 12 1 11 -\n");
    CHECK(rm::hex_detail("a b")=="612062");
    CHECK(rm::hex_detail(std::string(255,'a')+u8"机器人")==rm::hex_detail(std::string(255,'a')));
  }
  {
    rm::Session s; auto o=ready(); s.connect(); s.observe(o,0);
    s.handle({rm::Kind::Enable,1,0,{}},0,true); s.take_actions();
    s.enable_result(true,"ok",10); s.take_actions();
    o.allocation_session=11; o.gateway_session=11; o.armed=true;
    o.rearm_required=false; // Initial inputFresh=false must not cause a circular wait.
    s.observe(o,20); CHECK(s.state()==rm::State::Enabled);
    CHECK(s.handle({rm::Kind::Input,2,11,{}},30,true).ok);
  }
  {
    rm::OutputBuffer out;
    CHECK(out.push("HELLO ROV_MANUAL 1\n",0));
    CHECK(!out.stalled(499)); CHECK(out.stalled(500));
    out.consume(5); CHECK(out.bytes()==" ROV_MANUAL 1\n");
    out.consume(out.bytes().size()); CHECK(!out.stalled(999));
    CHECK(out.push(std::string(4096,'x'),1000));
    CHECK(!out.push("x",1001)); CHECK(out.bytes().size()==4096);
    out.reset(); CHECK(out.bytes().empty());
  }
  {
    rm::Session s; auto o=ready(); enabled(s,o);
    o.solver_ok=false; s.observe(o,25);
    CHECK(s.handle({rm::Kind::Input,2,11,{1,0,0,0,0,0}},30,true).ok);
    s.observe(o,31); CHECK(s.state()==rm::State::Enabled); // Old pre-input status is not a new solve failure.
    o.allocation_update=1; s.observe(o,40);
    CHECK(s.state()==rm::State::Stopping);
  }
  {
    rm::Session s; auto o=ready(); s.connect(); s.observe(o,0);
    s.handle({rm::Kind::Enable,1,0,{}},0,true); s.take_actions(); s.disconnect(10); s.take_actions();
    s.enable_result(false,"rejected",20);
    CHECK(s.owns_publisher()); // A dispatched stop must finish before ownership can be reused.
    s.disable_result(true,30); o.armed=false; s.observe(o,40);
    CHECK(!s.owns_publisher());
  }
  std::cout << "remote_manual: " << checks << " checks passed\n";
}
#else
#include <iostream>
int main() { std::cerr << "FAIL: remote manual protocol/session implementation is missing\n"; return 1; }
#endif
