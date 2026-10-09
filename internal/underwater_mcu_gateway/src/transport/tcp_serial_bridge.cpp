#include "underwater_mcu_gateway/tcp_serial_bridge.hpp"
#include <algorithm>
#include <arpa/inet.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cerrno>
#include <stdexcept>
#include <utility>

namespace underwater {
namespace {constexpr std::size_t max_pending=256*1024;}
TcpSerialBridge::~TcpSerialBridge(){close();}
void TcpSerialBridge::open(const std::string &address,std::uint16_t port,bool allow_tx) {
  close();sockaddr_in endpoint{};endpoint.sin_family=AF_INET;endpoint.sin_port=htons(port);
  if(inet_pton(AF_INET,address.c_str(),&endpoint.sin_addr)!=1)throw std::invalid_argument("tcp_bind_address must be an IPv4 address");
  listener_=::socket(AF_INET,SOCK_STREAM|SOCK_NONBLOCK|SOCK_CLOEXEC,0);
  if(listener_<0)throw std::runtime_error("TCP socket creation failed");
  int yes=1;setsockopt(listener_,SOL_SOCKET,SO_REUSEADDR,&yes,sizeof(yes));
  if(::bind(listener_,reinterpret_cast<sockaddr *>(&endpoint),sizeof(endpoint))!=0 || ::listen(listener_,4)!=0) {
    close();throw std::runtime_error("TCP bind/listen failed; check address and port ownership");
  }
  socklen_t length=sizeof(endpoint);
  if(getsockname(listener_,reinterpret_cast<sockaddr *>(&endpoint),&length)!=0){close();throw std::runtime_error("TCP bound port unavailable");}
  port_=ntohs(endpoint.sin_port);allow_tx_=allow_tx;disconnected_=false;detail_="listening; waiting for serial";
}
void TcpSerialBridge::close() noexcept {
  if(client_>=0)::close(client_);
  if(listener_>=0)::close(listener_);
  client_=listener_=-1;outgoing_.clear();offset_=0;ready_=false;port_=0;disconnected_=false;
}
void TcpSerialBridge::drop(const std::string &reason) {
  if(client_>=0){::close(client_);client_=-1;disconnected_=true;}
  outgoing_.clear();offset_=0;detail_=reason;
}
void TcpSerialBridge::set_serial_ready(bool ready) {
  ready_=ready;
  if(!ready)drop("serial unavailable; client must reconnect");
}
bool TcpSerialBridge::take_disconnect(){return std::exchange(disconnected_,false);}
void TcpSerialBridge::flush() {
  if(client_<0)return;
  std::size_t budget=65536;
  while(offset_<outgoing_.size() && budget) {
    const auto n=::send(client_,outgoing_.data()+offset_,std::min(budget,outgoing_.size()-offset_),MSG_NOSIGNAL);
    if(n>0){offset_+=static_cast<std::size_t>(n);budget-=static_cast<std::size_t>(n);tx_bytes_+=n;continue;}
    if(n<0 && errno==EINTR)continue;
    if(n<0 && (errno==EAGAIN || errno==EWOULDBLOCK))return;
    drop("TCP send failed");return;
  }
  if(offset_==outgoing_.size()){outgoing_.clear();offset_=0;}
}
wire::Bytes TcpSerialBridge::poll() {
  wire::Bytes incoming;
  if(listener_<0)return incoming;
  // Bound accept/read work so a client cannot starve ROS or serial processing.
  for(int i=0;i<4;++i) {
    const int candidate=::accept4(listener_,nullptr,nullptr,SOCK_NONBLOCK|SOCK_CLOEXEC);
    if(candidate<0)break;
    if(!ready_ || client_>=0 || disconnected_){::close(candidate);continue;}
    client_=candidate;++sessions_;int yes=1,seconds=5,interval=2,count=3;
    setsockopt(client_,IPPROTO_TCP,TCP_NODELAY,&yes,sizeof(yes));
    setsockopt(client_,SOL_SOCKET,SO_KEEPALIVE,&yes,sizeof(yes));
    setsockopt(client_,IPPROTO_TCP,TCP_KEEPIDLE,&seconds,sizeof(seconds));
    setsockopt(client_,IPPROTO_TCP,TCP_KEEPINTVL,&interval,sizeof(interval));
    setsockopt(client_,IPPROTO_TCP,TCP_KEEPCNT,&count,sizeof(count));
    int timeout_ms=5000;setsockopt(client_,IPPROTO_TCP,TCP_USER_TIMEOUT,&timeout_ms,sizeof(timeout_ms));
    int buffer=65536;setsockopt(client_,SOL_SOCKET,SO_SNDBUF,&buffer,sizeof(buffer));
    detail_=allow_tx_?"TCP connected; exclusive remote TX":"TCP connected; receive-only";
  }
  if(client_<0)return incoming;
  std::uint8_t chunk[4096];
  for(int i=0;i<4;++i) {
    const auto n=::recv(client_,chunk,sizeof(chunk),0);
    if(n>0) {
      rx_bytes_+=n;
      if(allow_tx_)incoming.insert(incoming.end(),chunk,chunk+n);else rejected_bytes_+=n;
      continue;
    }
    if(n<0 && errno==EINTR){--i;continue;}
    if(n<0 && (errno==EAGAIN || errno==EWOULDBLOCK))break;
    // Do not deliver commands received immediately before a dead session was detected.
    incoming.clear();drop("TCP client disconnected");return incoming;
  }
  flush();
  if(client_<0)incoming.clear();
  return incoming;
}
void TcpSerialBridge::forward_rx(const wire::Bytes &bytes) {
  if(client_<0 || bytes.empty())return;
  if(outgoing_.size()-offset_+bytes.size()>max_pending){drop("TCP client too slow; pending data discarded");return;}
  if(offset_){outgoing_.erase(outgoing_.begin(),outgoing_.begin()+offset_);offset_=0;}
  outgoing_.insert(outgoing_.end(),bytes.begin(),bytes.end());flush();
}
}
