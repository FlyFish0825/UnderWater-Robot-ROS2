#include "underwater_control/remote_tcp.hpp"
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <stdexcept>

namespace underwater::control::remote {
namespace {
bool configure(int fd) {
  const auto flags=fcntl(fd,F_GETFL,0);
  return flags>=0 && fcntl(fd,F_SETFL,flags|O_NONBLOCK)>=0 && fcntl(fd,F_SETFD,FD_CLOEXEC)>=0;
}
bool again() { return errno==EAGAIN || errno==EWOULDBLOCK || errno==EINTR; }
}
TcpServer::TcpServer(const std::string &address,std::uint16_t port) {
  sockaddr_in addr{}; addr.sin_family=AF_INET; addr.sin_port=htons(port);
  if(inet_pton(AF_INET,address.c_str(),&addr.sin_addr)!=1) throw std::invalid_argument("remote bind must be IPv4");
  listener_=socket(AF_INET,SOCK_STREAM,0);
  if(listener_<0) throw std::runtime_error("remote socket failed");
  int one=1;
  const bool ok=configure(listener_) && setsockopt(listener_,SOL_SOCKET,SO_REUSEADDR,&one,sizeof(one))==0 &&
    bind(listener_,reinterpret_cast<sockaddr *>(&addr),sizeof(addr))==0 && listen(listener_,4)==0;
  if(!ok) {
    const std::string error=std::strerror(errno); close(listener_); listener_=-1;
    throw std::runtime_error("remote listen failed: "+error);
  }
}
TcpServer::~TcpServer() { drop(); if(listener_>=0) close(listener_); }
void TcpServer::drop() {
  if(client_>=0) close(client_);
  client_=-1; input_.reset(); output_.reset();
  reserved_=false;
  ever_reserved_=false;
}
bool TcpServer::queue(const std::string &line,Millis at) {
  return connected() && line.size()<=1025 && output_.push(line,at);
}
void TcpServer::poll(Millis at,const Handler &handler) {
  auto failure=[&](const std::string &why) { drop(); handler(Event::Disconnected,why); };
  // Only a gateway-authenticated client may reserve the single controller slot.
  // A read-only/unverified connection cannot indefinitely exclude the owner.
  for(int n=0;n<4;++n) {
    const int fd=accept(listener_,nullptr,nullptr);
    if(fd<0) { if(!again()) throw std::runtime_error("remote accept failed"); break; }
    if(reserved_ || !configure(fd)) { close(fd); continue; }
    int one=1;
    if(setsockopt(fd,IPPROTO_TCP,TCP_NODELAY,&one,sizeof(one))!=0 ||
       setsockopt(fd,SOL_SOCKET,SO_KEEPALIVE,&one,sizeof(one))!=0) { close(fd); continue; }
    if(connected()) failure("unverified connection replaced");
    client_=fd; ++generation_; accepted_at_=at;
    queue("HELLO ROV_MANUAL 2\n",at);
    handler(Event::Connected,"");
  }
  if(!connected()) return;
  if(!ever_reserved_ && at-accepted_at_>=3000) { failure("manual authority handshake timed out"); return; }
  if(output_.stalled(at)) { failure("TCP output stalled for 500 ms"); return; }
  char bytes[4096]; std::size_t received=0,commands=0;
  while(connected() && received<sizeof(bytes)) {
    const auto size=recv(client_,bytes,sizeof(bytes)-received,0);
    if(size==0) { failure("TCP peer closed"); return; }
    if(size<0) { if(again()) break; failure("TCP receive failed"); return; }
    received+=static_cast<std::size_t>(size);
    std::vector<std::string> lines; std::string error;
    if(!input_.append(std::string_view(bytes,static_cast<std::size_t>(size)),lines,error)) { failure(error); return; }
    commands+=lines.size();
    if(commands>32) { failure("TCP command burst exceeded 32 per poll"); return; }
    for(const auto &line:lines) { handler(Event::Line,line); if(!connected()) return; }
  }
  // A full per-tick budget is treated as flooding, not a backlog to timestamp later.
  if(received==sizeof(bytes)) { failure("TCP receive burst exceeded budget"); return; }
  if(connected() && !output_.bytes().empty()) {
    const auto &out=output_.bytes();
    const auto size=send(client_,out.data(),out.size(),MSG_NOSIGNAL);
    if(size>0) output_.consume(static_cast<std::size_t>(size));
    else if(size==0 || !again()) failure("TCP send failed");
  }
}
} // namespace underwater::control::remote
