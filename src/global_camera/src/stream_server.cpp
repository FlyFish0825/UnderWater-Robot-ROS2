#include "global_camera/stream_server.hpp"
#include "global_camera/wire.hpp"
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <stdexcept>
namespace global_camera {
StreamServer::StreamServer(const std::string &address,std::uint16_t port) {
  listener_=socket(AF_INET,SOCK_STREAM|SOCK_NONBLOCK|SOCK_CLOEXEC,0);
  if(listener_<0) throw std::runtime_error("camera TCP socket: "+std::string(strerror(errno)));
  int reuse=1; setsockopt(listener_,SOL_SOCKET,SO_REUSEADDR,&reuse,sizeof(reuse));
  sockaddr_in a{};a.sin_family=AF_INET;a.sin_port=htons(port);
  if(inet_pton(AF_INET,address.c_str(),&a.sin_addr)!=1 ||
     bind(listener_,reinterpret_cast<sockaddr*>(&a),sizeof(a))<0 || listen(listener_,4)<0) {
    const auto reason=std::string(strerror(errno));close(listener_);listener_=-1;
    throw std::runtime_error("camera TCP bind/listen "+address+":"+std::to_string(port)+": "+reason);
  }
  socklen_t size=sizeof(a);getsockname(listener_,reinterpret_cast<sockaddr*>(&a),&size);port_=ntohs(a.sin_port);
  try { thread_=std::thread(&StreamServer::run,this); }
  catch(...) { close(listener_);listener_=-1;throw; }
}
StreamServer::~StreamServer() {
  stopping_=true;
  if(thread_.joinable())thread_.join();
  if(listener_>=0)close(listener_);
}
void StreamServer::publish(std::uint64_t sequence,std::uint64_t receipt_ns,
    std::uint32_t width,std::uint32_t height,const std::vector<std::uint8_t>& jpeg) {
  const auto h=make_header(sequence,receipt_ns,width,height,jpeg.size());
  auto packet=std::make_shared<Packet>();packet->sequence=sequence;
  packet->bytes.reserve(header_size+jpeg.size());
  packet->bytes.insert(packet->bytes.end(),h.begin(),h.end());
  packet->bytes.insert(packet->bytes.end(),jpeg.begin(),jpeg.end());
  std::lock_guard<std::mutex> lock(mutex_);latest_=std::move(packet);
}
void StreamServer::clear() {
  std::lock_guard<std::mutex> lock(mutex_);latest_.reset();++generation_;
}
void StreamServer::run() {
  using Clock=std::chrono::steady_clock;
  struct Client {
    int fd;
    std::shared_ptr<const Packet> packet;
    std::size_t offset=0;
    std::uint64_t completed=0;
    Clock::time_point started;
  };
  std::vector<Client> peers;
  std::uint64_t generation=0;
  while(!stopping_) {
    std::shared_ptr<const Packet> latest;
    std::uint64_t next_generation;
    { std::lock_guard<std::mutex> lock(mutex_);latest=latest_;next_generation=generation_; }
    if(next_generation!=generation) {
      for(auto &c:peers)close(c.fd);
      peers.clear();clients_=0;generation=next_generation;
    }
    std::vector<pollfd> events{{listener_,POLLIN,0}};
    for(auto &c:peers) {
      if(latest && latest->sequence>c.completed && (!c.packet || c.offset==0)) {
        if(!c.packet)c.started=Clock::now();
        c.packet=latest;
      }
      events.push_back({c.fd,static_cast<short>(POLLIN|(c.packet?POLLOUT:0)),0});
    }
    const int ready=poll(events.data(),events.size(),20);
    if(ready<0) { if(errno==EINTR)continue;healthy_=false;break; }
    for(std::size_t i=peers.size();i>0;--i) {
      auto &c=peers[i-1];const auto revents=events[i].revents;
      bool dead=(revents&(POLLERR|POLLHUP|POLLNVAL))!=0;
      // This is a receive-only stream. Client commands are deliberately unsupported.
      if(revents&POLLIN) {
        char data[64];const auto n=recv(c.fd,data,sizeof(data),0);
        if(n>=0 || (errno!=EAGAIN && errno!=EWOULDBLOCK && errno!=EINTR))dead=true;
      }
      if(!dead && c.packet && (revents&POLLOUT)) {
        const auto &bytes=c.packet->bytes;
        const auto n=send(c.fd,bytes.data()+c.offset,bytes.size()-c.offset,MSG_NOSIGNAL);
        if(n>0) {
          c.offset+=static_cast<std::size_t>(n);
          if(c.offset==bytes.size()) {
            c.completed=c.packet->sequence;c.packet.reset();c.offset=0;++frames_sent_;
          }
        } else if(n==0 || (errno!=EAGAIN && errno!=EWOULDBLOCK && errno!=EINTR))dead=true;
      }
      // A partial frame must complete or the connection closes; never splice JPEG frames.
      if(c.packet && Clock::now()-c.started>std::chrono::seconds(2))dead=true;
      if(dead) { close(c.fd);peers.erase(peers.begin()+static_cast<std::ptrdiff_t>(i-1)); }
    }
    if(events[0].revents&POLLIN) {
      for(;;) {
        int fd=accept4(listener_,nullptr,nullptr,SOCK_NONBLOCK|SOCK_CLOEXEC);
        if(fd<0)break;
        if(peers.size()>=4) { close(fd);continue; }
        // JPEG tails must not wait for Nagle/delayed ACK on the Windows client.
        // The capture worker already limits output to the newest complete frame.
        int yes=1;setsockopt(fd,IPPROTO_TCP,TCP_NODELAY,&yes,sizeof(yes));
        // Keep Linux TCP send-buffer autotuning. A fixed 64 KiB buffer caps
        // throughput on a LAN with delayed ACKs even when capture runs at 30 FPS.
        // User-space still holds only latest/current frames and closes a stalled
        // partial frame after two seconds; the kernel queue remains OS-bounded.
        peers.push_back({fd,nullptr,0,0,Clock::now()});
      }
    }
    clients_=static_cast<unsigned>(peers.size());
  }
  for(auto &c:peers)close(c.fd);
  clients_=0;
}
}
