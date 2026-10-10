// stream_server.cpp —— 双目图像 TCP 推流服务实现（默认端口 9001，UWSC 40 字节帧头 + JPEG）。
// 行为约定：只发不收（客户端指令一律忽略）；每个客户端只发“最新完整帧”，不跨帧拼接；
//          客户端上限 4 个；单帧发送超过 2 秒未完成即断开，避免慢客户端长期占用内存。
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
// 创建非阻塞监听 socket 并绑定 address:port（port=0 由内核分配），随后启动推送线程。
StreamServer::StreamServer(const std::string &address,std::uint16_t port) {
  listener_=socket(AF_INET,SOCK_STREAM|SOCK_NONBLOCK|SOCK_CLOEXEC,0);
  if(listener_<0) throw std::runtime_error("camera TCP socket: "+std::string(strerror(errno)));
  int reuse=1; setsockopt(listener_,SOL_SOCKET,SO_REUSEADDR,&reuse,sizeof(reuse));
  sockaddr_in a{};a.sin_family=AF_INET;a.sin_port=htons(port);  // htons：端口转网络字节序
  if(inet_pton(AF_INET,address.c_str(),&a.sin_addr)!=1 ||
     bind(listener_,reinterpret_cast<sockaddr*>(&a),sizeof(a))<0 || listen(listener_,4)<0) {  // backlog=4，与客户端上限一致
    const auto reason=std::string(strerror(errno));close(listener_);listener_=-1;
    throw std::runtime_error("camera TCP bind/listen "+address+":"+std::to_string(port)+": "+reason);
  }
  socklen_t size=sizeof(a);getsockname(listener_,reinterpret_cast<sockaddr*>(&a),&size);port_=ntohs(a.sin_port);
  try { thread_=std::thread(&StreamServer::run,this); }
  catch(...) { close(listener_);listener_=-1;throw; }
}
StreamServer::~StreamServer() {  // 置停止标志 -> 等推送线程退出 -> 关闭监听 fd
  stopping_=true;
  if(thread_.joinable())thread_.join();
  if(listener_>=0)close(listener_);
}
// 组装 header_size 字节帧头再拼接 JPEG 原始字节，整包替换 latest_（旧帧直接丢弃）。
void StreamServer::publish(std::uint64_t sequence,std::uint64_t receipt_ns,
    std::uint32_t width,std::uint32_t height,const std::vector<std::uint8_t>& jpeg) {
  const auto h=make_header(sequence,receipt_ns,width,height,jpeg.size());
  auto packet=std::make_shared<Packet>();packet->sequence=sequence;
  packet->bytes.reserve(header_size+jpeg.size());
  packet->bytes.insert(packet->bytes.end(),h.begin(),h.end());
  packet->bytes.insert(packet->bytes.end(),jpeg.begin(),jpeg.end());
  std::lock_guard<std::mutex> lock(mutex_);latest_=std::move(packet);
}
void StreamServer::clear() {  // 丢弃待发帧并递增 generation，触发推送线程断开所有客户端
  std::lock_guard<std::mutex> lock(mutex_);latest_.reset();++generation_;
}
void StreamServer::run() {  // 推送线程主循环：poll 同时处理新连接、客户端读事件与帧发送
  using Clock=std::chrono::steady_clock;
  struct Client {
    int fd;
    std::shared_ptr<const Packet> packet;  // 当前正在发送的整帧
    std::size_t offset=0;                  // 已发送字节数
    std::uint64_t completed=0;             // 最近完整发完的帧序号，用于跳过旧帧
    Clock::time_point started;             // 当前帧开始发送的时刻（超时判定用）
  };
  std::vector<Client> peers;
  std::uint64_t generation=0;
  while(!stopping_) {
    std::shared_ptr<const Packet> latest;
    std::uint64_t next_generation;
    { std::lock_guard<std::mutex> lock(mutex_);latest=latest_;next_generation=generation_; }
    if(next_generation!=generation) {  // clear() 后：关闭全部旧客户端，避免发送陈旧画面
      for(auto &c:peers)close(c.fd);
      peers.clear();clients_=0;generation=next_generation;
    }
    std::vector<pollfd> events{{listener_,POLLIN,0}};
    for(auto &c:peers) {
      if(latest && latest->sequence>c.completed && (!c.packet || c.offset==0)) {  // 仅在上一帧发完时换新帧，保证整帧连续
        if(!c.packet)c.started=Clock::now();
        c.packet=latest;
      }
      events.push_back({c.fd,static_cast<short>(POLLIN|(c.packet?POLLOUT:0)),0});
    }
    const int ready=poll(events.data(),events.size(),20);  // 20ms 超时，兼顾停止响应与 CPU 占用
    if(ready<0) { if(errno==EINTR)continue;healthy_=false;break; }
    for(std::size_t i=peers.size();i>0;--i) {  // 倒序遍历，便于 erase
      auto &c=peers[i-1];const auto revents=events[i].revents;
      bool dead=(revents&(POLLERR|POLLHUP|POLLNVAL))!=0;
      // This is a receive-only stream. Client commands are deliberately unsupported.
      if(revents&POLLIN) {  // 收到任何数据（含主动关闭）即视为该客户端退出
        char data[64];const auto n=recv(c.fd,data,sizeof(data),0);
        if(n>=0 || (errno!=EAGAIN && errno!=EWOULDBLOCK && errno!=EINTR))dead=true;
      }
      if(!dead && c.packet && (revents&POLLOUT)) {
        const auto &bytes=c.packet->bytes;
        const auto n=send(c.fd,bytes.data()+c.offset,bytes.size()-c.offset,MSG_NOSIGNAL);  // MSG_NOSIGNAL：对端断开不触发 SIGPIPE
        if(n>0) {
          c.offset+=static_cast<std::size_t>(n);
          if(c.offset==bytes.size()) {  // 整帧发完才计入 frames_sent_
            c.completed=c.packet->sequence;c.packet.reset();c.offset=0;++frames_sent_;
          }
        } else if(n==0 || (errno!=EAGAIN && errno!=EWOULDBLOCK && errno!=EINTR))dead=true;
      }
      // A partial frame must complete or the connection closes; never splice JPEG frames.
      if(c.packet && Clock::now()-c.started>std::chrono::seconds(2))dead=true;  // 单帧 2 秒发送超时
      if(dead) { close(c.fd);peers.erase(peers.begin()+static_cast<std::ptrdiff_t>(i-1)); }
    }
    if(events[0].revents&POLLIN) {
      for(;;) {
        int fd=accept4(listener_,nullptr,nullptr,SOCK_NONBLOCK|SOCK_CLOEXEC);  // 客户端同样设为非阻塞
        if(fd<0)break;
        if(peers.size()>=4) { close(fd);continue; }  // 超出 4 个客户端直接拒绝
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
