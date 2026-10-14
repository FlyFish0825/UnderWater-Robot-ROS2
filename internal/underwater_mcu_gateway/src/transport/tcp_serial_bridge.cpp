// 原始串口透传 TCP 服务端实现：单客户端、非阻塞、字节级透明转发。
// 仅当控制权属于 GUI 且客户端握手通过时，收到的数据才会交给调用方写入串口。
// 所有失败路径都走 drop()：断开连接并记录 detail()，由节点决定后续所有权处理。
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
namespace {constexpr std::size_t max_pending=256*1024;} // 待发缓冲上限，防止慢客户端拖垮网关
TcpSerialBridge::~TcpSerialBridge(){close();}
// 绑定并监听 IPv4 地址；port 传 0 时由内核分配，结果写回 port_。
void TcpSerialBridge::open(const std::string &address,std::uint16_t port) {
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
  port_=ntohs(endpoint.sin_port);disconnected_=false;detail_="listening; waiting for serial";
}
// 关闭监听与客户端，并把所有会话状态复位为初始值（owner 回到 Nano）。
void TcpSerialBridge::close() noexcept {
  if(client_>=0)::close(client_);
  if(listener_>=0)::close(listener_);
  client_=listener_=-1;outgoing_.clear();offset_=0;ready_=false;port_=0;disconnected_=false;
  authenticated_=false;handshake_.clear();token_.clear();owner_=ControllerOwner::Nano;write_disconnected_=false;
}
// 统一的断开入口：若断掉的是已认证的 GUI 会话且是 holder_disconnect，额外置位 write_disconnected_。
void TcpSerialBridge::drop(const std::string &reason,bool holder_disconnect) {
  if(client_>=0){write_disconnected_|=holder_disconnect && authenticated_ && owner_==ControllerOwner::Gui;::close(client_);client_=-1;disconnected_=true;}
  outgoing_.clear();offset_=0;authenticated_=false;handshake_.clear();detail_=reason;
}
void TcpSerialBridge::set_controller(const ControllerOwner owner,const std::string &token) {
  // A business lease is never a raw UART credential. Normalize before comparing
  // so periodic business-proof expiry does not drop a read-only data client.
  const auto raw_token=owner==ControllerOwner::Gui?token:std::string{};
  if(owner_==owner && token_==raw_token)return;
  // Closing an obsolete credential is intentional revocation, not loss of the
  // replacement owner. Pending disconnects also belong to the previous lease.
  if(client_>=0)drop("Control owner changed; reconnect to apply the new ownership",false);
  write_disconnected_=false;
  owner_=owner;token_=raw_token;
}
void TcpSerialBridge::set_serial_ready(bool ready) {
  ready_=ready;
  if(!ready)drop("serial unavailable; client must reconnect");
}
// 读取并清零标志，保证每个事件只被消费一次。
bool TcpSerialBridge::take_disconnect(){return std::exchange(disconnected_,false);}
bool TcpSerialBridge::take_write_disconnect(){return std::exchange(write_disconnected_,false);}
// 单次最多发送 64KiB；EAGAIN 直接返回，剩余数据留待下一次 poll。
void TcpSerialBridge::flush() {
  if(client_<0)return;
  std::size_t budget=65536; // 单轮发送预算，避免长时间占用线程
  while(offset_<outgoing_.size() && budget) {
    const auto n=::send(client_,outgoing_.data()+offset_,std::min(budget,outgoing_.size()-offset_),MSG_NOSIGNAL);
    if(n>0){offset_+=static_cast<std::size_t>(n);budget-=static_cast<std::size_t>(n);tx_bytes_+=n;continue;}
    if(n<0 && errno==EINTR)continue;
    if(n<0 && (errno==EAGAIN || errno==EWOULDBLOCK))return;
    drop("TCP send failed");return;
  }
  if(offset_==outgoing_.size()){outgoing_.clear();offset_=0;}
}
// 单轮：accept（≤4 次）、recv（≤4 次）、握手与 flush。
wire::Bytes TcpSerialBridge::poll() {
  wire::Bytes incoming;
  if(listener_<0)return incoming;
  // Bound accept/read work so a client cannot starve ROS or serial processing.
  for(int i=0;i<4;++i) {
    const int candidate=::accept4(listener_,nullptr,nullptr,SOCK_NONBLOCK|SOCK_CLOEXEC);
    if(candidate<0)break;
    if(!ready_ || client_>=0 || disconnected_){::close(candidate);continue;} // 只允许一个客户端且串口须就绪
    client_=candidate;++sessions_;int yes=1,seconds=5,interval=2,count=3; // keepalive 5s/2s/3 次
    setsockopt(client_,IPPROTO_TCP,TCP_NODELAY,&yes,sizeof(yes));
    setsockopt(client_,SOL_SOCKET,SO_KEEPALIVE,&yes,sizeof(yes));
    setsockopt(client_,IPPROTO_TCP,TCP_KEEPIDLE,&seconds,sizeof(seconds));
    setsockopt(client_,IPPROTO_TCP,TCP_KEEPINTVL,&interval,sizeof(interval));
    setsockopt(client_,IPPROTO_TCP,TCP_KEEPCNT,&count,sizeof(count));
    int timeout_ms=5000;setsockopt(client_,IPPROTO_TCP,TCP_USER_TIMEOUT,&timeout_ms,sizeof(timeout_ms));
    int buffer=65536;setsockopt(client_,SOL_SOCKET,SO_SNDBUF,&buffer,sizeof(buffer));
    authenticated_=false;handshake_.clear(); // 新会话必须重新握手
    detail_=owner_==ControllerOwner::Gui?"TCP connected; waiting for GUI ownership token":"TCP connected; receive-only";
  }
  if(client_<0)return incoming;
  std::uint8_t chunk[4096];
  for(int i=0;i<4;++i) {
    const auto n=::recv(client_,chunk,sizeof(chunk),0);
    if(n>0) {
      rx_bytes_+=n;
      if(owner_==ControllerOwner::Gui) {
        if(!authenticated_) { // GUI 会话需先收到 "OWNER <token>\n"，此前字节暂存不转发
          handshake_.append(reinterpret_cast<const char *>(chunk),static_cast<std::size_t>(n));
          if(handshake_.size()>128 && handshake_.find('\n')==std::string::npos) { // 无换行且超长：非法握手
            incoming.clear();drop("GUI ownership token missing or too long");return incoming;
          }
          const std::string expected="OWNER "+token_+"\n"; // 已收到的部分必须是它的前缀
          if(handshake_.find('\n')==std::string::npos
             && (handshake_.size()>expected.size()
                 || expected.compare(0,handshake_.size(),handshake_)!=0)) {
            rejected_bytes_+=handshake_.size();incoming.clear();drop("TCP control is owned by another client");return incoming;
          }
          const auto newline=handshake_.find('\n');
          if(newline!=std::string::npos) {
            std::string line=handshake_.substr(0,newline);
            if(!line.empty() && line.back()=='\r')line.pop_back();
            if(line!="OWNER "+token_) {
              rejected_bytes_+=handshake_.size();incoming.clear();drop("TCP control is owned by another client");return incoming;
            }
            authenticated_=true;detail_="TCP connected; GUI owns exclusive serial TX"; // 握手完成，此后字节直接透传
            static constexpr char accepted[]="OK GUI\n";
            if(::send(client_,accepted,sizeof(accepted)-1,MSG_NOSIGNAL)!=static_cast<ssize_t>(sizeof(accepted)-1)) {
              incoming.clear();drop("TCP control handshake reply failed");return incoming;
            }
            handshake_.erase(0,newline+1);
            incoming.insert(incoming.end(),handshake_.begin(),handshake_.end());
            handshake_.clear();
          }
        } else incoming.insert(incoming.end(),chunk,chunk+n);
      } else rejected_bytes_+=n; // 控制权不属于 GUI：收到的数据一律丢弃并计数
      continue;
    }
    if(n<0 && errno==EINTR){--i;continue;}
    if(n<0 && (errno==EAGAIN || errno==EWOULDBLOCK))break;
    // Do not deliver commands received immediately before a dead session was detected.
    incoming.clear();drop("TCP client disconnected");return incoming;
  }
  flush();
  if(client_<0)incoming.clear(); // 本轮内若连接已断，不返回任何已收字节
  return incoming;
}
// 把串口 RX 排入 TCP 发送缓冲；GUI 会话未认证时不外发，避免泄露串口数据。
void TcpSerialBridge::forward_rx(const wire::Bytes &bytes) {
  if(client_<0 || bytes.empty() || (owner_==ControllerOwner::Gui && !authenticated_))return;
  if(outgoing_.size()-offset_+bytes.size()>max_pending){drop("TCP client too slow; pending data discarded");return;} // 积压超限：断开慢客户端
  if(offset_){outgoing_.erase(outgoing_.begin(),outgoing_.begin()+offset_);offset_=0;} // 先回收已发送部分再追加
  outgoing_.insert(outgoing_.end(),bytes.begin(),bytes.end());flush();
}
}
