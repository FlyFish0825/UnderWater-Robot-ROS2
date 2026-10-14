// ==============================================================================
// internal/underwater_mcu_gateway/src/transport/tcp_serial_bridge.cpp
// ==============================================================================
// 原始串口透传 TCP 服务端实现 (9001 端口)：
// - 单客户端独占连接、非阻塞、字节级透明转发
// - 仅当控制权属于 GUI 且客户端完成 "OWNER <token>" 握手后才允许写串口
// - 所有失败路径都执行 drop()：断开连接并记录 detail()，由主节点收回控制权
// ==============================================================================
// 初学者导读：
// 1) 本文件实现 tcp_serial_bridge.hpp：原始串口字节透传桥。端口由 mcu_gateway_node 的
//    tcp_port 参数决定（bringup 默认 9000；9001 是双目相机 JPEG 端口，别混淆）。
// 2) 交互：poll() 返回要写串口的字节，forward_rx() 把串口收到的字节发回 GUI；
//    写权限依赖 set_controller() 与 "OWNER <token>" 握手。
// 3) ROS 2 概念：无 ROS API，纯 TCP <-> 串口管道。
// 4) C++ 概念：std::exchange 取旧值并重置；TCP keepalive 选项；非阻塞 socket 与 EAGAIN；
//    std::min 限流；reinterpret_cast 把字节缓冲当字符缓冲使用。
#include "underwater_mcu_gateway/tcp_serial_bridge.hpp"

#include <algorithm>
#include <arpa/inet.h>
#include <cerrno>
#include <netinet/tcp.h>
#include <stdexcept>
#include <sys/socket.h>
#include <unistd.h>
#include <utility>

namespace underwater {

namespace {

// 待发缓冲上限：客户端太慢时宁可断开，也不无限堆积内存。
constexpr std::size_t max_pending = 256 * 1024;  // 待发缓冲上限 256KiB

}  // namespace

TcpSerialBridge::~TcpSerialBridge() {
  close();
}

// 建监听套接字并绑定端口（address 必须是 IPv4 文本）。
void TcpSerialBridge::open(const std::string &address, std::uint16_t port) {
  close();

  sockaddr_in endpoint{};
  endpoint.sin_family = AF_INET;
  endpoint.sin_port = htons(port);
  if (inet_pton(AF_INET, address.c_str(), &endpoint.sin_addr) != 1) {
    throw std::invalid_argument("tcp_bind_address must be an IPv4 address");
  }

  listener_ = ::socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
  if (listener_ < 0) {
    throw std::runtime_error("TCP socket creation failed");
  }

  int yes = 1;
  setsockopt(listener_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

  if (::bind(listener_, reinterpret_cast<sockaddr *>(&endpoint), sizeof(endpoint)) != 0 ||
      ::listen(listener_, 4) != 0) {
    close();
    throw std::runtime_error("TCP bind/listen failed; check address and port ownership");
  }

  socklen_t length = sizeof(endpoint);
  if (getsockname(listener_, reinterpret_cast<sockaddr *>(&endpoint), &length) != 0) {
    close();
    throw std::runtime_error("TCP bound port unavailable");
  }

  port_ = ntohs(endpoint.sin_port);
  disconnected_ = false;
  detail_ = "listening; waiting for serial";
}

// 关闭套接字并复位全部会话状态。
void TcpSerialBridge::close() noexcept {
  if (client_ >= 0) {
    ::close(client_);
  }
  if (listener_ >= 0) {
    ::close(listener_);
  }
  client_ = -1;
  listener_ = -1;
  outgoing_.clear();
  offset_ = 0;
  ready_ = false;
  port_ = 0;
  disconnected_ = false;
  authenticated_ = false;
  handshake_.clear();
  token_.clear();
  owner_ = ControllerOwner::Nano;
  write_disconnected_ = false;
}

// 统一断开出口：若断的是"已认证的 GUI"，置 write_disconnected_ 让上层收回控制权。
void TcpSerialBridge::drop(const std::string &reason, bool holder_disconnect) {
  if (client_ >= 0) {
    write_disconnected_ |= (holder_disconnect && authenticated_ && owner_ == ControllerOwner::Gui);
    ::close(client_);
    client_ = -1;
    disconnected_ = true;
  }
  outgoing_.clear();
  offset_ = 0;
  authenticated_ = false;
  handshake_.clear();
  detail_ = reason;
}

// 控制权变化时强制断开旧客户端，避免旧 token 继续写串口。
void TcpSerialBridge::set_controller(const ControllerOwner owner, const std::string &token) {
  const auto raw_token = (owner == ControllerOwner::Gui ? token : std::string{});
  if (owner_ == owner && token_ == raw_token) {
    return;
  }
  if (client_ >= 0) {
    drop("Control owner changed; reconnect to apply the new ownership", false);
  }
  write_disconnected_ = false;
  owner_ = owner;
  token_ = raw_token;
}

void TcpSerialBridge::set_serial_ready(bool ready) {
  ready_ = ready;
  if (!ready) {
    drop("serial unavailable; client must reconnect");
  }
}

// 一次性读取并清除标志位（std::exchange 的典型用法）。
bool TcpSerialBridge::take_disconnect() {
  return std::exchange(disconnected_, false);
}

bool TcpSerialBridge::take_write_disconnect() {
  return std::exchange(write_disconnected_, false);
}

// 非阻塞发送：写不完就保留 offset_，下一轮继续。
void TcpSerialBridge::flush() {
  if (client_ < 0) {
    return;
  }
  std::size_t budget = 65536;  // 单轮最多 64KiB
  while (offset_ < outgoing_.size() && budget) {
    const auto n = ::send(
        client_,
        outgoing_.data() + offset_,
        std::min(budget, outgoing_.size() - offset_),
        MSG_NOSIGNAL);

    if (n > 0) {
      offset_ += static_cast<std::size_t>(n);
      budget -= static_cast<std::size_t>(n);
      tx_bytes_ += n;
      continue;
    }
    if (n < 0 && errno == EINTR) {
      continue;
    }
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      return;
    }
    drop("TCP send failed");
    return;
  }
  if (offset_ == outgoing_.size()) {
    outgoing_.clear();
    offset_ = 0;
  }
}

// 主轮询：accept -> 握手 -> 收数据；返回的字节由调用方写入串口。
wire::Bytes TcpSerialBridge::poll() {
  wire::Bytes incoming;
  if (listener_ < 0) {
    return incoming;
  }

  // 轮询接入新客户端
  for (int i = 0; i < 4; ++i) {
    const int candidate = ::accept4(listener_, nullptr, nullptr, SOCK_NONBLOCK | SOCK_CLOEXEC);
    if (candidate < 0) {
      break;
    }
    if (!ready_ || client_ >= 0 || disconnected_) {
      ::close(candidate);
      continue;
    }

    client_ = candidate;
    ++sessions_;

    int yes = 1;
    int seconds = 5;
    int interval = 2;
    int count = 3;
    setsockopt(client_, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));
// TCP keepalive + USER_TIMEOUT：掉线的 GUI 连接能在数秒内被内核判定失效。
    setsockopt(client_, SOL_SOCKET, SO_KEEPALIVE, &yes, sizeof(yes));
    setsockopt(client_, IPPROTO_TCP, TCP_KEEPIDLE, &seconds, sizeof(seconds));
    setsockopt(client_, IPPROTO_TCP, TCP_KEEPINTVL, &interval, sizeof(interval));
    setsockopt(client_, IPPROTO_TCP, TCP_KEEPCNT, &count, sizeof(count));

    int timeout_ms = 5000;
    setsockopt(client_, IPPROTO_TCP, TCP_USER_TIMEOUT, &timeout_ms, sizeof(timeout_ms));
    int buffer = 65536;
    setsockopt(client_, SOL_SOCKET, SO_SNDBUF, &buffer, sizeof(buffer));

    authenticated_ = false;
    handshake_.clear();
    detail_ = (owner_ == ControllerOwner::Gui ? "TCP connected; waiting for GUI ownership token"
                                              : "TCP connected; receive-only");
  }

  if (client_ < 0) {
    return incoming;
  }

  std::uint8_t chunk[4096];
  for (int i = 0; i < 4; ++i) {
    const auto n = ::recv(client_, chunk, sizeof(chunk), 0);
    if (n > 0) {
      rx_bytes_ += n;
      if (owner_ == ControllerOwner::Gui) {
        if (!authenticated_) {
          handshake_.append(reinterpret_cast<const char *>(chunk), static_cast<std::size_t>(n));
          if (handshake_.size() > 128 && handshake_.find('\n') == std::string::npos) {
            incoming.clear();
            drop("GUI ownership token missing or too long");
            return incoming;
          }

          const std::string expected = "OWNER " + token_ + "\n";
          if (handshake_.find('\n') == std::string::npos &&
              (handshake_.size() > expected.size() || expected.compare(0, handshake_.size(), handshake_) != 0)) {
            rejected_bytes_ += handshake_.size();
            incoming.clear();
            drop("TCP control is owned by another client");
            return incoming;
          }

          const auto newline = handshake_.find('\n');
          if (newline != std::string::npos) {
            std::string line = handshake_.substr(0, newline);
            if (!line.empty() && line.back() == '\r') {
              line.pop_back();
            }
            if (line != "OWNER " + token_) {
              rejected_bytes_ += handshake_.size();
              incoming.clear();
              drop("TCP control is owned by another client");
              return incoming;
            }

            authenticated_ = true;
            detail_ = "TCP connected; GUI owns exclusive serial TX";
            static constexpr char accepted[] = "OK GUI\n";
            if (::send(client_, accepted, sizeof(accepted) - 1, MSG_NOSIGNAL) != static_cast<ssize_t>(sizeof(accepted) - 1)) {
              incoming.clear();
              drop("TCP control handshake reply failed");
              return incoming;
            }

            handshake_.erase(0, newline + 1);
            incoming.insert(incoming.end(), handshake_.begin(), handshake_.end());
            handshake_.clear();
          }
        } else {
          incoming.insert(incoming.end(), chunk, chunk + n);
        }
      } else {
        rejected_bytes_ += n;
      }
      continue;
    }

    if (n < 0 && errno == EINTR) {
      --i;
      continue;
    }
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      break;
    }

    incoming.clear();
    drop("TCP client disconnected");
    return incoming;
  }

  flush();
  if (client_ < 0) {
    incoming.clear();
  }
  return incoming;
}

// 只有已认证的 GUI 或 Nano 模式才转发；积压超限视为客户端太慢，直接断开。
void TcpSerialBridge::forward_rx(const wire::Bytes &bytes) {
  if (client_ < 0 || bytes.empty() || (owner_ == ControllerOwner::Gui && !authenticated_)) {
    return;
  }
  if (outgoing_.size() - offset_ + bytes.size() > max_pending) {
    drop("TCP client too slow; pending data discarded");
    return;
  }
  if (offset_) {
    outgoing_.erase(outgoing_.begin(), outgoing_.begin() + offset_);
    offset_ = 0;
  }
  outgoing_.insert(outgoing_.end(), bytes.begin(), bytes.end());
  flush();
}

}  // namespace underwater
