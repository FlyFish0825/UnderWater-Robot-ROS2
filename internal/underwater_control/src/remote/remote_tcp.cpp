// ==============================================================================
// internal/underwater_control/src/remote/remote_tcp.cpp
// ==============================================================================
// 远程 TCP 服务端实现：
// - 非阻塞套接字配置与单客户端接入管理
// - 发送/接收突发限流与防泛洪保护
// - 3 秒未鉴权租约超时防护与 500 毫秒发送停滞检测
// ==============================================================================
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

// 设置文件描述符为非阻塞并开启 FD_CLOEXEC 标志
bool configure(int fd) {
  const auto flags = fcntl(fd, F_GETFL, 0);
  return (flags >= 0) &&
         (fcntl(fd, F_SETFL, flags | O_NONBLOCK) >= 0) &&
         (fcntl(fd, F_SETFD, FD_CLOEXEC) >= 0);
}

// 判定错误码是否为暂时不可读写或被信号打断
bool again() {
  return errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR;
}

}  // namespace

TcpServer::TcpServer(const std::string &address, std::uint16_t port) {
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);

  if (inet_pton(AF_INET, address.c_str(), &addr.sin_addr) != 1) {
    throw std::invalid_argument("remote bind must be IPv4");
  }

  listener_ = socket(AF_INET, SOCK_STREAM, 0);
  if (listener_ < 0) {
    throw std::runtime_error("remote socket failed");
  }

  int one = 1;
  const bool ok = configure(listener_) &&
                  (setsockopt(listener_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one)) == 0) &&
                  (bind(listener_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) == 0) &&
                  (listen(listener_, 4) == 0);

  if (!ok) {
    const std::string error = std::strerror(errno);
    close(listener_);
    listener_ = -1;
    throw std::runtime_error("remote listen failed: " + error);
  }
}

TcpServer::~TcpServer() {
  drop();
  if (listener_ >= 0) {
    close(listener_);
    listener_ = -1;
  }
}

void TcpServer::drop() {
  if (client_ >= 0) {
    close(client_);
    client_ = -1;
  }
  input_.reset();
  output_.reset();
  reserved_ = false;
  ever_reserved_ = false;
}

bool TcpServer::queue(const std::string &line, Millis at) {
  return connected() && line.size() <= 1025 && output_.push(line, at);
}

void TcpServer::poll(Millis at, const Handler &handler) {
  auto failure = [&](const std::string &why) {
    drop();
    handler(Event::Disconnected, why);
  };

  // 接收新接入的连接：单客户端排他模式
  for (int n = 0; n < 4; ++n) {
    const int fd = accept(listener_, nullptr, nullptr);
    if (fd < 0) {
      if (!again()) {
        throw std::runtime_error("remote accept failed");
      }
      break;
    }

    // 若当前已有已认证租约客户端接入，则拒绝新连接；并配置非阻塞
    if (reserved_ || !configure(fd)) {
      close(fd);
      continue;
    }

    // 禁用 Nagle 算法以降低交互延迟，开启 TCP 心跳保活
    int one = 1;
    if (setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one)) != 0 ||
        setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &one, sizeof(one)) != 0) {
      close(fd);
      continue;
    }

    // 若已有未鉴权的旧连接，替换为最新接入者
    if (connected()) {
      failure("unverified connection replaced");
    }

    client_ = fd;
    ++generation_;
    accepted_at_ = at;

    // 发送协议握手欢迎横幅
    queue("HELLO ROV_MANUAL 2\n", at);
    handler(Event::Connected, "");
  }

  if (!connected()) {
    return;
  }

  // 3 秒内未完成租约认证握手则踢出连接
  if (!ever_reserved_ && at - accepted_at_ >= 3000) {
    failure("manual authority handshake timed out");
    return;
  }

  // 发送缓冲区停滞超过 500 毫秒视为链路堵塞断开
  if (output_.stalled(at)) {
    failure("TCP output stalled for 500 ms");
    return;
  }

  // 轮询读取数据
  char bytes[4096];
  std::size_t received = 0;
  std::size_t commands = 0;

  while (connected() && received < sizeof(bytes)) {
    const auto size = recv(client_, bytes, sizeof(bytes) - received, 0);
    if (size == 0) {
      failure("TCP peer closed");
      return;
    }
    if (size < 0) {
      if (again()) {
        break;
      }
      failure("TCP receive failed");
      return;
    }

    received += static_cast<std::size_t>(size);
    std::vector<std::string> lines;
    std::string error;
    if (!input_.append(std::string_view(bytes, static_cast<std::size_t>(size)), lines, error)) {
      failure(error);
      return;
    }

    // 限制单次轮询处理的命令数量不超过 32 条
    commands += lines.size();
    if (commands > 32) {
      failure("TCP command burst exceeded 32 per poll");
      return;
    }

    for (const auto &line : lines) {
      handler(Event::Line, line);
      if (!connected()) {
        return;
      }
    }
  }

  // 单周期若读满整个 4096 字节预算，视为泛洪攻击
  if (received == sizeof(bytes)) {
    failure("TCP receive burst exceeded budget");
    return;
  }

  // 刷新发送缓冲区
  if (connected() && !output_.bytes().empty()) {
    const auto &out = output_.bytes();
    const auto size = send(client_, out.data(), out.size(), MSG_NOSIGNAL);
    if (size > 0) {
      output_.consume(static_cast<std::size_t>(size));
    } else if (size == 0 || !again()) {
      failure("TCP send failed");
    }
  }
}

}  // namespace underwater::control::remote
