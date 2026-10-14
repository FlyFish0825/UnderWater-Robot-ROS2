// ==============================================================================
// internal/underwater_control/src/remote/remote_tcp.cpp
// ==============================================================================
// 远程 TCP 服务端实现：
// - 非阻塞套接字配置与单客户端接入管理
// - 发送/接收突发限流与防泛洪保护
// - 3 秒未鉴权租约超时防护与 500 毫秒发送停滞检测
// ==============================================================================
// ------------------------------------------------------------------------------
// 【初学者导读】
// 所属包/目录：underwater_control 的 remote 源文件，实现 remote_tcp.hpp 的 TCP 服务端。
// 角色：用 POSIX 套接字 API 管理“单客户端、非阻塞”的连接；负责收发字节、拆分协议行、
//      限流防泛洪，并把事件回调给上层节点。
// 与谁交互：由 remote_manual_node.cpp 周期调用 poll()；收到的整行交给命令解析，
//          发出去的内容来自 queue()。本文件不持有、也不操作 MCU 串口。
// ROS 2 概念：本文件不含 ROS API，但它运行在节点的定时器回调里，属于 ROS 2 程序的一部分。
// C++ 概念速览：
//   RAII：在构造函数里获取资源、在析构函数里释放，避免忘记 close。
//   非阻塞 I/O：read/write 不等待，没数据就返回错误码，程序自己循环重试。
//   lambda 匿名函数：这里用于把“失败处理”封装成一个局部小函数。
// ------------------------------------------------------------------------------
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
// 设置套接字为非阻塞，并加上 FD_CLOEXEC（fork/exec 子进程时不继承该描述符）。
bool configure(int fd) {
  const auto flags = fcntl(fd, F_GETFL, 0);
  return (flags >= 0) &&
         (fcntl(fd, F_SETFL, flags | O_NONBLOCK) >= 0) &&
         (fcntl(fd, F_SETFD, FD_CLOEXEC) >= 0);
}

// 判定错误码是否为暂时不可读写或被信号打断
// EAGAIN/EWOULDBLOCK 表示“现在没数据/缓冲满，稍后再试”，EINTR 是被信号打断，都属正常。
bool again() {
  return errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR;
}

}  // namespace

// 构造函数：创建监听套接字并 bind/listen，失败就抛异常，让节点启动即失败。
TcpServer::TcpServer(const std::string &address, std::uint16_t port) {
  sockaddr_in addr{};
// sockaddr_in 是 IPv4 地址结构；htons 把端口转成网络字节序（大端）。
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);

// inet_pton 把点分十进制字符串转成二进制地址；这里只接受 IPv4。
  if (inet_pton(AF_INET, address.c_str(), &addr.sin_addr) != 1) {
    throw std::invalid_argument("remote bind must be IPv4");
  }

  listener_ = socket(AF_INET, SOCK_STREAM, 0);
  if (listener_ < 0) {
    throw std::runtime_error("remote socket failed");
  }

// SO_REUSEADDR 允许重启时立刻复用端口，避免 TIME_WAIT 导致绑定失败。
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

// 析构：关闭客户端与监听套接字，释放系统资源。
TcpServer::~TcpServer() {
  drop();
  if (listener_ >= 0) {
    close(listener_);
    listener_ = -1;
  }
}

// drop：关闭客户端描述符、清空收发缓冲、重置租约标记。
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

// queue：仅在已连接、且单行不超过 1025 字节时才放入发送缓冲。
bool TcpServer::queue(const std::string &line, Millis at) {
  return connected() && line.size() <= 1025 && output_.push(line, at);
}

// poll：非阻塞处理接入、读数据、写数据；任何错误都通过 handler(Disconnected) 上报。
void TcpServer::poll(Millis at, const Handler &handler) {
// 失败统一走这个 lambda：先 drop，再通知上层断开原因。
  auto failure = [&](const std::string &why) {
    drop();
    handler(Event::Disconnected, why);
  };

  // 接收新接入的连接：单客户端排他模式
// 循环 accept 新连接（一次最多 4 个），保持单客户端排他。
  for (int n = 0; n < 4; ++n) {
    const int fd = accept(listener_, nullptr, nullptr);
    if (fd < 0) {
      if (!again()) {
        throw std::runtime_error("remote accept failed");
      }
      break;
    }

// 已保留租约时拒绝新连接；configure 失败也拒绝。
    // 若当前已有已认证租约客户端接入，则拒绝新连接；并配置非阻塞
    if (reserved_ || !configure(fd)) {
      close(fd);
      continue;
    }

// TCP_NODELAY 关闭 Nagle 合并小包，降低遥控延迟；SO_KEEPALIVE 开启保活探测。
    // 禁用 Nagle 算法以降低交互延迟，开启 TCP 心跳保活
    int one = 1;
    if (setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one)) != 0 ||
        setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &one, sizeof(one)) != 0) {
      close(fd);
      continue;
    }

// 若已有未鉴权的旧连接，用最新接入者替换（防止半途连接卡住）。
    // 若已有未鉴权的旧连接，替换为最新接入者
    if (connected()) {
      failure("unverified connection replaced");
    }

    client_ = fd;
    ++generation_;
    accepted_at_ = at;

// 发送协议欢迎行，告诉客户端使用 v2 协议。
    // 发送协议握手欢迎横幅
    queue("HELLO ROV_MANUAL 2\n", at);
    handler(Event::Connected, "");
  }

  if (!connected()) {
    return;
  }

// 3 秒内没完成租约鉴权就断开，防止空连接长期占用。
  // 3 秒内未完成租约认证握手则踢出连接
  if (!ever_reserved_ && at - accepted_at_ >= 3000) {
    failure("manual authority handshake timed out");
    return;
  }

// 发送缓冲 500ms 没有进展，认为链路阻塞，主动断开。
  // 发送缓冲区停滞超过 500 毫秒视为链路堵塞断开
  if (output_.stalled(at)) {
    failure("TCP output stalled for 500 ms");
    return;
  }

// 下面开始读取客户端数据，char bytes[4096] 是本次读取的字节缓冲。
  // 轮询读取数据
  char bytes[4096];
  std::size_t received = 0;
  std::size_t commands = 0;

// recv 返回 0 表示对端正常关闭；<0 且 again() 表示暂时无数据；其余是错误。
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

// 把读到的字节交给 LineBuffer，切出完整行。
    received += static_cast<std::size_t>(size);
    std::vector<std::string> lines;
    std::string error;
    if (!input_.append(std::string_view(bytes, static_cast<std::size_t>(size)), lines, error)) {
      failure(error);
      return;
    }

// 单次轮询最多处理 32 条命令，超过视为突发泛洪攻击。
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

// 一个周期就读满 4096 字节，同样视为泛洪并断开。
  // 单周期若读满整个 4096 字节预算，视为泛洪攻击
  if (received == sizeof(bytes)) {
    failure("TCP receive burst exceeded budget");
    return;
  }

// 把发送缓冲里的数据真正 send 出去；MSG_NOSIGNAL 防止对端关闭触发 SIGPIPE。
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
