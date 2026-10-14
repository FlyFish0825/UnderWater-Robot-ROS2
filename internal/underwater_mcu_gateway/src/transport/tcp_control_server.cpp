// ==============================================================================
// internal/underwater_mcu_gateway/src/transport/tcp_control_server.cpp
// ==============================================================================
// 管理通道 TCP 服务端实现 (9002 端口)：
// - 按行解析命令，将控制权请求交给 ControlOwnership 仲裁
// - 只处理文本命令、不碰原始串口数据
// - 全部 I/O 非阻塞且单轮处理量有上限，避免管理流量饿死串口 I/O
// ==============================================================================
#include "underwater_mcu_gateway/tcp_control_server.hpp"
#include "underwater_mcu_gateway/control_protocol.hpp"

#include <algorithm>
#include <arpa/inet.h>
#include <cerrno>
#include <netinet/tcp.h>
#include <stdexcept>
#include <sys/socket.h>
#include <unistd.h>

namespace underwater {

namespace {

constexpr std::size_t kMaxClients = 8;  // 同时最多 8 个管理连接
constexpr std::size_t kMaxLine = 128;   // 单行命令与未成行缓冲的上限

}  // namespace

TcpControlServer::~TcpControlServer() {
  close();
}

void TcpControlServer::open(const std::string &address, const std::uint16_t port) {
  close();

  sockaddr_in endpoint{};
  endpoint.sin_family = AF_INET;
  endpoint.sin_port = htons(port);
  if (inet_pton(AF_INET, address.c_str(), &endpoint.sin_addr) != 1) {
    throw std::invalid_argument("tcp_control_bind_address must be an IPv4 address");
  }

  listener_ = ::socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
  if (listener_ < 0) {
    throw std::runtime_error("control TCP socket creation failed");
  }

  int yes = 1;
  setsockopt(listener_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

  if (::bind(listener_, reinterpret_cast<sockaddr *>(&endpoint), sizeof(endpoint)) != 0 ||
      ::listen(listener_, 8) != 0) {
    close();
    throw std::runtime_error("control TCP bind/listen failed; check address and port ownership");
  }

  socklen_t size = sizeof(endpoint);
  if (getsockname(listener_, reinterpret_cast<sockaddr *>(&endpoint), &size) != 0) {
    close();
    throw std::runtime_error("control TCP bound port unavailable");
  }
  port_ = ntohs(endpoint.sin_port);
}

void TcpControlServer::close() noexcept {
  for (const auto &client : clients_) {
    if (client.fd >= 0) {
      ownership_.disconnect_gui(client.fd);
      ::close(client.fd);
    }
  }
  clients_.clear();
  if (listener_ >= 0) {
    ::close(listener_);
  }
  listener_ = -1;
  port_ = 0;
}

void TcpControlServer::queue(Client &client, std::string response) {
  if (client.output.size() + response.size() > 1024) {
    client.output.clear();
    client.output = "ERROR response overflow\n";
    return;
  }
  client.output += response;
}

bool TcpControlServer::handle_line(Client &client, const std::string &line) {
  queue(client, control_reply(ownership_, client.fd, line));
  return true;
}

void TcpControlServer::remove_client(const std::size_t index) {
  if (index >= clients_.size()) {
    return;
  }
  const int fd = clients_[index].fd;
  ownership_.disconnect_gui(fd);
  ::close(fd);
  clients_.erase(clients_.begin() + static_cast<std::ptrdiff_t>(index));
}

void TcpControlServer::poll() {
  if (listener_ < 0) {
    return;
  }

  // 接收新接入的连接
  while (true) {
    const int fd = ::accept4(listener_, nullptr, nullptr, SOCK_NONBLOCK | SOCK_CLOEXEC);
    if (fd < 0) {
      break;
    }
    if (clients_.size() >= kMaxClients) {
      ::close(fd);
      continue;
    }
    int yes = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));
    clients_.push_back({fd, {}, {}});
  }

  // 轮询各客户端 I/O
  for (std::size_t index = clients_.size(); index > 0; --index) {
    auto &client = clients_[index - 1];
    bool remove = false;
    char buffer[256];

    for (int read = 0; read < 4; ++read) {
      const auto size = ::recv(client.fd, buffer, sizeof(buffer), 0);
      if (size > 0) {
        client.input.append(buffer, static_cast<std::size_t>(size));
        if (client.input.size() > kMaxLine && client.input.find('\n') == std::string::npos) {
          remove = true;
          break;
        }

        std::size_t newline = 0;
        int commands = 0;
        while ((newline = client.input.find('\n')) != std::string::npos && commands++ < 8) {
          std::string line = client.input.substr(0, newline);
          client.input.erase(0, newline + 1);
          if (!line.empty() && line.back() == '\r') {
            line.pop_back();
          }
          if (line.size() > kMaxLine) {
            remove = true;
            break;
          }
          handle_line(client, line);
        }
        if (remove) {
          break;
        }
        continue;
      }
      if (size == 0) {
        remove = true;
        break;
      }
      if (errno == EINTR) {
        --read;
        continue;
      }
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        break;
      }
      remove = true;
      break;
    }

    if (!remove && !client.output.empty()) {
      const auto sent = ::send(client.fd, client.output.data(), client.output.size(), MSG_NOSIGNAL);
      if (sent > 0) {
        client.output.erase(0, static_cast<std::size_t>(sent));
      } else if (sent == 0 || (errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK)) {
        remove = true;
      }
    }

    if (remove) {
      remove_client(index - 1);
    }
  }

  ownership_.expire_gui();  // 心跳超时的 GUI 自动交还 Nano
}

}  // namespace underwater
