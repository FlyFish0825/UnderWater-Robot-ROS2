#include "underwater_mcu_gateway/tcp_control_server.hpp"

#include <algorithm>
#include <arpa/inet.h>
#include <cerrno>
#include <netinet/tcp.h>
#include <stdexcept>
#include <sys/socket.h>
#include <unistd.h>

namespace underwater {
namespace {
constexpr std::size_t kMaxClients = 8;
constexpr std::size_t kMaxLine = 128;

std::string owner_line(const char *prefix, const OwnershipResult &result) {
  return std::string(prefix) + " " + controller_name(result.owner) + " " + result.token
      + " " + result.message + "\n";
}
}

TcpControlServer::~TcpControlServer() { close(); }

void TcpControlServer::open(const std::string &address, const std::uint16_t port) {
  close();
  sockaddr_in endpoint{};
  endpoint.sin_family = AF_INET;
  endpoint.sin_port = htons(port);
  if (inet_pton(AF_INET, address.c_str(), &endpoint.sin_addr) != 1)
    throw std::invalid_argument("tcp_control_bind_address must be an IPv4 address");
  listener_ = ::socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
  if (listener_ < 0)
    throw std::runtime_error("control TCP socket creation failed");
  int yes = 1;
  setsockopt(listener_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
  if (::bind(listener_, reinterpret_cast<sockaddr *>(&endpoint), sizeof(endpoint)) != 0
      || ::listen(listener_, 8) != 0) {
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
  if (listener_ >= 0) ::close(listener_);
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
  if (line == "STATE") {
    queue(client, std::string("STATE ") + controller_name(ownership_.owner()) + "\n");
  } else if (line == "CLAIM GUI" || line == "TAKEOVER GUI") {
    const bool takeover = line == "TAKEOVER GUI";
    const auto result = ownership_.claim_gui(client.fd, takeover);
    queue(client, owner_line(result.success ? "OK" : "BUSY", result));
  } else if (line == "RELEASE GUI") {
    const auto result = ownership_.release_gui(client.fd);
    queue(client, owner_line(result.success ? "OK" : "BUSY", result));
  } else if (line.rfind("PING ", 0) == 0) {
    const std::string token = line.substr(5);
    if (ownership_.heartbeat_gui(client.fd, token))
      queue(client, "HEARTBEAT GUI\n");
    else
      queue(client, std::string("REVOKED ") + controller_name(ownership_.owner()) + "\n");
  } else {
    queue(client, "ERROR expected STATE, CLAIM GUI, TAKEOVER GUI, RELEASE GUI, or PING <token>\n");
  }
  return true;
}

void TcpControlServer::remove_client(const std::size_t index) {
  if (index >= clients_.size()) return;
  const int fd = clients_[index].fd;
  ownership_.disconnect_gui(fd);
  ::close(fd);
  clients_.erase(clients_.begin() + static_cast<std::ptrdiff_t>(index));
}

void TcpControlServer::poll() {
  if (listener_ < 0) return;
  while (true) {
    const int fd = ::accept4(listener_, nullptr, nullptr, SOCK_NONBLOCK | SOCK_CLOEXEC);
    if (fd < 0) break;
    if (clients_.size() >= kMaxClients) { ::close(fd); continue; }
    int yes = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));
    clients_.push_back({fd, {}, {}});
  }

  // Process a bounded number of bytes and commands so control traffic cannot starve UART IO.
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
          if (!line.empty() && line.back() == '\r') line.pop_back();
          if (line.size() > kMaxLine) { remove = true; break; }
          handle_line(client, line);
        }
        if (remove) break;
        continue;
      }
      if (size == 0) { remove = true; break; }
      if (errno == EINTR) { --read; continue; }
      if (errno == EAGAIN || errno == EWOULDBLOCK) break;
      remove = true;
      break;
    }
    if (!remove && !client.output.empty()) {
      const auto sent = ::send(client.fd, client.output.data(), client.output.size(), MSG_NOSIGNAL);
      if (sent > 0) client.output.erase(0, static_cast<std::size_t>(sent));
      else if (sent == 0 || (errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK))
        remove = true;
    }
    if (remove) remove_client(index - 1);
  }
  ownership_.expire_gui();
}

}  // namespace underwater
