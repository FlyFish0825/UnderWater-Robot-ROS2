#pragma once

#include "underwater_mcu_gateway/control_ownership.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace underwater {

// Line based management channel. Raw serial bytes remain on their existing port.
class TcpControlServer {
public:
  explicit TcpControlServer(ControlOwnership &ownership) : ownership_(ownership) {}
  ~TcpControlServer();
  TcpControlServer(const TcpControlServer &) = delete;
  TcpControlServer &operator=(const TcpControlServer &) = delete;

  void open(const std::string &bind_address, std::uint16_t port);
  void close() noexcept;
  void poll();
  std::uint16_t port() const noexcept { return port_; }

private:
  struct Client { int fd = -1; std::string input; std::string output; };
  bool handle_line(Client &client, const std::string &line);
  void queue(Client &client, std::string response);
  void remove_client(std::size_t index);

  ControlOwnership &ownership_;
  int listener_ = -1;
  std::uint16_t port_ = 0;
  std::vector<Client> clients_;
};

}  // namespace underwater
