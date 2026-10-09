#include "underwater_mcu_gateway/protocol.hpp"
#include "underwater_mcu_gateway/tcp_serial_bridge.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>

using namespace underwater;
namespace {
int checks = 0;
void check(const bool ok, const char *why) {
  ++checks;
  if (!ok) throw std::runtime_error(why);
}
int connect_client(const std::uint16_t port, const char *source = nullptr) {
  const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
  check(fd >= 0, "socket");
  if (source) {
    sockaddr_in local{};
    local.sin_family = AF_INET;
    inet_pton(AF_INET, source, &local.sin_addr);
    check(::bind(fd, reinterpret_cast<sockaddr *>(&local), sizeof(local)) == 0,
          "changed client source address");
  }
  timeval timeout{1, 0};
  setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
  sockaddr_in endpoint{};
  endpoint.sin_family = AF_INET;
  endpoint.sin_port = htons(port);
  inet_pton(AF_INET, "127.0.0.1", &endpoint.sin_addr);
  check(::connect(fd, reinterpret_cast<sockaddr *>(&endpoint), sizeof(endpoint)) == 0,
        "connect");
  return fd;
}
void authenticate(TcpSerialBridge &server, const int fd, const std::string &token) {
  const std::string request = "OWNER " + token + "\n";
  check(::send(fd, request.data(), request.size(), MSG_NOSIGNAL)
          == static_cast<ssize_t>(request.size()), "send control token");
  server.poll();
  char response[32]{};
  const auto size = ::recv(fd, response, sizeof(response), 0);
  check(size == 7 && std::string(response, static_cast<std::size_t>(size)) == "OK GUI\n",
        "control token accepted before raw bytes");
  check(server.write_authorized(), "authenticated client holds serial write permission");
}
}

int main() {
  try {
    TcpSerialBridge server;
    server.open("127.0.0.1", 0);
    server.set_serial_ready(true);
    int fd = connect_client(server.port());
    server.poll();
    check(server.connected(), "connected read-only monitor");
    const wire::Bytes payload{0, 0xaa, 0x55, 0xff, 0x58, 0, 10, 13};
    server.forward_rx(payload);
    char buffer[4096];
    auto size = ::recv(fd, buffer, sizeof(buffer), 0);
    check(size == static_cast<ssize_t>(payload.size()) && wire::Bytes(buffer, buffer + size) == payload,
          "monitor receives exact raw UART bytes");
    check(::send(fd, payload.data(), payload.size(), MSG_NOSIGNAL)
            == static_cast<ssize_t>(payload.size()), "send while Nano owns control");
    for (int i = 0; i < 20 && server.rejected_bytes() == 0; ++i) {
      check(server.poll().empty(), "Nano ownership rejects monitor TX");
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    check(server.rejected_bytes() == payload.size(), "rejected write counted");

    server.set_controller(ControllerOwner::Gui, "session-token");
    check(!server.connected() && server.take_disconnect(), "ownership change drops the old monitor");
    ::close(fd);
    fd = connect_client(server.port());
    server.poll();
    check(::send(fd, payload.data(), payload.size(), MSG_NOSIGNAL)
            == static_cast<ssize_t>(payload.size()), "unauthenticated GUI-owner payload rejected");
    for (int i = 0; i < 20 && server.connected(); ++i) {
      server.poll();
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    check(!server.connected() && server.rejected_bytes() >= payload.size(),
          "raw commands cannot bypass the owner-token handshake");
    check(server.take_disconnect(), "rejected handshake reports a disconnected session");
    ::close(fd);

    fd = connect_client(server.port());
    authenticate(server, fd, "session-token");
    wire::Bytes incoming;
    for (const auto value : payload) {
      check(::send(fd, &value, 1, MSG_NOSIGNAL) == 1, "send raw byte");
      auto part = server.poll();
      incoming.insert(incoming.end(), part.begin(), part.end());
    }
    for (int i = 0; i < 50 && incoming.size() < payload.size(); ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
      auto part = server.poll();
      incoming.insert(incoming.end(), part.begin(), part.end());
    }
    check(incoming == payload, "authenticated fragmented commands delivered exactly");

    const int second = connect_client(server.port());
    server.poll();
    check(::recv(second, buffer, sizeof(buffer), 0) == 0, "second serial client rejected");
    ::close(second);
    ::close(fd);
    server.poll();
    check(!server.connected() && server.take_disconnect(), "disconnect event");
    server.forward_rx(payload);
    fd = connect_client(server.port(), "127.0.0.2");
    server.poll();
    authenticate(server, fd, "session-token");
    check(::recv(fd, buffer, sizeof(buffer), MSG_DONTWAIT) < 0, "no previous-session data replayed");
    server.forward_rx(wire::Bytes(256 * 1024 + 1, 0x55));
    check(!server.connected() && server.take_disconnect(), "slow-client bound");
    ::close(fd);
    server.set_serial_ready(false);
    server.close();
    server.open("127.0.0.1", 0);
    server.set_serial_ready(true);
    fd = connect_client(server.port());
    server.poll();
    check(!server.write_authorized(), "fresh listener starts with Nano-only ownership");
    server.forward_rx(payload);
    check(::recv(fd, buffer, sizeof(buffer), 0) == static_cast<ssize_t>(payload.size()),
          "read-only monitoring remains available after release");
    ::close(fd);
    std::cout << "PASS " << checks << " TCP raw bridge and ownership checks\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "FAIL " << error.what() << '\n';
    return 1;
  }
}
