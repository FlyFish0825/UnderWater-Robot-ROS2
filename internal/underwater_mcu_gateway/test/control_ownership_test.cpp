#include "underwater_mcu_gateway/control_ownership.hpp"
#include "underwater_mcu_gateway/tcp_serial_bridge.hpp"
#include "underwater_mcu_gateway/tcp_control_server.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

using namespace underwater;
namespace {
int checks = 0;
void check(const bool ok, const char *why) {
  ++checks;
  if (!ok) throw std::runtime_error(why);
}
int connect_client(const std::uint16_t port) {
  const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
  check(fd >= 0, "create control client");
  timeval timeout{1, 0};
  setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
  sockaddr_in endpoint{};
  endpoint.sin_family = AF_INET;
  endpoint.sin_port = htons(port);
  inet_pton(AF_INET, "127.0.0.1", &endpoint.sin_addr);
  check(::connect(fd, reinterpret_cast<sockaddr *>(&endpoint), sizeof(endpoint)) == 0,
        "connect control client");
  return fd;
}
std::string request(TcpControlServer &server, const int fd, const std::string &command) {
  check(::send(fd, command.data(), command.size(), 0) == static_cast<ssize_t>(command.size()),
        "send control request");
  for (int i = 0; i < 10; ++i) {
    server.poll();
    char buffer[512];
    const auto size = ::recv(fd, buffer, sizeof(buffer), MSG_DONTWAIT);
    if (size > 0) return std::string(buffer, static_cast<std::size_t>(size));
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  throw std::runtime_error("control reply timed out");
}
}

int main() {
  try {
    ControlOwnership ownership;
    TcpControlServer server(ownership);
    server.open("127.0.0.1", 0);
    const int gui1 = connect_client(server.port());
    const int gui2 = connect_client(server.port());
    check(request(server, gui1, "STATE\n").rfind("STATE NANO\n", 0) == 0,
          "Nano owns control on startup");
    check(request(server, gui1, "CLAIM GUI\n").rfind("BUSY NANO", 0) == 0,
          "GUI claim never silently preempts Nano");
    const auto takeover = request(server, gui1, "TAKEOVER GUI\n");
    check(takeover.rfind("OK GUI ", 0) == 0 && !ownership.gui_token().empty(),
          "explicit GUI takeover returns a session token");
    const auto token = ownership.gui_token();
    const int guiSession = ownership.gui_session();
    check(ownership.owner() == ControllerOwner::Gui && guiSession >= 0,
          "GUI is the unique owner");
    check(request(server, gui2, "CLAIM GUI\n").rfind("BUSY GUI", 0) == 0,
          "second GUI cannot share ownership");
    check(request(server, gui1, ("PING " + token + "\n")).rfind("HEARTBEAT GUI", 0) == 0,
          "owner heartbeat renews its lease");
    check(!ownership.heartbeat_gui(guiSession, "wrong-token"), "wrong token cannot renew a lease");
    auto result = ownership.claim_nano(false);
    check(!result.success && result.owner == ControllerOwner::Gui,
          "Nano claim reports busy and preserves GUI owner");
    result = ownership.claim_nano(true);
    check(result.success && result.owner == ControllerOwner::Nano && ownership.gui_token().empty(),
          "Nano takeover revokes GUI token immediately");
    check(request(server, gui1, ("PING " + token + "\n")).rfind("REVOKED NANO", 0) == 0,
          "preempted GUI learns that its lease is revoked");
    const auto second = request(server, gui2, "TAKEOVER GUI\n");
    check(second.rfind("OK GUI ", 0) == 0 && ownership.gui_token() != token,
          "a new GUI takeover gets a fresh token");
    check(request(server, gui1, "RELEASE GUI\n").rfind("BUSY GUI", 0) == 0,
          "non-owner cannot release another session");
    check(request(server, gui2, "RELEASE GUI\n").rfind("OK NANO", 0) == 0,
          "GUI release returns control to Nano");

    result = ownership.claim_gui(gui1, true);
    check(result.success, "GUI lease available for disconnect test");
    check(ownership.disconnect_gui(gui1) && ownership.owner() == ControllerOwner::Nano,
          "unexpected GUI disconnect returns control to Nano");
    result = ownership.claim_gui(gui1, true);
    check(result.success, "GUI lease available for heartbeat-expiry test");
    const auto expired = ControlOwnership::Clock::now() + std::chrono::seconds(4);
    check(ownership.expire_gui(expired) && ownership.owner() == ControllerOwner::Nano,
          "lost GUI heartbeat expires the lease");
    ::close(gui1);
    ::close(gui2);
    server.close();

    // End-to-end localhost path: 9002 grants a token that 9000 consumes before
    // forwarding the unwrapped bytes, then a Nano takeover revokes the session.
    ControlOwnership integration;
    TcpControlServer control_port(integration);
    control_port.open("127.0.0.1", 0);
    TcpSerialBridge serial_port;
    serial_port.open("127.0.0.1", 0);
    serial_port.set_serial_ready(true);
    const int manager = connect_client(control_port.port());
    const auto grant = request(control_port, manager, "TAKEOVER GUI\n");
    check(grant.rfind("OK GUI ", 0) == 0, "management port grants GUI ownership");
    const auto token_start = std::string("OK GUI ").size();
    const auto token_end = grant.find(' ', token_start);
    const auto lease = grant.substr(token_start, token_end - token_start);
    check(lease.size() == 32, "management response includes the active lease token");
    serial_port.set_controller(integration.owner(), integration.gui_token());
    const int data = connect_client(serial_port.port());
    const std::string hello = "OWNER " + lease + "\n";
    check(::send(data, hello.data(), hello.size(), MSG_NOSIGNAL)
            == static_cast<ssize_t>(hello.size()), "GUI presents the granted token on 9000");
    serial_port.poll();
    char bytes[256]{};
    auto received = ::recv(data, bytes, sizeof(bytes), 0);
    check(received == 7 && std::string(bytes, static_cast<std::size_t>(received)) == "OK GUI\n"
          && serial_port.write_authorized(), "9000 authenticates the management lease");
    const wire::Bytes command{0x00, 0xaa, 0x55, 0x10, 0xff};
    check(::send(data, command.data(), command.size(), MSG_NOSIGNAL)
            == static_cast<ssize_t>(command.size()), "GUI sends raw MCU bytes");
    wire::Bytes to_uart;
    for (int i = 0; i < 10 && to_uart.empty(); ++i) {
      auto next = serial_port.poll();
      to_uart.insert(to_uart.end(), next.begin(), next.end());
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    check(to_uart == command, "Nano forwards GUI bytes unchanged to the sole UART owner");
    const wire::Bytes from_uart{0x58, 0x00, 0xaa, 0x5b};
    serial_port.forward_rx(from_uart);
    received = ::recv(data, bytes, sizeof(bytes), 0);
    check(received == static_cast<ssize_t>(from_uart.size())
          && wire::Bytes(bytes, bytes + received) == from_uart,
          "raw MCU data remains readable in the owned GUI session");
    result = integration.claim_nano(true);
    serial_port.set_controller(integration.owner(), integration.gui_token());
    check(result.success && integration.owner() == ControllerOwner::Nano
          && !serial_port.write_authorized(), "Nano takeover revokes UART write authorization");
    check(::recv(data, bytes, sizeof(bytes), 0) == 0, "revoked raw session is closed");
    check(request(control_port, manager, "PING " + lease + "\n").rfind("REVOKED NANO", 0) == 0,
          "the old GUI management session sees its revoked lease");
    ::close(data);
    ::close(manager);
    serial_port.close();
    control_port.close();
    std::cout << "PASS " << checks << " ownership arbitration checks\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "FAIL " << error.what() << '\n';
    return 1;
  }
}
