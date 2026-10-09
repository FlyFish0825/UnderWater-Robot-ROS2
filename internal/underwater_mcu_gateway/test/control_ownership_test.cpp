#include "underwater_mcu_gateway/control_ownership.hpp"
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
    std::cout << "PASS " << checks << " ownership arbitration checks\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "FAIL " << error.what() << '\n';
    return 1;
  }
}
