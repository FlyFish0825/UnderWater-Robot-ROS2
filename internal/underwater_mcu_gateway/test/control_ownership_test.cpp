// ==============================================================================
// internal/underwater_mcu_gateway/test/control_ownership_test.cpp
// ==============================================================================
// 【初学者导读】本文件是“控制权仲裁”（到底谁有权控制机器人）的端到端测试，属于
// internal/underwater_mcu_gateway/test。它不依赖 ROS 运行环境，而是直接构造
// ControlOwnership、TcpControlServer（9002 管理通道）、TcpSerialBridge（9000 原始串口通道）
// 三个对象，验证“Nano 默认持有 -> GUI 申请/抢占 -> 心跳续租 -> 释放/断线后回归 Nano”的规则。
// 涉及概念：
// - 这些类是本项目的普通 C++ 类，不是 ROS 节点，但对应 mcu_gateway 节点内部的组件；
// - TCP Socket：用 POSIX socket 连到 127.0.0.1 的监听端口，模拟 GUI 客户端；
// - 所有权令牌(token)/租约(lease)：只有持有效令牌的客户端才能写串口；
// - 作用域对象/RAII：server、socket 在作用域结束时析构并自动关闭。
// ==============================================================================
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
// 简易断言计数：每检查一次 ++checks，失败就抛异常让 main 捕获。
int checks = 0;
// check()：本测试自制的断言，ok 为假就抛异常，由 main 统一捕获并打印 FAIL。
void check(const bool ok, const char *why) {
  ++checks;
  if (!ok) throw std::runtime_error(why);
}
// 建立一个连到本地测试服务器的 TCP 客户端，返回文件描述符 fd。
// setsockopt 设置 SO_RCVTIMEO=1 秒，避免测试永久阻塞。
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
// 向管理通道发送一行文本命令，再反复 poll 服务器并尝试 recv 读取应答。
// 用“循环 + 短睡眠”模拟等待服务器处理，最多重试 10 次，超时抛异常。
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

// 主测试流程：先单测仲裁规则，再做“9002 发令牌 + 9000 消费令牌”的端到端联调。
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

    check(request(server, gui1, "TAKEOVER GUI_MANUAL\n").rfind("OK GUI_MANUAL ", 0) == 0,
          "business takeover is available over the real management socket");
    const auto manual_token = ownership.gui_token();
    check(request(server, gui1, "PING " + manual_token + "\n") == "HEARTBEAT GUI_MANUAL\n",
          "business heartbeat reports its actual owner on the socket");
    check(request(server, gui2, "PING " + manual_token + "\n") == "REVOKED GUI_MANUAL\n",
          "another socket cannot heartbeat the copied business credential");
    check(request(server, gui1, "CLAIM GUI\n").rfind("BUSY GUI_MANUAL", 0) == 0,
          "business owner cannot silently switch to raw mode");
    check(request(server, gui1, "TAKEOVER GUI\n").rfind("OK GUI ", 0) == 0
          && ownership.gui_token() != manual_token,
          "explicit socket mode switch rotates the token");
    check(request(server, gui1, "TAKEOVER GUI_MANUAL\n").rfind("OK GUI_MANUAL ", 0) == 0,
          "business takeover restores the correct owner");
    check(request(server, gui1, "RELEASE GUI_MANUAL\n").rfind("OK NANO", 0) == 0,
          "business release over TCP restores Nano");
    ::close(gui1);
    ::close(gui2);
    server.close();

// 下面进入端到端集成：管理通道发放 token，数据通道必须先用 OWNER <token> 认证才能写串口。
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

    check(request(control_port, manager, "TAKEOVER GUI\n").rfind("OK GUI ", 0) == 0,
          "raw holder acquired for credential replacement regression");
    serial_port.take_disconnect();
    serial_port.take_write_disconnect();
    serial_port.set_controller(integration.owner(), integration.gui_token());
    const int old_raw = connect_client(serial_port.port());
    const std::string old_hello = "OWNER " + integration.gui_token() + "\n";
    check(::send(old_raw, old_hello.data(), old_hello.size(), MSG_NOSIGNAL)
          == static_cast<ssize_t>(old_hello.size()), "authenticate old raw holder");
    serial_port.poll();
    check(::recv(old_raw, bytes, sizeof(bytes), 0) == 7 && serial_port.write_authorized(),
          "old raw holder authenticated before takeover");
    const int new_manager = connect_client(control_port.port());
    check(request(control_port, new_manager, "TAKEOVER GUI\n").rfind("OK GUI ", 0) == 0,
          "second management session acquires replacement raw authority");
    const auto replacement_token = integration.gui_token();
    serial_port.set_controller(integration.owner(), replacement_token);
    check(::recv(old_raw, bytes, sizeof(bytes), 0) == 0, "replacement takeover closes old raw client");
    // Mirror the gateway's disconnect consumption: only a genuine current
    // holder's loss may return ownership to Nano.
    if(serial_port.take_write_disconnect() && integration.owner() == ControllerOwner::Gui)
      integration.claim_nano(true);
    check(integration.owner() == ControllerOwner::Gui && integration.gui_token() == replacement_token,
          "old raw revocation does not revoke the new GUI owner");
    check(request(control_port, new_manager, "PING " + replacement_token + "\n") == "HEARTBEAT GUI\n",
          "replacement raw management holder remains live");
    ::close(old_raw);
    ::close(new_manager);
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
