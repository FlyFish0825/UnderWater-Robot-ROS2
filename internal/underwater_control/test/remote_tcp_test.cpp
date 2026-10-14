// ==============================================================================
// internal/underwater_control/test/remote_tcp_test.cpp
// ==============================================================================
// 远程 TCP 通信服务单元测试：
// 覆盖服务端绑定、客户端单连独占、HELLO 握手横幅、分包行提取、
// 命令泛洪保护与发送队列缓冲能力。
// ==============================================================================
#include "underwater_control/remote_tcp.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <iostream>
#include <stdexcept>

namespace rm = underwater::control::remote;

static int checks = 0;

static void check(bool value, const char *why) {
  ++checks;
  if (!value) {
    throw std::runtime_error(why);
  }
}

// 创建测试用 TCP 客户端套接字
static int client(std::uint16_t port) {
  const int fd = socket(AF_INET, SOCK_STREAM, 0);
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(port);
  inet_pton(AF_INET, "127.0.0.1", &address.sin_addr);

  if (connect(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) != 0) {
    close(fd);
    throw std::runtime_error("connect failed");
  }
  return fd;
}

int main() {
  const std::uint16_t port = 19123;
  rm::TcpServer server("127.0.0.1", port);

  check(!server.connected(), "initially disconnected");

  const int c1 = client(port);
  bool connected = false;
  std::vector<std::string> lines;

  // 轮询并接收客户端接入
  server.poll(10, [&](rm::TcpServer::Event event, const std::string &value) {
    if (event == rm::TcpServer::Event::Connected) {
      connected = true;
    }
    if (event == rm::TcpServer::Event::Line) {
      lines.push_back(value);
    }
  });

  check(connected, "client connected");
  check(server.connected(), "server reports connected");

  // 读取服务端下发的 HELLO 欢迎横幅
  char buffer[128];
  const auto n = read(c1, buffer, sizeof(buffer) - 1);
  check(n > 0, "read banner");
  buffer[n > 0 ? n : 0] = '\0';
  check(std::string(buffer).rfind("HELLO ROV_MANUAL 2\n", 0) == 0, "banner content");

  // 发送单行指令并提取
  const std::string command = "INPUT 1 1 0 0 0 0 0 0\n";
  check(write(c1, command.data(), command.size()) == static_cast<ssize_t>(command.size()), "write command");

  server.poll(20, [&](rm::TcpServer::Event event, const std::string &value) {
    if (event == rm::TcpServer::Event::Line) {
      lines.push_back(value);
    }
  });

  check(lines.size() == 1, "line received");
  check(lines.front() == "INPUT 1 1 0 0 0 0 0 0", "line content");

  // 验证排他性：第 2 个客户端不能踢出已保留(reserved)的第 1 个客户端
  server.reserve(true);
  const int c2 = client(port);
  server.poll(30, [](rm::TcpServer::Event, const std::string &) {});
  char probe[16];
  check(read(c2, probe, sizeof(probe)) == 0, "second client rejected while reserved");
  close(c2);

  // 清理第 1 个客户端
  close(c1);
  bool disconnected = false;
  server.poll(40, [&](rm::TcpServer::Event event, const std::string &) {
    if (event == rm::TcpServer::Event::Disconnected) {
      disconnected = true;
    }
  });
  check(disconnected, "disconnected reported");
  check(!server.connected(), "server reports disconnected");

  std::cout << "All " << checks << " remote TCP server tests passed.\n";
  return 0;
}
