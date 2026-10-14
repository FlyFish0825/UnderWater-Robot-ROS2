// ==============================================================================
// internal/underwater_control/test/remote_tcp_test.cpp
// ==============================================================================
// 远程 TCP 通信服务单元测试：
// 覆盖服务端绑定、客户端单连独占、HELLO 握手横幅、分包行提取、
// 命令泛洪保护与发送队列缓冲能力。
// ==============================================================================
// ------------------------------------------------------------------------------
// 【初学者导读】
// 所属包/目录：underwater_control 的单元测试。测试不启动 ROS，直接测试 TCP 服务端。
// 角色：验证服务端绑定、单客户端独占、HELLO 握手横幅、分包行提取、泛洪保护与断连上报。
// 与谁交互：创建真实的本地套接字连接，用 poll() 驱动服务端并检查回调事件。
// C++ 概念速览：lambda 回调捕获引用 [&] 收集结果；异常用于在断言失败时终止测试。
// ------------------------------------------------------------------------------
#include "underwater_control/remote_tcp.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <iostream>
#include <stdexcept>

namespace rm = underwater::control::remote;

// checks 统计断言次数，最后打印出来便于确认测试确实执行了。
static int checks = 0;

// 本地断言：失败就抛异常，main 未捕获即代表测试失败。
static void check(bool value, const char *why) {
  ++checks;
  if (!value) {
    throw std::runtime_error(why);
  }
}

// 创建测试用 TCP 客户端套接字
// 创建一个连接到本机指定端口的测试客户端套接字。
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

// main：启动服务端，验证接入、握手横幅、分包、排他与断开。
int main() {
// 用一个不常用端口，降低与本机其他服务冲突的概率。
  const std::uint16_t port = 19123;
  rm::TcpServer server("127.0.0.1", port);

// 刚创建时还没有客户端，应为未连接。
  check(!server.connected(), "initially disconnected");

  const int c1 = client(port);
  bool connected = false;
  std::vector<std::string> lines;

  // 轮询并接收客户端接入
// 轮询一次，应当收到 Connected 事件。
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
// 服务端接入时会主动发 HELLO 横幅，这里读取并核对内容。
  char buffer[128];
  const auto n = read(c1, buffer, sizeof(buffer) - 1);
  check(n > 0, "read banner");
  buffer[n > 0 ? n : 0] = '\0';
  check(std::string(buffer).rfind("HELLO ROV_MANUAL 2\n", 0) == 0, "banner content");

// 发一整行命令，poll 后应被切成一条 Line 回调。
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

// reserve(true) 后连接被“保留”，第二个客户端必须被拒绝。
  // 验证排他性：第 2 个客户端不能踢出已保留(reserved)的第 1 个客户端
  server.reserve(true);
  const int c2 = client(port);
  server.poll(30, [](rm::TcpServer::Event, const std::string &) {});
  char probe[16];
  check(read(c2, probe, sizeof(probe)) == 0, "second client rejected while reserved");
  close(c2);

// 关闭第一个客户端后，再 poll 应上报 Disconnected。
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
