// ==============================================================================
// internal/underwater_mcu_gateway/include/underwater_mcu_gateway/tcp_control_server.hpp
// ==============================================================================
// 管理用行协议 TCP 服务端 (默认 9002 端口)：
// 提供 STATE / CLAIM GUI / TAKEOVER GUI / RELEASE GUI / PING <token> 交互服务。
// 只做控制权仲裁管理，原始串口字节走 TcpSerialBridge (9001 端口)，两者互不影响。
// 客户端断开连接即释放其 GUI 所有权；心跳超时由 ControlOwnership 负责回收。
// ==============================================================================
#pragma once

#include "underwater_mcu_gateway/control_ownership.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace underwater {

class TcpControlServer {
public:
  explicit TcpControlServer(ControlOwnership &ownership)
      : ownership_(ownership) {}
  ~TcpControlServer();

  TcpControlServer(const TcpControlServer &) = delete;
  TcpControlServer &operator=(const TcpControlServer &) = delete;

  // 打开 TCP 监听端口
  void open(const std::string &bind_address, std::uint16_t port);

  // 关闭服务端与所有活动客户端
  void close() noexcept;

  // 非阻塞单轮处理：accept、解析命令、发送应答并检查 GUI 心跳超时
  void poll();

  // 获取实际监听端口
  std::uint16_t port() const noexcept { return port_; }

private:
  // 单个管理客户端连接会话
  struct Client {
    int fd = -1;
    std::string input;
    std::string output;
  };

  bool handle_line(Client &client, const std::string &line);
  void queue(Client &client, std::string response);
  void remove_client(std::size_t index);

  ControlOwnership &ownership_;
  int listener_ = -1;
  std::uint16_t port_ = 0;
  std::vector<Client> clients_;
};

}  // namespace underwater
