#pragma once
// 管理用行协议 TCP 服务端：STATE / CLAIM GUI / TAKEOVER GUI / RELEASE GUI / PING <token>。
// 只做控制权管理，原始串口字节走另一个端口（TcpSerialBridge），两者互不影响。
// 客户端断开即释放其 GUI 所有权；心跳超时由 ControlOwnership 负责回收。

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
  // 非阻塞单轮处理：accept、解析命令、发送应答并检查 GUI 心跳超时。
  void poll();
  // 实际监听端口；参数传 0 时为内核分配的端口。
  std::uint16_t port() const noexcept { return port_; }

private:
  // 单个管理连接：input 为未处理完的行缓冲，output 为待发送应答。
  struct Client { int fd = -1; std::string input; std::string output; };
  // 处理一条已按 \n 切好的命令；未知命令回 ERROR，不断开连接。
  bool handle_line(Client &client, const std::string &line);
  void queue(Client &client, std::string response);
  void remove_client(std::size_t index);

  ControlOwnership &ownership_;
  int listener_ = -1;
  std::uint16_t port_ = 0;
  std::vector<Client> clients_;
};

}  // namespace underwater
