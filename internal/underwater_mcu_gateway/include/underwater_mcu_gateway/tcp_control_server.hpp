// ==============================================================================
// internal/underwater_mcu_gateway/include/underwater_mcu_gateway/tcp_control_server.hpp
// ==============================================================================
// 管理用行协议 TCP 服务端 (默认 9002 端口)：
// 提供 STATE / CLAIM GUI / TAKEOVER GUI / RELEASE GUI / PING <token> 交互服务。
// 只做控制权仲裁管理，原始串口字节走 TcpSerialBridge (9001 端口)，两者互不影响。
// 勘误：原始串口字节走 TcpSerialBridge，bringup 默认端口是 9000（9001 是双目相机图像）。
// 客户端断开连接即释放其 GUI 所有权；心跳超时由 ControlOwnership 负责回收。
// ==============================================================================
// 初学者导读：
// 1) 所属：internal/underwater_mcu_gateway 的传输层，默认监听 9002 的"管理/文本"通道。
// 2) 交互：GUI 连上后发 STATE/CLAIM/TAKEOVER/RELEASE/PING 文本行，本类调用
//    control_reply() -> ControlOwnership 完成仲裁，再把应答行发回；不碰串口原始字节。
// 3) ROS 2 概念：本文件不含 ROS API；原始字节透传由 TcpSerialBridge 负责（bringup 默认 9000）。
// 4) C++ 概念：struct Client 表示一个连接会话；std::vector<Client> 动态数组存放所有客户端；
//    析构函数里 close() 属 RAII；poll() 是非阻塞"单轮驱动"。
#pragma once

#include "underwater_mcu_gateway/control_ownership.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace underwater {

// 管理 TCP 服务端：非阻塞、单线程轮询；每轮 poll() 处理 accept/命令/应答/心跳超时。
// 注意：必须由同一个线程调用，因为它直接操作 ControlOwnership（其内部无锁）。
class TcpControlServer {
public:
  explicit TcpControlServer(ControlOwnership &ownership)
      : ownership_(ownership) {}
  ~TcpControlServer();

  TcpControlServer(const TcpControlServer &) = delete;
  TcpControlServer &operator=(const TcpControlServer &) = delete;

  // 打开 TCP 监听端口
// 创建监听套接字并绑定端口；port 传 0 时由系统分配，之后用 port() 查真实端口。
  void open(const std::string &bind_address, std::uint16_t port);

  // 关闭服务端与所有活动客户端
// 关闭监听与所有客户端；对每个客户端调用 disconnect_gui() 归还控制权。
  void close() noexcept;

  // 非阻塞单轮处理：accept、解析命令、发送应答并检查 GUI 心跳超时
// 单轮处理：accept 新连接、读命令按行解析、发应答，最后处理心跳超时。
  void poll();

  // 获取实际监听端口
  std::uint16_t port() const noexcept { return port_; }

private:
  // 单个管理客户端连接会话
// 单个管理连接：fd 套接字、input 未成行缓冲、output 待发应答缓冲。
  struct Client {
    int fd = -1;
    std::string input;
    std::string output;
  };

// 处理一整行命令：交给 control_reply() 并把应答排入 client.output。
  bool handle_line(Client &client, const std::string &line);
// 把应答追加到待发缓冲；超过 1024 字节则丢弃并回一条 overflow 错误，防止内存被塞爆。
  void queue(Client &client, std::string response);
// 关闭并移除指定客户端，同时归还它可能持有的 GUI 控制权。
  void remove_client(std::size_t index);

// ownership_ 借用外部仲裁器；listener_ 监听 fd；clients_ 当前全部客户端。
  ControlOwnership &ownership_;
  int listener_ = -1;
  std::uint16_t port_ = 0;
  std::vector<Client> clients_;
};

}  // namespace underwater
