// ==============================================================================
// internal/underwater_mcu_gateway/include/underwater_mcu_gateway/tcp_serial_bridge.hpp
// ==============================================================================
// 原始串口透传 TCP 服务端 (单客户端、非阻塞，默认 9001 端口)：
// 勘误：本类没有硬编码 9001；实际监听端口由 tcp_port 参数决定，bringup 默认 9000。
// 字节原样透明转发，不解析、不包装、不重放。
// 只有控制权属于 GUI 且客户端完成 "OWNER <token>" 握手后才能写串口，否则仅接收。
// 串口未就绪、控制权变更或收发失败都会断开客户端，要求其重连并重新认证。
// ==============================================================================
// 初学者导读：
// 1) 所属：internal/underwater_mcu_gateway 传输层，负责"原始串口字节透传"；实际端口由
//    tcp_port 参数给出（bringup 默认 9000；9001 是双目相机 JPEG 端口）。
// 2) 交互：poll() 返回本应写入串口的字节（调用方负责写），forward_rx() 把串口收到的字节送回 GUI；
//    只有控制权属于 GUI 且完成 "OWNER <token>" 握手后才允许写串口。
// 3) ROS 2 概念：本文件不含 ROS API，是纯 TCP/串口字节管道。
// 4) C++ 概念：单客户端状态机；std::exchange 一步完成"取值并重置"；std::size_t 作偏移量；
//    析构函数里 close() 属 RAII。
#pragma once

#include "underwater_mcu_gateway/control_ownership.hpp"
#include "underwater_mcu_gateway/protocol.hpp"

#include <cstdint>
#include <string>

namespace underwater {

// 透传桥：单客户端独占。字节原样转发，不解析、不包装。
// 任何异常都会 drop() 断开连接，让上层重新认证，避免旧会话继续写串口。
class TcpSerialBridge {
public:
  TcpSerialBridge() = default;
  ~TcpSerialBridge();

  TcpSerialBridge(const TcpSerialBridge &) = delete;
  TcpSerialBridge &operator=(const TcpSerialBridge &) = delete;

  // 打开监听套接字
// 打开监听套接字并绑定端口；port=0 时由系统分配。
  void open(const std::string &bind_address, std::uint16_t port);

  // 关闭服务端与客户端连接
// 关闭监听与客户端并复位所有状态。
  void close() noexcept;

  // 控制权或 token 变化时断开现有客户端，避免旧会话继续持有写权限
// 通知桥"当前控制权归谁、token 是什么"；一旦变化就断开旧客户端，强制重新握手。
  void set_controller(ControllerOwner owner, const std::string &token);

  // 串口未就绪 (含尚未度过 settling 期) 时断开客户端
// 通知桥串口是否可用；不可用时断开客户端。
  void set_serial_ready(bool ready);

  // 处理 accept / 收发 / 握手，返回允许透传给串口的字节流 (每轮必须调用)
// 主轮询：处理 accept、握手、收字节；返回允许写入串口的字节（每轮都应调用）。
  wire::Bytes poll();

  // 把串口收到的字节排入发送缓冲；GUI 未认证或积压超限时丢弃外发
// 把串口收到的字节排入待发缓冲；未认证或积压超过 256KiB 时丢弃。
  void forward_rx(const wire::Bytes &bytes);

  // 检测并消费断开标志
// 一次性读取并清除断开标志（std::exchange 的典型用法）。
  bool take_disconnect();
  bool take_write_disconnect();

  // 状态属性查询
  bool connected() const { return client_ >= 0; }
  bool write_authorized() const {
    return client_ >= 0 && owner_ == ControllerOwner::Gui && authenticated_;
  }
  std::uint16_t port() const { return port_; }
  std::uint64_t rx_bytes() const { return rx_bytes_; }
  std::uint64_t tx_bytes() const { return tx_bytes_; }
  std::uint64_t rejected_bytes() const { return rejected_bytes_; }
  std::uint64_t sessions() const { return sessions_; }
  const std::string &detail() const { return detail_; }

private:
// 统一的断开路径：关闭客户端、清握手/缓冲，并记录 detail() 原因。
  void drop(const std::string &reason, bool holder_disconnect = true);
// 尽力把待发缓冲写出去；单轮最多 64KiB，遇 EAGAIN 就留到下一轮。
  void flush();

// 连接与握手状态（listener_/client_/ready_/authenticated_/owner_/token_ 等），
// 以及 outgoing_/offset_ 待发缓冲和各类统计计数。
  int listener_ = -1;
  int client_ = -1;
  std::uint16_t port_ = 0;
  bool ready_ = false;
  bool authenticated_ = false;
  bool disconnected_ = false;
  bool write_disconnected_ = false;

  ControllerOwner owner_ = ControllerOwner::Nano;
  std::string token_;
  std::string handshake_;
  wire::Bytes outgoing_;
  std::size_t offset_ = 0;

  std::uint64_t rx_bytes_ = 0;
  std::uint64_t tx_bytes_ = 0;
  std::uint64_t rejected_bytes_ = 0;
  std::uint64_t sessions_ = 0;
  std::string detail_ = "disabled";
};

}  // namespace underwater
