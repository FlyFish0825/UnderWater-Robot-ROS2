// ==============================================================================
// internal/underwater_control/include/underwater_control/remote_tcp.hpp
// ==============================================================================
// 远程 TCP 服务端适配器：
// 基于 POSIX 异步非阻塞套接字实现单客户端独占连接接入。
// 严禁持有或直接操作 MCU 串口文件描述符，专职处理遥控网络字节流与分包。
// ==============================================================================
#pragma once

#include "underwater_control/remote_manual.hpp"
#include <functional>

namespace underwater::control::remote {

class TcpServer {
public:
  enum class Event {
    Connected,     // 客户端接入成功事件
    Line,          // 完整协议行接收事件
    Disconnected   // 连接断开或异常终止事件
  };

  using Handler = std::function<void(Event, const std::string &)>;

  TcpServer(const std::string &address, std::uint16_t port);
  ~TcpServer();

  TcpServer(const TcpServer &) = delete;
  TcpServer &operator=(const TcpServer &) = delete;

  // 轮询套接字事件并在主循环中触发回调处理
  void poll(Millis at, const Handler &handler);

  // 将待发送文本行加入发送队列 (行末自动补充换行符)
  bool queue(const std::string &line, Millis at);

  // 强制断开当前活动客户端连接
  void drop();

  // 设置当前会话是否已被保留
  void reserve(bool reserved) {
    reserved_ = reserved;
    ever_reserved_ |= reserved;
  }

  // 查询当前客户端是否在线
  bool connected() const {
    return client_ >= 0;
  }

  // 获取当前客户端连接的代际序号
  std::uint64_t generation() const {
    return generation_;
  }

private:
  int listener_ = -1;              // 监听套接字描述符
  int client_ = -1;                // 当前活动的客户端套接字描述符
  std::uint64_t generation_ = 0;   // 客户端连接代际计数器
  bool reserved_ = false;          // 租约保留状态
  bool ever_reserved_ = false;     // 连接期内是否曾被成功保留
  Millis accepted_at_ = 0;         // 客户端连接建立时刻
  LineBuffer input_;               // 接收行缓冲区
  OutputBuffer output_;            // 发送数据缓冲区
};

}  // namespace underwater::control::remote
