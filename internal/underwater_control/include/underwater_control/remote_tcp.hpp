// ==============================================================================
// internal/underwater_control/include/underwater_control/remote_tcp.hpp
// ==============================================================================
// 远程 TCP 服务端适配器：
// 基于 POSIX 异步非阻塞套接字实现单客户端独占连接接入。
// 严禁持有或直接操作 MCU 串口文件描述符，专职处理遥控网络字节流与分包。
// ==============================================================================
// ------------------------------------------------------------------------------
// 【初学者导读】
// 所属包/目录：underwater_control 的网络层头文件，声明 TCP 服务端 TcpServer。
// 角色：只做“把一个客户端接进来、收字节切成完整行、把要发的内容排队发出去”，
//      不解析协议内容（那是 remote_manual.hpp 的活），也不碰 MCU 串口。
// 与谁交互：由 remote_manual_node.cpp 在 20ms 定时器里调用 poll()，把收到的整行交给命令解析；
//          节点用 queue() 回发 RESULT/STATE 文本。
// C++ 概念速览：
//   std::function<void(Event,const std::string&)>：可存放任意“函数/lambda”的通用可调用对象。
//   lambda 是匿名函数，写法 [捕获](参数){ 函数体 }，常直接当回调传进去。
//   = delete 表示禁止拷贝；TcpServer 独占系统套接字，复制一份没有意义。
//   析构函数 ~TcpServer() 在对象销毁时自动执行，负责关闭文件描述符（RAII 思想：
//     资源随对象生命周期自动获取和释放）。
// ------------------------------------------------------------------------------
#pragma once

#include "underwater_control/remote_manual.hpp"
#include <functional>

namespace underwater::control::remote {

// TcpServer 持有一个监听套接字和至多一个客户端套接字，保证“同一时刻只服务一个客户端”。
class TcpServer {
public:
// Event 表示 poll() 期间可能发生的事件，回调里用它决定怎么处理。
  enum class Event {
    Connected,     // 客户端接入成功事件
    Line,          // 完整协议行接收事件
    Disconnected   // 连接断开或异常终止事件
  };

// Handler 是事件回调类型；std::function 能装下普通函数、lambda 或成员函数绑定。
  using Handler = std::function<void(Event, const std::string &)>;

// 构造函数创建并绑定监听套接字；地址必须是点分十进制 IPv4，否则抛异常。
  TcpServer(const std::string &address, std::uint16_t port);
  ~TcpServer();

  TcpServer(const TcpServer &) = delete;
  TcpServer &operator=(const TcpServer &) = delete;

// poll 是“非阻塞轮询”：每次调用试着接入新连接、读数据、发数据后立刻返回，
// 绝不阻塞等待，这样才能塞进 ROS 的定时器里周期执行。
  // 轮询套接字事件并在主循环中触发回调处理
  void poll(Millis at, const Handler &handler);

// queue 把一行待发送文本放入发送缓冲区，稍后由 poll() 真正 send 出去。
  // 将待发送文本行加入发送队列 (行末自动补充换行符)
  bool queue(const std::string &line, Millis at);

// drop 主动断开当前客户端并清空收发缓冲区。
  // 强制断开当前活动客户端连接
  void drop();

// reserve 标记当前连接是否已通过租约鉴权；已保留时新连接会被拒绝。
  // 设置当前会话是否已被保留
  void reserve(bool reserved) {
    reserved_ = reserved;
    ever_reserved_ |= reserved;
  }

// 判断当前是否有客户端在线（文件描述符 >= 0 表示在线）。
  // 查询当前客户端是否在线
  bool connected() const {
    return client_ >= 0;
  }

// generation（代际）每接入一个客户端就 +1；异步回调用它确认“还是同一个连接”。
  // 获取当前客户端连接的代际序号
  std::uint64_t generation() const {
    return generation_;
  }

// private 成员只允许本类内部访问。
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
