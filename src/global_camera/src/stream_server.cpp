// ==============================================================================
// src/global_camera/src/stream_server.cpp
// ==============================================================================
// 双目图像 TCP 推流服务实现 (默认 9001 端口，UWSC 40 字节帧头 + JPEG)：
// - 行为约定：只发不收 (客户端指令一律忽略)；
// - 每个客户端仅推送最新完整帧，杜绝跨帧拼接；
// - 客户端连接上限 4 个；单帧发送超时 2 秒主动断开慢客户端，防止阻塞内存。
// ==============================================================================

// ── 初学者导读 ────────────────────────────────────────────────────────────
// 本文件实现 StreamServer（TCP 9001 推流）：一个后台线程负责把最新一帧 JPEG 发给 GUI。
// 它使用 Linux/POSIX 套接字 API（socket/bind/listen/accept/poll），因此只在 Linux 上编译。
// 会用到的系统调用：
//   - socket/bind/listen：创建、绑定、监听一个 TCP 端口；
//   - accept4：取出新连接（非阻塞）；
//   - poll：等待“可读/可写”，避免忙等；
//   - send/recv：收发送字节；
//   - close：关闭文件描述符（套接字也是一种 fd）。
// 会出现的 C++ 概念：
//   - std::thread：后台线程；run() 是线程体，析构时 join 等它退出。
//   - std::atomic：跨线程读写的原子变量（stopping_、clients_ 等），不会读到“半截”值。
//   - std::mutex + std::lock_guard：RAII 加锁，离开作用域自动解锁。
//   - std::shared_ptr<const Packet>：多线程共享“最新帧”的所有权，引用计数为 0 自动释放。
//   - 非阻塞 + 部分写：send 可能只发出去一部分，所以用 offset 记录进度、下次接着发。
// ─────────────────────────────────────────────────────────────────────────

#include "global_camera/stream_server.hpp"
#include "global_camera/wire.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <netinet/tcp.h>
#include <poll.h>
#include <stdexcept>
#include <sys/socket.h>
#include <unistd.h>

namespace global_camera {

// 构造函数：创建 TCP 监听套接字并 bind/listen，然后启动后台线程 run()。
// 这里用的是 Linux/POSIX 套接字 API（socket/bind/listen），所以本文件只在 Linux 上编译运行。
StreamServer::StreamServer(const std::string &address, std::uint16_t port) {
  listener_ = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
  if (listener_ < 0) {
    throw std::runtime_error("camera TCP socket: " + std::string(strerror(errno)));
  }

  int reuse = 1;
  setsockopt(listener_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

  sockaddr_in a{};
  a.sin_family = AF_INET;
  // htons：把主机字节序转成网络字节序（大端）
  a.sin_port = htons(port);

  if (inet_pton(AF_INET, address.c_str(), &a.sin_addr) != 1 ||
      bind(listener_, reinterpret_cast<sockaddr *>(&a), sizeof(a)) < 0 ||
      listen(listener_, 4) < 0) {
    const auto reason = std::string(strerror(errno));
    close(listener_);
    listener_ = -1;
    throw std::runtime_error("camera TCP bind/listen " + address + ":" + std::to_string(port) + ": " + reason);
  }

  socklen_t size = sizeof(a);
  getsockname(listener_, reinterpret_cast<sockaddr *>(&a), &size);
  port_ = ntohs(a.sin_port);

  try {
    thread_ = std::thread(&StreamServer::run, this);
  } catch (...) {
    close(listener_);
    listener_ = -1;
    throw;
  }
}

// 析构：先置 stopping_ 通知线程退出，join 等它结束，再关闭监听套接字（RAII 收尾）。
StreamServer::~StreamServer() {
  stopping_ = true;
  if (thread_.joinable()) {
    thread_.join();
  }
  if (listener_ >= 0) {
    close(listener_);
  }
}

// 发布最新一帧：组装 UWSC 帧头 + JPEG 成 Packet，加锁替换 latest_。
// 注意：这里只保存“最新一帧”，旧帧会被直接覆盖丢弃。
void StreamServer::publish(
    std::uint64_t sequence,
    std::uint64_t receipt_ns,
    std::uint32_t width,
    std::uint32_t height,
    const std::vector<std::uint8_t> &jpeg) {
  const auto h = make_header(sequence, receipt_ns, width, height, jpeg.size());
  auto packet = std::make_shared<Packet>();
  packet->sequence = sequence;
  packet->bytes.reserve(header_size + jpeg.size());
  packet->bytes.insert(packet->bytes.end(), h.begin(), h.end());
  packet->bytes.insert(packet->bytes.end(), jpeg.begin(), jpeg.end());

  // std::lock_guard：RAII 加锁，构造时 lock、离开作用域自动 unlock，绝不忘解锁。
  std::lock_guard<std::mutex> lock(mutex_);
  latest_ = std::move(packet);
}

// 相机掉线时调用：清空最新帧并让 generation_ 自增，run() 检测到代际变化会断开所有客户端。
void StreamServer::clear() {
  std::lock_guard<std::mutex> lock(mutex_);
  latest_.reset();
  ++generation_;
}

// 后台推流线程主体：poll 监听新连接与可写事件，向每个客户端发送最新帧。
// 采用“非阻塞套接字 + poll”的单线程事件循环，避免多线程逐客户端发送的复杂度。
void StreamServer::run() {
  using Clock = std::chrono::steady_clock;

  // 每个已连接客户端的状态：fd=套接字；packet=正在发送的帧；offset=已发字节数。
  struct Client {
    int fd = -1;
    std::shared_ptr<const Packet> packet;
    std::size_t offset = 0;
    std::uint64_t completed = 0;
    Clock::time_point started;
  };

  std::vector<Client> peers;
  std::uint64_t generation = 0;

  while (!stopping_) {
    std::shared_ptr<const Packet> latest;
    std::uint64_t next_generation = 0;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      latest = latest_;
      next_generation = generation_;
    }

    // clear() 后断开所有现有客户端，防止残留旧帧
    if (next_generation != generation) {
      for (auto &c : peers) {
        close(c.fd);
      }
      peers.clear();
      clients_ = 0;
      generation = next_generation;
    }

    std::vector<pollfd> events{{listener_, POLLIN, 0}};
    for (auto &c : peers) {
      if (latest && latest->sequence > c.completed && (!c.packet || c.offset == 0)) {
        if (!c.packet) {
          c.started = Clock::now();
        }
        c.packet = latest;
      }
      events.push_back({c.fd, static_cast<short>(POLLIN | (c.packet ? POLLOUT : 0)), 0});
    }

    // poll：等待一组套接字就绪（可读/可写），最多等 20ms 后返回，避免空转。
    const int ready = poll(events.data(), events.size(), 20);  // 20ms poll 响应
    if (ready < 0) {
      if (errno == EINTR) {
        continue;
      }
      healthy_ = false;
      break;
    }

    // 倒序遍历客户端，便于安全 erase
    for (std::size_t i = peers.size(); i > 0; --i) {
      auto &c = peers[i - 1];
      const auto revents = events[i].revents;
      bool dead = (revents & (POLLERR | POLLHUP | POLLNVAL)) != 0;

      // 推流为单向只读流，客户端发来任何数据视为退出断开
      if (revents & POLLIN) {
        char data[64];
        const auto n = recv(c.fd, data, sizeof(data), 0);
        if (n >= 0 || (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)) {
          dead = true;
        }
      }

      if (!dead && c.packet && (revents & POLLOUT)) {
        const auto &bytes = c.packet->bytes;
        const auto n = send(c.fd, bytes.data() + c.offset, bytes.size() - c.offset, MSG_NOSIGNAL);
        if (n > 0) {
          c.offset += static_cast<std::size_t>(n);
          if (c.offset == bytes.size()) {
            c.completed = c.packet->sequence;
            c.packet.reset();
            c.offset = 0;
            ++frames_sent_;
          }
        } else if (n == 0 || (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)) {
          dead = true;
        }
      }

      // 单帧发送超过 2 秒超时淘汰
      if (c.packet && Clock::now() - c.started > std::chrono::seconds(2)) {
        dead = true;
      }

      if (dead) {
        close(c.fd);
        peers.erase(peers.begin() + static_cast<std::ptrdiff_t>(i - 1));
      }
    }

    // 接入新客户端
    if (events[0].revents & POLLIN) {
      while (true) {
        // accept4：取出一个已完成三次握手的新连接，并直接设为非阻塞、exec 时自动关闭。
        int fd = accept4(listener_, nullptr, nullptr, SOCK_NONBLOCK | SOCK_CLOEXEC);
        if (fd < 0) {
          break;
        }
        if (peers.size() >= 4) {
          close(fd);  // 超过 4 个客户端连接上限拒绝
          continue;
        }

        int yes = 1;
        setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));
        peers.push_back({fd, nullptr, 0, 0, Clock::now()});
      }
    }

    clients_ = static_cast<unsigned>(peers.size());
  }

  for (auto &c : peers) {
    close(c.fd);
  }
  peers.clear();
  clients_ = 0;
}

}  // namespace global_camera
