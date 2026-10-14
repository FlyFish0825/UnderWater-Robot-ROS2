// ==============================================================================
// src/global_camera/src/stream_server.cpp
// ==============================================================================
// 双目图像 TCP 推流服务实现 (默认 9001 端口，UWSC 40 字节帧头 + JPEG)：
// - 行为约定：只发不收 (客户端指令一律忽略)；
// - 每个客户端仅推送最新完整帧，杜绝跨帧拼接；
// - 客户端连接上限 4 个；单帧发送超时 2 秒主动断开慢客户端，防止阻塞内存。
// ==============================================================================
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

StreamServer::StreamServer(const std::string &address, std::uint16_t port) {
  listener_ = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
  if (listener_ < 0) {
    throw std::runtime_error("camera TCP socket: " + std::string(strerror(errno)));
  }

  int reuse = 1;
  setsockopt(listener_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

  sockaddr_in a{};
  a.sin_family = AF_INET;
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

StreamServer::~StreamServer() {
  stopping_ = true;
  if (thread_.joinable()) {
    thread_.join();
  }
  if (listener_ >= 0) {
    close(listener_);
  }
}

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

  std::lock_guard<std::mutex> lock(mutex_);
  latest_ = std::move(packet);
}

void StreamServer::clear() {
  std::lock_guard<std::mutex> lock(mutex_);
  latest_.reset();
  ++generation_;
}

void StreamServer::run() {
  using Clock = std::chrono::steady_clock;

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
