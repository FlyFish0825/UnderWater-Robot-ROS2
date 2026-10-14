// ==============================================================================
// src/global_camera/include/global_camera/stream_server.hpp
// ==============================================================================
// 双目图像 TCP 推流服务声明 (默认 9001 端口，UWSC 帧格式)：
// - 采集端永不等待网络客户端，独立高帧率运行；
// - 队列只保留最新一帧 (不积压、不跨帧拼接)；
// - 客户端上限 4 个；单帧发送阻塞超 2 秒主动切断慢客户端；
// - clear() 在相机掉线时立即断开全部连接并重置代际。
// ==============================================================================
#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace global_camera {

class StreamServer {
public:
  StreamServer(const std::string &address, std::uint16_t port);
  ~StreamServer();

  StreamServer(const StreamServer &) = delete;
  StreamServer &operator=(const StreamServer &) = delete;

  // 发布最新一帧双目拼接图像
  void publish(
      std::uint64_t sequence,
      std::uint64_t receipt_ns,
      std::uint32_t width,
      std::uint32_t height,
      const std::vector<std::uint8_t> &jpeg);

  // 清空待发帧并递增 generation，相机掉线时断开所有客户端
  void clear();

  // 状态与指标查询
  std::uint16_t port() const { return port_; }
  unsigned clients() const { return clients_.load(); }
  std::uint64_t frames_sent() const { return frames_sent_.load(); }
  bool healthy() const { return healthy_.load(); }

private:
  struct Packet {
    std::uint64_t sequence;
    std::vector<std::uint8_t> bytes;  // 40 字节 UWSC 帧头 + JPEG 数据
  };

  void run();  // 后台推流工作线程

  int listener_ = -1;
  std::uint16_t port_ = 0;
  std::atomic<bool> stopping_{false};
  std::atomic<bool> healthy_{true};
  std::atomic<unsigned> clients_{0};
  std::atomic<std::uint64_t> frames_sent_{0};
  std::mutex mutex_;
  std::shared_ptr<const Packet> latest_;
  std::uint64_t generation_ = 0;
  std::thread thread_;
};

}  // namespace global_camera
