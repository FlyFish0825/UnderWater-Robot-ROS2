// ==============================================================================
// src/global_camera/include/global_camera/stream_server.hpp
// ==============================================================================
// 双目图像 TCP 推流服务声明 (默认 9001 端口，UWSC 帧格式)：
// - 采集端永不等待网络客户端，独立高帧率运行；
// - 队列只保留最新一帧 (不积压、不跨帧拼接)；
// - 客户端上限 4 个；单帧发送阻塞超 2 秒主动切断慢客户端；
// - clear() 在相机掉线时立即断开全部连接并重置代际。
// ==============================================================================

// ── 初学者导读 ────────────────────────────────────────────────────────────
// StreamServer 是一个“后台常驻的推流服务”：它自己开一个线程，把相机最新的一帧 JPEG
// 通过 TCP 端口 9001 发给上位机（GUI）。这正是信号流图里“旁路”的图像那一支。
// 设计要点（初学者先记住结论即可）：
//   - 只保留最新一帧：网络慢也不拖慢相机，更不会把两帧拼在一起；
//   - 每帧发送超过 2 秒就踢掉该客户端，避免一个慢客户端拖垮全部；
//   - 相机掉线时 clear() 会断开所有连接并让 generation_ 自增，客户端据此知道“代际变了”。
// 会出现的概念：
//   - std::thread：C++ 标准库线程；本类的 run() 就是这个线程的执行体。
//   - std::atomic<T>：多线程共享的“原子变量”，读写不会被别的线程打断，用来做统计/标志。
//   - std::mutex + std::shared_ptr：用锁保护 latest_ 这张“最新帧”，用 shared_ptr 让发布者
//     与发送线程安全共享同一份数据的所有权，无需手动 delete。
//   - = delete：显式禁用拷贝构造/赋值，防止这个“带线程和端口”的对象被意外复制。
// ─────────────────────────────────────────────────────────────────────────

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
