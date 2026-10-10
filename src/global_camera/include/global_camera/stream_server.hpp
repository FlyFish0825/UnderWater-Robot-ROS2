// stream_server.hpp —— 双目 JPEG 的 TCP 推流服务（默认端口 9001，见 wire.hpp 的 UWSC 帧格式）。
// 设计约定：采集端永不等待网络客户端；只保留“最新一帧”，不排队、不丢帧拼接。
// 最多 4 个客户端；客户端断开/落后 2 秒即被关闭；clear() 会主动断开全部客户端。
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
// Capture never waits for a network client. Only the newest complete frame is queued.
class StreamServer {
public:
  // address/port 绑定监听；port 传 0 时由系统分配（单元测试用）。
  StreamServer(const std::string &address, std::uint16_t port);
  ~StreamServer();
  StreamServer(const StreamServer&)=delete;
  StreamServer& operator=(const StreamServer&)=delete;
  // 发布一帧：sequence 递增序号，receipt_ns 采集时间戳（ns），jpeg 为左右拼接后的 JPEG。
  void publish(std::uint64_t sequence, std::uint64_t receipt_ns, std::uint32_t width,
               std::uint32_t height, const std::vector<std::uint8_t>& jpeg);
  // 清空待发帧并递增 generation，用于相机掉线时立即断开所有客户端。
  void clear();
  std::uint16_t port() const { return port_; }        // 实际监听端口
  unsigned clients() const { return clients_.load(); }        // 当前客户端数（上限 4）
  std::uint64_t frames_sent() const { return frames_sent_.load(); }  // 已完整发送的帧数
  bool healthy() const { return healthy_.load(); }    // 推送线程是否正常
private:
  struct Packet { std::uint64_t sequence; std::vector<std::uint8_t> bytes; };  // 帧头+JPEG 的整包
  void run();                                  // 推送线程主循环（poll 驱动）
  int listener_=-1;
  std::uint16_t port_=0;
  std::atomic<bool> stopping_{false}, healthy_{true};
  std::atomic<unsigned> clients_{0};
  std::atomic<std::uint64_t> frames_sent_{0};
  std::mutex mutex_;
  std::shared_ptr<const Packet> latest_;       // 仅保留最新一帧
  std::uint64_t generation_=0;                 // clear() 时递增，用于踢掉旧客户端
  std::thread thread_;
};
}
