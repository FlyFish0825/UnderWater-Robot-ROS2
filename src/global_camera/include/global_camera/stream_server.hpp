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
  StreamServer(const std::string &address, std::uint16_t port);
  ~StreamServer();
  StreamServer(const StreamServer&)=delete;
  StreamServer& operator=(const StreamServer&)=delete;
  void publish(std::uint64_t sequence, std::uint64_t receipt_ns, std::uint32_t width,
               std::uint32_t height, const std::vector<std::uint8_t>& jpeg);
  void clear();
  std::uint16_t port() const { return port_; }
  unsigned clients() const { return clients_.load(); }
  std::uint64_t frames_sent() const { return frames_sent_.load(); }
  bool healthy() const { return healthy_.load(); }
private:
  struct Packet { std::uint64_t sequence; std::vector<std::uint8_t> bytes; };
  void run();
  int listener_=-1;
  std::uint16_t port_=0;
  std::atomic<bool> stopping_{false}, healthy_{true};
  std::atomic<unsigned> clients_{0};
  std::atomic<std::uint64_t> frames_sent_{0};
  std::mutex mutex_;
  std::shared_ptr<const Packet> latest_;
  std::uint64_t generation_=0;
  std::thread thread_;
};
}
