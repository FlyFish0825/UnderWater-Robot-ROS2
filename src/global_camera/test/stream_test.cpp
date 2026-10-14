// ==============================================================================
// src/global_camera/test/stream_test.cpp
// ==============================================================================
// StreamServer + UWSC 协议帧头端到端测试：
// - 校验帧头魔数与大端字节序 (receipt_ns, width, height, jpeg_size)；
// - 校验超限尺寸与奇数宽度的拒绝保护；
// - 校验“只发最新帧”机制与双客户端并发无交错；
// - 校验 clear() 掉线断开与慢客户端 2 秒超时淘汰。
// ==============================================================================

// ── 初学者导读 ────────────────────────────────────────────────────────────
// 这是一个不依赖 ROS 的端到端测试：真的起一个 StreamServer，用本地 TCP 客户端连上去，
// 检查 UWSC 帧头的大端字段是否正确、“只发最新帧”“多客户端互不串帧”“掉线断开”
// “慢客户端 2 秒被踢”等约定是否成立。
// C++/系统概念：socket/connect/recv/poll、htons 网络字节序、std::thread、
// 时间字面量 100ms，以及用 throw 表示断言失败。
// ─────────────────────────────────────────────────────────────────────────

#include "global_camera/stream_server.hpp"
#include "global_camera/wire.hpp"

#include <arpa/inet.h>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <poll.h>
#include <stdexcept>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>

using namespace std::chrono_literals;

namespace {

// 断言小工具：条件不成立就抛异常，由 main 的 try/catch 统一报告。
void require(bool condition, const char *message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

// 按大端字节序读取 count 个字节组成 64 位无符号整数
std::uint64_t get(const std::vector<std::uint8_t> &bytes, std::size_t start, std::size_t count) {
  std::uint64_t value = 0;
  for (std::size_t i = 0; i < count; ++i) {
    value = (value << 8) | bytes.at(start + i);
  }
  return value;
}

// 作为客户端连接本地测试服务器，返回套接字 fd（失败抛异常）。
int connect_to(std::uint16_t port) {
  int fd = socket(AF_INET, SOCK_STREAM, 0);
  require(fd >= 0, "client socket creation failed");

  sockaddr_in a{};
  a.sin_family = AF_INET;
  a.sin_port = htons(port);
  inet_pton(AF_INET, "127.0.0.1", &a.sin_addr);

  if (connect(fd, reinterpret_cast<sockaddr *>(&a), sizeof(a)) < 0) {
    close(fd);
    throw std::runtime_error("client connect failed");
  }
  return fd;
}

// 模拟分片流读取收满 size 个字节
std::vector<std::uint8_t> read_bytes(int fd, std::size_t size) {
  std::vector<std::uint8_t> out(size);
  std::size_t offset = 0;

  while (offset < size) {
    pollfd p{fd, POLLIN, 0};
    require(poll(&p, 1, 1500) > 0, "client read timeout");

    const auto n = recv(fd, out.data() + offset, std::min<std::size_t>(31, size - offset), 0);
    require(n > 0, "unexpected client EOF");
    offset += static_cast<std::size_t>(n);
  }
  return out;
}

// 读取 40 字节 UWSC 帧头 + JPEG 数据，校验大端字段并返回帧流水号
std::uint64_t read_frame(int fd, const std::vector<std::uint8_t> &expected) {
  const auto header = read_bytes(fd, 40);

  require(header[0] == 'U' && header[1] == 'W' && header[2] == 'S' && header[3] == 'C', "wire magic mismatch");
  require(get(header, 4, 2) == 1 && get(header, 6, 2) == 1, "version and side-by-side flag mismatch");
  require(get(header, 16, 8) == 0x0102030405060708ULL, "64-bit receipt timestamp byte order mismatch");
  require(get(header, 24, 4) == 2560 && get(header, 28, 4) == 720, "dimensions byte order mismatch");
  require(get(header, 32, 4) == expected.size() && get(header, 36, 4) == 0, "JPEG length and reserved field mismatch");
  require(read_bytes(fd, expected.size()) == expected, "JPEG bytes changed or frames interleaved");

  return get(header, 8, 8);
}

}  // namespace

// 测试入口：依次验证帧头编码、最新帧语义、离线/重连与慢客户端超时。
int main() {
  try {
    // 1. JPEG 长度校验测试
    for (const auto size : {0U, 8U * 1024 * 1024 + 1}) {
      bool rejected = false;
      try {
        global_camera::make_header(1, 2, 2560, 720, size);
      } catch (const std::invalid_argument &) {
        rejected = true;
      }
      require(rejected, "invalid JPEG size accepted");
    }

    // 奇数宽度拒绝
    bool rejected = false;
    try {
      global_camera::make_header(1, 2, 2559, 720, 12);
    } catch (const std::invalid_argument &) {
      rejected = true;
    }
    require(rejected, "odd stereo width accepted");

    // 2. 启动测试服务端 (端口 0 由内核分配)
    global_camera::StreamServer server("127.0.0.1", 0);
    const std::vector<std::uint8_t> jpeg{0xff, 0xd8, 1, 2, 3, 4, 5, 0xff, 0xd9};
    const std::uint64_t stamp = 0x0102030405060708ULL;

    // 连续发布 100 帧，新客户端只应当接收到最新的第 100 帧
    for (unsigned i = 1; i <= 100; ++i) {
      server.publish(i, stamp, 2560, 720, jpeg);
    }

    int a = connect_to(server.port());
    int b = connect_to(server.port());
    require(read_frame(a, jpeg) == 100 && read_frame(b, jpeg) == 100, "new clients did not get latest frame");

    pollfd idle{a, POLLIN, 0};
    require(poll(&idle, 1, 100) == 0, "same frame sent twice");

    server.publish(101, stamp, 2560, 720, jpeg);
    require(read_frame(a, jpeg) == 101 && read_frame(b, jpeg) == 101, "multi-client frame ordering");

    // clear() 掉线断开测试
    server.clear();
    pollfd closed{a, POLLIN, 0};
    require(poll(&closed, 1, 1000) > 0, "offline did not close client");
    char c = 0;
    require(recv(a, &c, 1, 0) == 0, "offline delivered stale frame");
    close(a);
    close(b);

    // 重新连接后不应收到旧残留帧
    a = connect_to(server.port());
    idle = {a, POLLIN, 0};
    require(poll(&idle, 1, 100) == 0, "reconnect got cached stale JPEG");

    server.publish(102, stamp, 2560, 720, jpeg);
    require(read_frame(a, jpeg) == 102, "reconnect lost next frame");
    close(a);

    // 3. 慢客户端 2 秒超时淘汰测试
    server.clear();
    std::this_thread::sleep_for(50ms);
    a = connect_to(server.port());
    const std::vector<std::uint8_t> large(6 * 1024 * 1024, 0x45);  // 6 MiB 阻塞大包
    server.publish(103, stamp, 2560, 720, large);
    std::this_thread::sleep_for(2300ms);
    require(server.clients() == 0, "slow client was not disconnected within bound");
    close(a);

    server.clear();
    std::this_thread::sleep_for(50ms);
    a = connect_to(server.port());
    server.publish(104, stamp, 2560, 720, jpeg);
    require(read_frame(a, jpeg) == 104, "slow consumer damaged later client");
    close(a);

    require(server.healthy(), "stream worker failed");
    std::cout << "PASS stereo TCP: framing, endian, latest-only, two clients, offline/reconnect, slow-client bound\n";
  } catch (const std::exception &e) {
    std::cerr << "FAIL: " << e.what() << '\n';
    return 1;
  }
  return 0;
}
