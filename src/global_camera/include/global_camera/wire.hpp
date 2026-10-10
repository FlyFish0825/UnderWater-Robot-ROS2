// wire.hpp —— 双目图像 TCP 转发（端口 9001）的 UWSC 帧头定义。
// 约定：固定 40 字节帧头 + 拼接后的 JPEG 数据，全部按网络字节序（大端）。
// 帧头布局：magic"UWSC"(0-3) / version(4-5)=1 / flags(6-7)=1(side-by-side) /
//           sequence(8-15) / receipt_ns(16-23) / width(24-27) / height(28-31) /
//           jpeg_size(32-35) / reserved(36-39)。
#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
namespace global_camera {
constexpr std::size_t header_size = 40;                 // UWSC 帧头固定长度（字节）
constexpr std::size_t max_jpeg_size = 8 * 1024 * 1024;  // 单帧 JPEG 上限 8 MiB，超出即拒绝
// Network byte order; UWSC/version/flags/sequence/receipt_ns/width/height/jpeg_size/reserved.
// 组装帧头并做范围校验：width 必须为偶数（左右目等分）、height<=4096、jpeg 非空且 <=8 MiB。
inline std::array<std::uint8_t, header_size> make_header(
    std::uint64_t sequence, std::uint64_t receipt_ns, std::uint32_t width,
    std::uint32_t height, std::size_t jpeg_size) {
  if (!width || width % 2 || width > 8192 || !height || height > 4096 ||
      !jpeg_size || jpeg_size > max_jpeg_size)
    throw std::invalid_argument("invalid stereo dimensions or JPEG length");
  std::array<std::uint8_t, header_size> h{};
  h[0]='U'; h[1]='W'; h[2]='S'; h[3]='C'; h[5]=1; h[7]=1;  // magic + 协议版本1 + flags=1(左右拼接)
  auto put=[&](std::size_t offset, std::uint64_t value, std::size_t bytes) {
    // 高位先写，保证大端字节序；测试用例按 0x0102030405060708 校验该顺序。
    for (std::size_t i=0; i<bytes; ++i) h[offset+bytes-1-i]=static_cast<std::uint8_t>(value>>(8*i));
  };
  put(8,sequence,8); put(16,receipt_ns,8); put(24,width,4); put(28,height,4); put(32,jpeg_size,4);
  return h;
}
}
