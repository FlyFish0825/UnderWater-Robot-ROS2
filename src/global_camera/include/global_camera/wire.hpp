// ==============================================================================
// src/global_camera/include/global_camera/wire.hpp
// ==============================================================================
// 双目图像 TCP 转发 (端口 9001) 的 UWSC 协议帧头定义：
// 约定：固定 40 字节帧头 + 左右拼接后的 JPEG 图像字节流，全部按网络大端序 (Big-Endian)。
// 帧头布局：
// - magic:       "UWSC" (字节 0..3)
// - version:     1 (字节 4..5)
// - flags:       1 (左右并排拼接 side-by-side, 字节 6..7)
// - sequence:    递增帧流水号 (字节 8..15)
// - receipt_ns:  V4L2 采集单调时间戳纳秒 (字节 16..23)
// - width:       图像总宽度，必须为偶数 (字节 24..27)
// - height:      图像总高度 (字节 28..31)
// - jpeg_size:   JPEG 负载字节数 (字节 32..35)
// - reserved:    预留字段 (字节 36..39)
// ==============================================================================

// ── 初学者导读 ────────────────────────────────────────────────────────────
// 这个头文件定义双目推流的“信封”格式 UWSC：固定 40 字节帧头 + 一段 JPEG 图像数据。
// 上位机（GUI）先读 40 字节头，就知道这一帧多大、多宽多高、序号是多少，再读对应长度的 JPEG。
// 因为要跨机器网络传输，所有多字节整数都按“网络大端序(Big-Endian)”写入。
// 会出现的概念：
//   - std::array<uint8_t, 40>：定长数组，大小在编译期确定，正好表示 40 字节帧头。
//   - constexpr：编译期常量；header_size / max_jpeg_size 就是这种常量。
//   - lambda（put）：写在函数里的匿名小函数，这里用来把整数按字节写进帧头；
//     [&] 表示按引用捕获外部变量（例如数组 h）。
//   - 位移 << 与 & 0xff：按字节拆解整数，是底层协议代码里最常见的手法。
// ─────────────────────────────────────────────────────────────────────────

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <stdexcept>

namespace global_camera {

constexpr std::size_t header_size = 40;                 // UWSC 帧头固定长度 (字节)
constexpr std::size_t max_jpeg_size = 8 * 1024 * 1024;  // 单帧 JPEG 上限 8 MiB

// 组装 UWSC 协议帧头并执行安全范围校验：
// - width 必须为偶数 (确保左右目等分) 且 <= 8192；
// - height 必须 <= 4096；
// - jpeg_size 非空且 <= 8 MiB。
inline std::array<std::uint8_t, header_size> make_header(
    std::uint64_t sequence,
    std::uint64_t receipt_ns,
    std::uint32_t width,
    std::uint32_t height,
    std::size_t jpeg_size) {
  if (!width || (width % 2 != 0) || width > 8192 ||
      !height || height > 4096 ||
      !jpeg_size || jpeg_size > max_jpeg_size) {
    throw std::invalid_argument("invalid stereo dimensions or JPEG length");
  }

  std::array<std::uint8_t, header_size> h{};
  h[0] = 'U';
  h[1] = 'W';
  h[2] = 'S';
  h[3] = 'C';
  h[5] = 1;  // 协议版本 1
  h[7] = 1;  // flags: 1 = side-by-side (左右并排拼接)

  auto put = [&](std::size_t offset, std::uint64_t value, std::size_t bytes) {
    // 高位先写入，确保网络大端序
    for (std::size_t i = 0; i < bytes; ++i) {
      h[offset + bytes - 1 - i] = static_cast<std::uint8_t>((value >> (8 * i)) & 0xff);
    }
  };

  put(8, sequence, 8);
  put(16, receipt_ns, 8);
  put(24, width, 4);
  put(28, height, 4);
  put(32, jpeg_size, 4);

  return h;
}

}  // namespace global_camera
