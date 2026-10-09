#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
namespace global_camera {
constexpr std::size_t header_size = 40;
constexpr std::size_t max_jpeg_size = 8 * 1024 * 1024;
// Network byte order; UWSC/version/flags/sequence/receipt_ns/width/height/jpeg_size/reserved.
inline std::array<std::uint8_t, header_size> make_header(
    std::uint64_t sequence, std::uint64_t receipt_ns, std::uint32_t width,
    std::uint32_t height, std::size_t jpeg_size) {
  if (!width || width % 2 || width > 8192 || !height || height > 4096 ||
      !jpeg_size || jpeg_size > max_jpeg_size)
    throw std::invalid_argument("invalid stereo dimensions or JPEG length");
  std::array<std::uint8_t, header_size> h{};
  h[0]='U'; h[1]='W'; h[2]='S'; h[3]='C'; h[5]=1; h[7]=1;
  auto put=[&](std::size_t offset, std::uint64_t value, std::size_t bytes) {
    for (std::size_t i=0; i<bytes; ++i) h[offset+bytes-1-i]=static_cast<std::uint8_t>(value>>(8*i));
  };
  put(8,sequence,8); put(16,receipt_ns,8); put(24,width,4); put(28,height,4); put(32,jpeg_size,4);
  return h;
}
}
