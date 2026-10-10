#pragma once
// MCU 串口网关的线协议定义：帧结构、解析统计与流式解析器接口。
// 约定：多字节字段一律小端；CRC 不覆盖 0xAA 帧头，帧尾回显 family 字节。
// Frame/Parser 被网关节点与测试共用，改布局需同步 codec.cpp 与 stream_parser.cpp。
#include <cstddef>
#include <cstdint>
#include <vector>
namespace underwater::wire {
using Bytes = std::vector<std::uint8_t>;
std::uint16_t u16(const Bytes &, std::size_t);
std::uint32_t u32(const Bytes &, std::size_t);
float f32(const Bytes &, std::size_t);
// CRC-8：多项式 0x07、初值 0，用于 0x55 旧版下行帧。
std::uint8_t crc8(const std::uint8_t *, std::size_t);
// CRC-16/CCITT-FALSE：多项式 0x1021、初值 0xFFFF，用于 0x58/0x59/0x5b 帧。
std::uint16_t crc16(const std::uint8_t *, std::size_t);
// 一条校验通过的帧；0x55 系列填 can_id/flags，0x58/0x59/0x5b 系列填 target。
struct Frame {
  std::uint8_t family=0, command=0, flags=0, target=0;
  std::uint32_t sequence=0, can_id=0, timestamp_us=0;
  bool timestamp_valid=false;
  Bytes payload;
};
struct Counters { std::uint64_t valid=0, crc_errors=0, shape_errors=0, discarded=0; };
// 增量流解析器：字节流按任意切分喂入，只输出完整且 CRC 正确的帧。
class Parser {
public:
  // monotonic_ms 用于残帧超时判定（残帧等待超过 1s 即丢弃）。
  std::vector<Frame> feed(const Bytes &, std::uint64_t monotonic_ms);
  void reset();
  const Counters & counters() const { return counters_; }
  std::size_t buffered() const { return buffer_.size(); }
private:
  Bytes buffer_;
  Counters counters_;
  std::uint64_t partial_since_=0;
  bool partial_=false;
};
bool decode(const Bytes &, Frame &);
}  // namespace underwater::wire
