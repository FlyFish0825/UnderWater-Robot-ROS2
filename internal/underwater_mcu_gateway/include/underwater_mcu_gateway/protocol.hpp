#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>
namespace underwater::wire {
using Bytes = std::vector<std::uint8_t>;
std::uint16_t u16(const Bytes &, std::size_t);
std::uint32_t u32(const Bytes &, std::size_t);
float f32(const Bytes &, std::size_t);
std::uint8_t crc8(const std::uint8_t *, std::size_t);
std::uint16_t crc16(const std::uint8_t *, std::size_t);
struct Frame {
  std::uint8_t family=0, command=0, flags=0, target=0;
  std::uint32_t sequence=0, can_id=0, timestamp_us=0;
  bool timestamp_valid=false;
  Bytes payload;
};
struct Counters { std::uint64_t valid=0, crc_errors=0, shape_errors=0, discarded=0; };
class Parser {
public:
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
