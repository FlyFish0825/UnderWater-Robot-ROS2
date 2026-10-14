// ==============================================================================
// internal/underwater_mcu_gateway/src/protocol/stream_parser.cpp
// ==============================================================================
// 串口字节流增量解析器实现：
// - 喂入任意切分到达的底层字节流，滑窗寻找合规同步帧头 (0xAA)
// - 仅做帧头同步与长度预判，字段与 CRC 校验交由 codec.cpp 的 decode() 处理
// - 抛弃的噪声字节计入 counters.discarded，超时残帧记入 shape_errors
// ==============================================================================
#include "underwater_mcu_gateway/protocol.hpp"

#include <algorithm>

namespace underwater::wire {

void Parser::reset() {
  buffer_.clear();
  partial_ = false;
}

std::vector<Frame> Parser::feed(const Bytes &bytes, std::uint64_t now) {
  std::vector<Frame> result;

  // 缓冲上限 64KiB：超限视为噪声泛洪，整体清空丢弃
  if (buffer_.size() + bytes.size() > 65536) {
    counters_.discarded += buffer_.size();
    buffer_.clear();
    partial_ = false;
    if (bytes.size() > 65536) {
      counters_.discarded += bytes.size();
      return result;
    }
  }

  buffer_.insert(buffer_.end(), bytes.begin(), bytes.end());
  std::size_t cursor = 0;

  while (cursor < buffer_.size()) {
    const auto left = buffer_.size() - cursor;
    const auto *head = buffer_.data() + cursor;

    // 帧头必须为 0xAA 且第 2 字节是已知 family，否则单字节步进重新同步
    if (head[0] != 0xaa || (left > 1 && head[1] != 0x55 &&
                            head[1] != 0x58 && head[1] != 0x59 && head[1] != 0x5b)) {
      ++cursor;
      ++counters_.discarded;
      partial_ = false;
      continue;
    }

    std::size_t total = 0;
    bool bad_length = false;

    if (left >= 3 && head[1] == 0x55) {
      const auto n = head[2];
      bad_length = (n < 8 || n > 76);
      total = n + 6;  // 0x55：第 3 字节即帧体长度，整包 = 帧体 + 6
    } else if (left >= 16) {
      const auto n = u16(buffer_, cursor + 10);
      total = n + 20;  // 其余 family 的长度字段在偏移 10，整包 = 载荷 + 20
      bad_length = (head[1] == 0x58 && n != 0 && n != 5) ||
                   (head[1] == 0x5b && n > 58) ||
                   (n > 4096);
    }

    if (bad_length) {
      ++counters_.shape_errors;
      ++counters_.discarded;
      ++cursor;
      partial_ = false;
      continue;
    }

    // 尚未收全整包：等待后续数据；超过 1s 未收齐则视为断帧丢弃
    if (total == 0 || left < total) {
      if (!partial_) {
        partial_ = true;
        partial_since_ = now;
      }
      if (now - partial_since_ >= 1000) {
        ++counters_.shape_errors;
        ++counters_.discarded;
        ++cursor;
        partial_ = false;
        continue;
      }
      break;
    }

    Bytes packet(head, head + total);
    Frame frame;

    if (!decode(packet, frame)) {
      // 帧尾匹配但 CRC 不符记为 crc_errors，其余结构问题记为 shape_errors
      const bool tail = (packet[total - 2] == packet[1] && packet.back() == 0xaa);
      const bool crc = (packet[1] == 0x55) ?
          (crc8(packet.data() + 2, total - 5) == packet[total - 3]) :
          (crc16(packet.data() + 1, total - 5) == u16(packet, total - 4));

      if (tail && !crc) {
        ++counters_.crc_errors;
      } else {
        ++counters_.shape_errors;
      }
      ++counters_.discarded;
      ++cursor;
      partial_ = false;
      continue;
    }

    ++counters_.valid;
    result.push_back(std::move(frame));
    cursor += total;
    partial_ = false;
  }

  // 单次压缩缓冲区，保证线性扫描时间复杂度
  buffer_.erase(buffer_.begin(), buffer_.begin() + cursor);
  return result;
}

}  // namespace underwater::wire
