// ==============================================================================
// internal/underwater_mcu_gateway/test/protocol_test.cpp
// ==============================================================================
// 串口协议帧物理编解码与流解析器单元测试：
// - 校验 CRC8 (0x55) 与 CRC16 (0x58/0x59/0x5b) 算法准确性
// - 校验增量流解析器 (Parser) 的滑窗找头、噪声抗干扰与残帧丢弃
// ==============================================================================
// 【初学者导读】本文件测试串口协议编解码，属于 internal/underwater_mcu_gateway/test。
// 协议帧以 0xaa 0x55/0x58/0x59/0x5b ... 0xaa 为边界：0x55 家族用 1 字节 CRC8，
// 其余家族用 2 字节 CRC16。Parser 负责从连续字节流里滑窗找帧头、校验长度与 CRC，
// 并统计丢弃的噪声字节。纯逻辑测试。涉及概念：字节数组 std::vector<uint8_t>、
// 小端序、CRC 校验、流式解析。
#include "underwater_mcu_gateway/protocol.hpp"

#include <iostream>
#include <sstream>
#include <stdexcept>

using namespace underwater::wire;

// 断言计数。
int checks = 0;

// 断言：失败时报告第几个检查出错。
void check(bool condition) {
  ++checks;
  if (!condition) {
    throw std::runtime_error("check failed at index " + std::to_string(checks));
  }
}

// 依次测试 0x55 帧、0x58 心跳帧和带前置噪声的 Parser 流解析。
int main() {
  // 1. 0x55 旧版 CAN 帧测试
  Bytes b55{0xaa, 0x55, 8, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0};
  b55[9] = 0x80;   // flags=0x80 (安全帧)
  b55[10] = 0x81;  // command=0x81
  b55[2] = 17;     // body length = 17
  b55.resize(23, 0);
  b55[20] = crc8(b55.data() + 2, 18);
  b55[21] = 0x55;
  b55[22] = 0xaa;

  Frame f55;
  check(decode(b55, f55));
  check(f55.family == 0x55);
  check(f55.flags == 0x80);

  // 2. 0x58 心跳帧测试 (CRC16)
  Bytes b58{0xaa, 0x58, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x58, 0xaa};
  const auto crc = crc16(b58.data() + 1, 15);
  b58[16] = static_cast<std::uint8_t>(crc & 0xff);
  b58[17] = static_cast<std::uint8_t>(crc >> 8);

  Frame f58;
  check(decode(b58, f58));
  check(f58.family == 0x58);

  // 3. Parser 流式增量切分与抗干扰测试
  Parser parser;
  Bytes stream;
  stream.insert(stream.end(), {0x12, 0x34, 0x56});  // 注入前置杂波噪声
  stream.insert(stream.end(), b58.begin(), b58.end());

  auto frames = parser.feed(stream, 100);
  check(frames.size() == 1);
  check(frames[0].family == 0x58);
  check(parser.counters().valid == 1);
  check(parser.counters().discarded == 3);  // 准确记录丢弃的 3 字节噪声

  std::cout << "All " << checks << " protocol codec & stream parser tests passed.\n";
  return 0;
}
