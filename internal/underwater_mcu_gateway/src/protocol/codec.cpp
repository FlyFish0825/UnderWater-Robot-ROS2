// ==============================================================================
// internal/underwater_mcu_gateway/src/protocol/codec.cpp
// ==============================================================================
// 线协议底层编解码实现：
// - 字节序读取 (u16, u32, f32)
// - 校验和计算 (CRC-8: 0x07, CRC-16/CCITT-FALSE: 0x1021)
// - 整帧解码与合法性校验 (decode)
// 规格说明：
// - 0x55: 旧版下行控制与 CAN 透传帧 (CRC8)
// - 0x58 / 0x59 / 0x5b: 带时间戳的上行传感器/心跳帧 (CRC16)
// ==============================================================================
#include "underwater_mcu_gateway/protocol.hpp"

#include <cstring>
#include <stdexcept>

namespace underwater::wire {

// 读取 16 位小端序整数
std::uint16_t u16(const Bytes &b, std::size_t o) {
  return static_cast<std::uint16_t>(b.at(o)) |
         (static_cast<std::uint16_t>(b.at(o + 1)) << 8);
}

// 读取 32 位小端序整数
std::uint32_t u32(const Bytes &b, std::size_t o) {
  return static_cast<std::uint32_t>(u16(b, o)) |
         (static_cast<std::uint32_t>(u16(b, o + 2)) << 16);
}

// 读取 IEEE 754 32 位单精度浮点数 (小端序)
float f32(const Bytes &b, std::size_t o) {
  const auto bits = u32(b, o);
  float value;
  static_assert(sizeof(value) == sizeof(bits), "binary32 required");
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

// CRC-8 校验：初值 0x00，多项式 0x07 (x^8 + x^2 + x + 1)
std::uint8_t crc8(const std::uint8_t *p, std::size_t n) {
  std::uint8_t c = 0;
  for (std::size_t i = 0; i < n; ++i) {
    c ^= p[i];
    for (int j = 0; j < 8; ++j) {
      c = (c & 0x80) ? static_cast<std::uint8_t>((c << 1) ^ 7)
                     : static_cast<std::uint8_t>(c << 1);
    }
  }
  return c;
}

// CRC-16/CCITT-FALSE 校验：初值 0xFFFF，多项式 0x1021 (x^16 + x^12 + x^5 + 1)
std::uint16_t crc16(const std::uint8_t *p, std::size_t n) {
  std::uint16_t c = 0xffff;
  for (std::size_t i = 0; i < n; ++i) {
    c ^= static_cast<std::uint16_t>(p[i]) << 8;
    for (int j = 0; j < 8; ++j) {
      c = (c & 0x8000) ? static_cast<std::uint16_t>((c << 1) ^ 0x1021)
                       : static_cast<std::uint16_t>(c << 1);
    }
  }
  return c;
}

// 校验帧头、长度、CRC 与各 family 的字段约束，成功时把结果移入 output
bool decode(const Bytes &p, Frame &output) {
  // 最短合法帧（0x55 空载荷帧）为 14 字节，帧头必须为 0xAA
  if (p.size() < 14 || p[0] != 0xaa) {
    return false;
  }

  Frame f;
  f.family = p[1];

  // 1. 旧版 0x55 family (下行控制/CAN 透传)：格式为 AA 55 <len> ... CRC8 55 AA
  if (f.family == 0x55) {
    const auto body = p[2];  // 第 3 字节表示帧体长度
    // 帧体范围在 8..76 字节；整包长度 = body + 6；帧尾为 0x55 0xAA；CRC8 位于倒数第 3 字节
    if (body < 8 || body > 76 || p.size() != static_cast<std::size_t>(body) + 6 ||
        p[p.size() - 2] != 0x55 || p.back() != 0xaa ||
        crc8(p.data() + 2, body + 1) != p[body + 3]) {
      return false;
    }

    f.sequence = u16(p, 3);
    f.can_id = u32(p, 5);
    f.flags = p[9];

    if (f.flags == 0x80) {
      // 0x80：固定长度安全控制帧，帧体长度固定为 17 或 21 字节
      if ((body != 17 && body != 21) || p[10] != 0x81) {
        return false;
      }
      f.command = p[10];
      f.payload.assign(p.begin() + 11, p.begin() + 20);
      f.timestamp_valid = (body == 21);
      if (f.timestamp_valid) {
        f.timestamp_us = u32(p, 20);
      }
    } else {
      // 变长载荷帧
      const auto n = p[10];  // 载荷长度
      if (n > 64 || (body != 8 + n && body != 12 + n && body != 72)) {
        return false;
      }
      f.payload.assign(p.begin() + 11, p.begin() + 11 + n);
      f.timestamp_valid = (body == 12 + n);
      if (f.timestamp_valid) {
        f.timestamp_us = u32(p, 11 + n);
      }
    }
  } else {
    // 2. 0x58/0x59/0x5b family：帧尾回显 family 字节，CRC16 覆盖从 family 到载荷末尾全部数据
    if (p.size() < 20 || (f.family != 0x58 && f.family != 0x59 && f.family != 0x5b)) {
      return false;
    }

    const auto n = u16(p, 10);  // 载荷长度位于偏移 10
    if (p.size() != 20u + n || p[2] != 1 || p[18 + n] != f.family || p[19 + n] != 0xaa ||
        crc16(p.data() + 1, 15 + n) != u16(p, 16 + n)) {
      return false;
    }

    // 0x58 心跳/缓冲状态帧：载荷为 0 或 5 字节，协议字段结构固定
    if (f.family == 0x58 && ((n != 0 && n != 5) || p[3] != 1 || p[4] != 0 || p[5] != 0)) {
      return false;
    }

    // 0x59 通用变长帧：载荷上限 4096 字节
    if (f.family == 0x59 && n > 4096) {
      return false;
    }

    // 0x5b 设备管理帧：载荷 <= 58 字节；target 必须为 1(IMU) 或 2(压力计)；flags 必须属于 {1, 2, 6, 8}
    if (f.family == 0x5b && (n > 58 || (p[5] != 1 && p[5] != 2) ||
        (p[4] != 1 && p[4] != 2 && p[4] != 6 && p[4] != 8))) {
      return false;
    }

    f.command = p[3];
    f.flags = p[4];
    f.target = p[5];
    f.sequence = u32(p, 6);
    f.timestamp_valid = true;
    f.timestamp_us = u32(p, 12);
    f.payload.assign(p.begin() + 16, p.begin() + 16 + n);
  }

  output = std::move(f);
  return true;
}

}  // namespace underwater::wire
