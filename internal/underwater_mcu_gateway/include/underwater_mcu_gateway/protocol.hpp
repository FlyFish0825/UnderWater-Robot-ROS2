// ==============================================================================
// internal/underwater_mcu_gateway/include/underwater_mcu_gateway/protocol.hpp
// ==============================================================================
// MCU 串口网关的线协议定义：帧结构、解析统计与流式解析器接口。
// 约定：
// - 多字节数值字段一律小端序 (Little-Endian)；
// - CRC 校验不覆盖 0xAA 帧头，帧尾回显 family 字节。
// Frame / Parser 被网关节点与测试共用，修改布局需同步 codec.cpp 与 stream_parser.cpp。
// ==============================================================================
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace underwater::wire {

using Bytes = std::vector<std::uint8_t>;

// 字节序与浮点数解码辅助函数 (小端序)
std::uint16_t u16(const Bytes &bytes, std::size_t offset);
std::uint32_t u32(const Bytes &bytes, std::size_t offset);
float f32(const Bytes &bytes, std::size_t offset);

// CRC-8 校验：多项式 0x07、初值 0，用于 0x55 旧版下行帧
std::uint8_t crc8(const std::uint8_t *data, std::size_t length);

// CRC-16/CCITT-FALSE 校验：多项式 0x1021、初值 0xFFFF，用于 0x58/0x59/0x5b 帧
std::uint16_t crc16(const std::uint8_t *data, std::size_t length);

// 校验通过的完整协议帧：
// - 0x55 系列填充 can_id / flags / command；
// - 0x58 / 0x59 / 0x5b 系列填充 target / flags / command / timestamp。
struct Frame {
  std::uint8_t family = 0;             // 协议族 (0x55: CAN, 0x58: 心跳, 0x59: 通用, 0x5b: 设备)
  std::uint8_t command = 0;            // 指令字
  std::uint8_t flags = 0;              // 标志位
  std::uint8_t target = 0;             // 目标设备 (1: IMU, 2: 压力计)
  std::uint32_t sequence = 0;          // 帧流水号
  std::uint32_t can_id = 0;            // CAN 标识符 (仅 0x55 生效)
  std::uint32_t timestamp_us = 0;      // 硬件时间戳 (微秒)
  bool timestamp_valid = false;        // 时间戳是否有效
  Bytes payload;                       // 数据净荷负载
};

// 协议解析统计计数器
struct Counters {
  std::uint64_t valid = 0;             // 成功解析的有效帧数
  std::uint64_t crc_errors = 0;        // CRC 校验错误计数
  std::uint64_t shape_errors = 0;      // 帧格式/长度结构畸形计数
  std::uint64_t discarded = 0;         // 滑窗抛弃的杂波字节数
};

// 增量流解析器：将任意分片到达的字节流喂入，滑窗找头并输出完整且 CRC 正确的帧
class Parser {
public:
  // monotonic_ms 用于残帧超时判定 (残帧等待超过 1s 即超时丢弃)
  std::vector<Frame> feed(const Bytes &bytes, std::uint64_t monotonic_ms);

  // 清空接收缓冲区与残帧状态
  void reset();

  // 查询统计指标与当前缓冲区堆积字节量
  const Counters &counters() const { return counters_; }
  std::size_t buffered() const { return buffer_.size(); }

private:
  Bytes buffer_;
  Counters counters_;
  std::uint64_t partial_since_ = 0;
  bool partial_ = false;
};

// 核心解帧函数：校验帧头、长度、CRC 与各 family 的字段约束，成功时输出至 output
bool decode(const Bytes &bytes, Frame &output);

}  // namespace underwater::wire
