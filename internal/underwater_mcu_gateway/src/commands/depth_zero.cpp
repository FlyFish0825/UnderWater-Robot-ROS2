// ==============================================================================
// internal/underwater_mcu_gateway/src/commands/depth_zero.cpp
// ==============================================================================
// 深度零点标定状态机实现：
// - Status -> Zero -> ReadReference -> Verify 完整事务流
// - 请求按 0x5b family、target=2 发送给压力设备，期望回复命令字为请求命令 | 0x40，序号自增；
// - 时间约束：单步请求 2s、等待采样 5s、整事务上限 15s；失败不自动重试。
// ==============================================================================
#include "underwater_mcu_gateway/depth_zero.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>

namespace underwater::wire {

namespace {

// 小端序写入 32 位无符号整数
void append32(Bytes &b, std::uint32_t n) {
  for (int i = 0; i < 4; ++i) {
    b.push_back(static_cast<std::uint8_t>((n >> (8 * i)) & 0xff));
  }
}

}  // namespace

// 启动事务：若正在执行则拒绝，整事务超时上限 15 秒
bool DepthZero::start(std::uint64_t now) {
  if (busy()) {
    return false;
  }
  result_.reset();
  outgoing_.reset();
  last_sample_.reset();
  overall_deadline_ = now + 15000;  // 整事务 15s 上限

  // 发起 Stage::Status 状态查询请求 (command = 2)
  request(Stage::Status, 2, {}, now);
  return true;
}

// 构造一帧请求：期望回复命令 = cmd + 0x40，序号自增，本步超时 2 秒
void DepthZero::request(Stage s, std::uint8_t cmd, const Bytes &payload, std::uint64_t now) {
  stage_ = s;
  expected_command_ = cmd + 0x40;
  expected_sequence_ = ++sequence_;
  deadline_ = now + 2000;

  // target=2 (压力设备)，flags=1 (读状态/指令)
  Bytes b{0xaa, 0x5b, 1, cmd, 1, 2};
  append32(b, expected_sequence_);
  b.push_back(static_cast<std::uint8_t>(payload.size()));
  b.push_back(0);
  append32(b, 0);
  b.insert(b.end(), payload.begin(), payload.end());

  const auto crc = crc16(b.data() + 1, b.size() - 1);
  b.push_back(static_cast<std::uint8_t>(crc & 0xff));
  b.push_back(static_cast<std::uint8_t>(crc >> 8));
  b.insert(b.end(), {0x5b, 0xaa});

  outgoing_ = std::move(b);
}

// 等待不早于 timestamp 的新采样帧，本步超时 5 秒
void DepthZero::wait_sample(Stage s, std::uint32_t timestamp, std::uint64_t now) {
  stage_ = s;
  minimum_time_ = timestamp;
  deadline_ = now + 5000;
}

void DepthZero::finish(bool success, const std::string &message) {
  stage_ = Stage::Idle;
  outgoing_.reset();
  result_ = DepthZeroResult{success, message};
}

void DepthZero::cancel(const std::string &reason) {
  if (busy()) {
    finish(false, reason);
  }
}

std::optional<Bytes> DepthZero::take_request() {
  auto b = std::move(outgoing_);
  outgoing_.reset();
  return b;
}

std::optional<DepthZeroResult> DepthZero::take_result() {
  auto r = std::move(result_);
  result_.reset();
  return r;
}

void DepthZero::tick(std::uint64_t now) {
  if (busy() && (now >= deadline_ || now >= overall_deadline_)) {
    finish(false, "Depth zero timed out; no automatic retry. Keep the sensor at the surface and call again.");
  }
}

// 消化一帧上行数据推进状态机；非本事务或非压力设备的帧直接忽略
void DepthZero::observe(const Frame &f, std::uint64_t now) {
  if (!busy() || f.family != 0x5b || f.target != 2) {
    return;
  }
  tick(now);
  if (!busy()) {
    return;
  }

  const auto &b = f.payload;

  // 1. 压力数据帧：flags=8、command=0x82、32 字节载荷
  if (f.flags == 8 && f.command == 0x82 && b.size() == 32) {
    if (last_sample_) {
      // 32 位回绕比较判断采样序号新旧
      const auto delta = static_cast<std::uint32_t>(f.sequence - *last_sample_);
      if (delta == 0 || delta >= 0x80000000u) {
        return;  // 重复或乱序采样丢弃
      }
    }
    last_sample_ = f.sequence;

    if (stage_ != Stage::Pressure && stage_ != Stage::Verify) {
      return;
    }
    if (!f.timestamp_valid || static_cast<std::uint32_t>(f.timestamp_us - minimum_time_) >= 0x80000000u) {
      return;
    }

    const auto state = u32(b, 0);
    const float pressure = f32(b, 4);

    // ONLINE, RAW, PROM and compensated pressure 位校验
    constexpr std::uint32_t kValidPressure = 0x00000113U;
    if ((state & kValidPressure) != kValidPressure || !std::isfinite(pressure) ||
        pressure < 10000 || pressure > 200000) {
      return;
    }

    // 若在 Pressure 阶段，且压力正常，触发 ZERO_DEPTH 去皮指令 (command = 0x0c)
    if (stage_ == Stage::Pressure) {
      request(Stage::Zero, 0x0c, {}, now);
      return;
    }

    // 在 Verify 阶段，校验深度与去皮状态位
    constexpr std::uint32_t kValidDepthAndZero = 0x000001d3U;
    const float raw = f32(b, 12);
    const float filtered = f32(b, 16);
    const float p0 = f32(b, 20);

    if ((state & kValidDepthAndZero) != kValidDepthAndZero ||
        !std::isfinite(raw) || !std::isfinite(filtered) || !std::isfinite(p0)) {
      return;
    }

    if (std::abs(p0 - reference_) > 0.02f) {
      finish(false, "Depth reference changed before verification");
      return;
    }

    std::ostringstream message;
    message << std::fixed << std::setprecision(3)
            << "Surface zero verified, P0=" << reference_
            << " Pa, pressure=" << pressure
            << " Pa, raw_depth=" << raw
            << " m, filtered_depth=" << filtered
            << " m. Reference is stored in MCU RAM.";
    finish(true, message.str());
    return;
  }

  if (stage_ == Stage::Pressure || stage_ == Stage::Verify) {
    return;
  }

  // 2. 指令应答帧匹配
  if ((f.flags != 2 && f.flags != 6) || f.command != expected_command_ || f.sequence != expected_sequence_) {
    return;
  }

  // 载荷首字节为设备返回码，非 0 即拒绝
  if (b.empty() || b[0] != 0 || f.flags != 2) {
    finish(false, "Depth command rejected, result=" + (b.empty() ? std::string("missing") : std::to_string(b[0])));
    return;
  }

  const auto stage = stage_;
  if (stage == Stage::Status) {
    if (b.size() != 21 || !f.timestamp_valid) {
      finish(false, "Invalid depth status reply");
      return;
    }
    wait_sample(Stage::Pressure, f.timestamp_us, now);
  } else if (stage == Stage::Zero) {
    if (b.size() != 1) {
      finish(false, "Invalid ZERO_DEPTH reply");
      return;
    }
    // 触发读取表面基准压力 P0 (command = 3, payload = {0x03, 0x01})
    request(Stage::ReadReference, 3, {0x03, 0x01}, now);
  } else if (stage == Stage::ReadReference) {
    if (b.size() != 9 || u16(b, 1) != 0x0103 || b[3] != 7 || b[4] != 4) {
      finish(false, "Invalid surface pressure readback");
      return;
    }
    reference_ = f32(b, 5);  // 表面大气压基准 (Pa)
    if (!std::isfinite(reference_) || reference_ < 10000 || reference_ > 200000) {
      finish(false, "Invalid surface pressure value");
      return;
    }
    wait_sample(Stage::Verify, f.timestamp_us, now);
  }
}

}  // namespace underwater::wire
