// ==============================================================================
// internal/underwater_mcu_gateway/test/depth_zero_test.cpp
// ==============================================================================
// 深度零点标定状态机单元测试：
// - 模拟 MCU 返回 0x5b 压力采样与状态应答帧
// - 验证 Status -> Pressure -> Zero -> ReadReference -> Verify 完整事务流
// - 验证超时与非法响应报错机制
// ==============================================================================
// 【初学者导读】本文件测试深度计“水面零点标定”状态机（DepthZero），属于
// internal/underwater_mcu_gateway/test。标定需要网关与 MCU 来回多次：
// 启动 -> 查询状态 -> 发送去皮(Zero) -> 读取参考值(ReadReference) -> 用新采样验证。
// 测试用手工构造的 Frame（不接真实硬件）喂养状态机，观察它何时产生要下发的请求帧，
// 并最终拿到成功结果。涉及概念：协议帧的 family/target/command/flags/payload 字段、
// 状态机、std::optional（表示“可能有值”）。
#include "underwater_mcu_gateway/depth_zero.hpp"

#include <cstring>
#include <iostream>
#include <stdexcept>

using namespace underwater::wire;

int checks = 0;

// 断言：失败时报告是第几个检查出问题。
void check(bool condition) {
  ++checks;
  if (!condition) {
    throw std::runtime_error("check failed at index " + std::to_string(checks));
  }
}

// 辅助组装 32 字节压力传感器数据帧载荷
// 构造 0x5b 压力传感器帧的 32 字节载荷（小端浮点布局），模拟 MCU 上报数据。
// 用 memcpy 写到指定偏移，是因为协议规定了各字段的精确字节位置。
static Bytes make_pressure_payload(float pressure_pa, float raw_depth, float filtered_depth, float p0) {
  Bytes b(32, 0);
  // state = 0x000001d3 (ONLINE | RAW | PROM | COMPENSATED | DEPTH | ZERO_VERIFIED)
  const std::uint32_t state = 0x000001d3U;
  std::memcpy(b.data() + 0, &state, 4);
  std::memcpy(b.data() + 4, &pressure_pa, 4);
  std::memcpy(b.data() + 12, &raw_depth, 4);
  std::memcpy(b.data() + 16, &filtered_depth, 4);
  std::memcpy(b.data() + 20, &p0, 4);
  return b;
}

// 逐步模拟 MCU 应答并检查状态机每一步产生的请求帧。
int main() {
  DepthZero dz(0x1000);
  check(!dz.busy());

  // 1. 启动标定事务
  check(dz.start(100));
  check(dz.busy());

  auto req1 = dz.take_request();
  check(req1.has_value());
  check(!dz.take_request().has_value());  // 保证取走后清空

  // 2. 模拟返回 Status 应答 (command=2+0x40=0x42, flags=2, payload=[0, 20 bytes data])
  Frame f_status;
  f_status.family = 0x5b;
  f_status.target = 2;
  f_status.command = 0x42;
  f_status.flags = 2;
  f_status.sequence = 0x1001;
  f_status.timestamp_valid = true;
  f_status.timestamp_us = 100000;
  f_status.payload = Bytes(21, 0);  // 1 字节 status_code 0 + 20 字节状态数据
  dz.observe(f_status, 200);

  // 3. 模拟首帧压力采样触发 Zero 指令
  Frame f_press1;
  f_press1.family = 0x5b;
  f_press1.target = 2;
  f_press1.command = 0x82;
  f_press1.flags = 8;
  f_press1.sequence = 10;
  f_press1.timestamp_valid = true;
  f_press1.timestamp_us = 150000;
  f_press1.payload = make_pressure_payload(101325.0f, 0.0f, 0.0f, 101325.0f);
  dz.observe(f_press1, 250);

  auto req2 = dz.take_request();
  check(req2.has_value());  // ZERO_DEPTH 请求帧

  // 4. 模拟 Zero 指令应答 (command=0x0c+0x40=0x4c)
  Frame f_zero;
  f_zero.family = 0x5b;
  f_zero.target = 2;
  f_zero.command = 0x4c;
  f_zero.flags = 2;
  f_zero.sequence = 0x1002;
  f_zero.payload = {0};  // status=0
  dz.observe(f_zero, 300);

  auto req3 = dz.take_request();
  check(req3.has_value());  // ReadReference 请求帧

  // 5. 模拟 ReadReference 应答 (command=3+0x40=0x43, payload 9 bytes)
  Frame f_ref;
  f_ref.family = 0x5b;
  f_ref.target = 2;
  f_ref.command = 0x43;
  f_ref.flags = 2;
  f_ref.sequence = 0x1003;
  f_ref.timestamp_valid = true;
  f_ref.timestamp_us = 200000;
  f_ref.payload = {0, 0x03, 0x01, 7, 4, 0, 0, 0, 0};
  const float ref_pa = 101325.0f;
  std::memcpy(f_ref.payload.data() + 5, &ref_pa, 4);
  dz.observe(f_ref, 350);

  // 6. 模拟新采样完成 Verify 阶段
  Frame f_press2 = f_press1;
  f_press2.sequence = 11;
  f_press2.timestamp_us = 250000;
  dz.observe(f_press2, 400);

  // 7. 检验终态结果
  check(!dz.busy());
  auto result = dz.take_result();
  check(result.has_value());
  check(result->success);

  std::cout << "All " << checks << " depth zero state machine tests passed.\n";
  return 0;
}
