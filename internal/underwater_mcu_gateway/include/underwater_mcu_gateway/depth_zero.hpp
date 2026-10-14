// ==============================================================================
// internal/underwater_mcu_gateway/include/underwater_mcu_gateway/depth_zero.hpp
// ==============================================================================
// 深度零点标定状态机 (Status → Pressure → Zero → ReadReference → Verify)：
// - 请求以 0x5b family、target=2 (压力传感器) 发出，期望回复命令字 = 请求命令 + 0x40；
// - 单步超时为 2s (命令应答) 或 5s (等待新采样)，整事务上限 15s；
// - 失败时不执行自动重试，保证操作员必须将传感器置于水面重新发起调用。
// ==============================================================================
#pragma once

#include "underwater_mcu_gateway/protocol.hpp"

#include <optional>
#include <string>

namespace underwater::wire {

// 深度零点标定执行结果
struct DepthZeroResult {
  bool success = false;
  std::string message;
};

// 非重入的单事务状态机：同一时刻最多持有一个待发请求和一个就绪结果
class DepthZero {
public:
  explicit DepthZero(std::uint32_t seed = 0x52000000)
      : sequence_(seed) {}

  // 查询状态机当前是否处于忙碌标定流程中
  bool busy() const { return stage_ != Stage::Idle; }

  // 启动标定事务：若已在运行则返回 false；成功则排入第 1 帧状态查询请求
  bool start(std::uint64_t now_ms);

  // 接收并处理一帧上行数据推进状态机；与本事务无关的帧自动忽略
  void observe(const Frame &frame, std::uint64_t now_ms);

  // 周期性超时巡检
  void tick(std::uint64_t now_ms);

  // 强制取消标定
  void cancel(const std::string &reason);

  // 提取待发请求字节流 (提取后内部清空)
  std::optional<Bytes> take_request();

  // 提取标定终态结果 (提取后内部清空)
  std::optional<DepthZeroResult> take_result();

private:
  // 标定内部阶段枚举
  enum class Stage {
    Idle,           // 空闲
    Status,         // 等待状态查询响应
    Pressure,       // 等待有效压力采样
    Zero,           // 等待去皮 ZERO_DEPTH 响应
    ReadReference,  // 等待读取基准 P0 响应
    Verify          // 等待新采样校验基准一致性
  };

  void request(Stage stage, std::uint8_t command, const Bytes &payload, std::uint64_t now_ms);
  void wait_sample(Stage stage, std::uint32_t device_time, std::uint64_t now_ms);
  void finish(bool success, const std::string &message);

  Stage stage_ = Stage::Idle;
  std::uint32_t sequence_ = 0;
  std::uint32_t expected_sequence_ = 0;
  std::uint32_t minimum_time_ = 0;
  std::uint8_t expected_command_ = 0;
  std::uint64_t deadline_ = 0;
  std::uint64_t overall_deadline_ = 0;

  std::optional<std::uint32_t> last_sample_;
  std::optional<Bytes> outgoing_;
  std::optional<DepthZeroResult> result_;
  float reference_ = 0.0f;
};

}  // namespace underwater::wire
