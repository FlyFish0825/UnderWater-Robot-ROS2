// ==============================================================================
// internal/underwater_mcu_gateway/include/underwater_mcu_gateway/depth_zero.hpp
// ==============================================================================
// 深度零点标定状态机 (Status → Pressure → Zero → ReadReference → Verify)：
// - 请求以 0x5b family、target=2 (压力传感器) 发出，期望回复命令字 = 请求命令 + 0x40；
// - 单步超时为 2s (命令应答) 或 5s (等待新采样)，整事务上限 15s；
// - 失败时不执行自动重试，保证操作员必须将传感器置于水面重新发起调用。
// ==============================================================================
// 初学者导读：
// 1) 所属：internal/underwater_mcu_gateway，深度零点标定用的"非重入状态机"。
//    物理含义：把压力计放到水面，把此刻的气压/深度记为 0 点（去皮），之后算深度才准。
// 2) 交互：网关节点把上行 Frame 喂给 observe()，把 take_request() 取出的字节写串口，
//    再把 take_result() 的结果回报给操作员/服务接口。
// 3) ROS 2 概念：本文件是纯 C++ 逻辑，不含 ROS API；通常由 ROS 服务或话题触发标定。
// 4) C++ 概念：enum class Stage 表示状态机阶段；std::optional<T> 表示"可能没有值"；
//    std::move 把对象内容搬走、避免拷贝；std::uint64_t 存毫秒时间戳。
#pragma once

#include "underwater_mcu_gateway/protocol.hpp"

#include <optional>
#include <string>

namespace underwater::wire {

// 深度零点标定执行结果
// 标定事务的最终结果：success 是否成功，message 给操作员看的说明。
struct DepthZeroResult {
  bool success = false;
  std::string message;
};

// 非重入的单事务状态机：同一时刻最多持有一个待发请求和一个就绪结果
// 深度零点标定状态机：同一时刻只允许一个标定事务。
// 调用节奏：start() -> 反复 (observe()/tick() + take_request()) -> take_result()。
class DepthZero {
public:
  explicit DepthZero(std::uint32_t seed = 0x52000000)
      : sequence_(seed) {}

  // 查询状态机当前是否处于忙碌标定流程中
  bool busy() const { return stage_ != Stage::Idle; }

  // 启动标定事务：若已在运行则返回 false；成功则排入第 1 帧状态查询请求
// 开始标定：已在忙时返回 false；成功则排入第 1 帧"状态查询"请求。
  bool start(std::uint64_t now_ms);

  // 接收并处理一帧上行数据推进状态机；与本事务无关的帧自动忽略
// 消化一帧上行数据；只认 0x5b family、target=2(压力计) 且属于本事务的帧。
  void observe(const Frame &frame, std::uint64_t now_ms);

  // 周期性超时巡检
// 周期调用做超时巡检；超时只会失败，不会自动重试（必须重新把传感器放回水面再发起）。
  void tick(std::uint64_t now_ms);

  // 强制取消标定
  void cancel(const std::string &reason);

  // 提取待发请求字节流 (提取后内部清空)
// 取出并清空待发请求字节；返回 std::optional，无请求时为空。
  std::optional<Bytes> take_request();

  // 提取标定终态结果 (提取后内部清空)
  std::optional<DepthZeroResult> take_result();

private:
  // 标定内部阶段枚举
// 状态机内部阶段：Idle 空闲；Status 等状态查询应答；Pressure 等有效压力采样；
// Zero 等去皮应答；ReadReference 等基准 P0 应答；Verify 等新采样校验。
  enum class Stage {
    Idle,           // 空闲
    Status,         // 等待状态查询响应
    Pressure,       // 等待有效压力采样
    Zero,           // 等待去皮 ZERO_DEPTH 响应
    ReadReference,  // 等待读取基准 P0 响应
    Verify          // 等待新采样校验基准一致性
  };

// 组装一帧 0x5b 请求（target=2 压力设备），期望应答命令字 = 命令 + 0x40。
  void request(Stage stage, std::uint8_t command, const Bytes &payload, std::uint64_t now_ms);
// 进入"等待新采样"阶段：只接受时间戳不早于 device_time 的采样帧。
  void wait_sample(Stage stage, std::uint32_t device_time, std::uint64_t now_ms);
// 结束事务：清空待发请求，保存结果供 take_result() 取走。
  void finish(bool success, const std::string &message);

// 状态机内部字段：deadline_ 单步超时点，overall_deadline_ 整事务超时点，
// last_sample_ 已处理的最大采样序号，outgoing_ 待发字节，result_ 终态结果。
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
