#pragma once
// 深度零点标定状态机（Status → Zero → ReadReference → Verify）。
// 请求以 0x5b family、target=2（压力设备）发出，期望回复命令 = 请求命令 + 0x40。
// 单步超时 2s/5s、整事务 15s；失败不自动重试，必须由调用方重新 start()。
#include "underwater_mcu_gateway/protocol.hpp"
#include <optional>
#include <string>
namespace underwater::wire {
struct DepthZeroResult { bool success=false; std::string message; };
// 非重入的单事务状态机：同一时刻最多持有一个待发请求和一个结果。
class DepthZero {
public:
  explicit DepthZero(std::uint32_t seed=0x52000000):sequence_(seed) {}
  bool busy() const { return stage_!=Stage::Idle; }
  // 已在运行则返回 false；成功则排入第一帧状态查询请求。
  bool start(std::uint64_t now_ms);
  // 消化一帧上行数据推进状态机；与本事务无关的帧会被忽略。
  void observe(const Frame &,std::uint64_t now_ms);
  void tick(std::uint64_t now_ms);
  void cancel(const std::string & reason);
  std::optional<Bytes> take_request();
  std::optional<DepthZeroResult> take_result();
private:
  enum class Stage { Idle,Status,Pressure,Zero,ReadReference,Verify };
  void request(Stage,std::uint8_t,const Bytes &,std::uint64_t);
  void wait_sample(Stage,std::uint32_t device_time,std::uint64_t);
  void finish(bool,const std::string &);
  Stage stage_=Stage::Idle;
  std::uint32_t sequence_=0,expected_sequence_=0,minimum_time_=0;
  std::uint8_t expected_command_=0;
  std::uint64_t deadline_=0,overall_deadline_=0;
  std::optional<std::uint32_t> last_sample_;
  std::optional<Bytes> outgoing_;
  std::optional<DepthZeroResult> result_;
  float reference_=0;
};
}
