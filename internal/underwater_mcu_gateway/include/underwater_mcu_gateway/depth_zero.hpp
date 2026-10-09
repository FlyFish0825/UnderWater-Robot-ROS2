#pragma once
#include "underwater_mcu_gateway/protocol.hpp"
#include <optional>
#include <string>
namespace underwater::wire {
struct DepthZeroResult { bool success=false; std::string message; };
class DepthZero {
public:
  explicit DepthZero(std::uint32_t seed=0x52000000):sequence_(seed) {}
  bool busy() const { return stage_!=Stage::Idle; }
  bool start(std::uint64_t now_ms);
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
