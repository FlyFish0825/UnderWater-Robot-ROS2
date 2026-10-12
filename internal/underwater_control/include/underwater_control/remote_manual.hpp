#pragma once
#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace underwater::control::remote {
using Levels = std::array<int,6>;
using Millis = std::int64_t; // Local steady clock only; never a peer's wall clock.
enum class Kind { Enable, Disable, Input, Auth };
enum class State { Disabled, Enabling, Enabled, Stopping, Fault };
struct Command { Kind kind; std::uint64_t sequence=0,session=0; Levels levels{}; std::string lease_token; };
struct Result { std::uint64_t sequence=0; bool ok=false; std::uint64_t session=0; std::string detail; };
struct Observation {
  bool gateway_ready=false,allocation_fresh=false,armed=false,input_fresh=false,solver_ok=false;
  bool rearm_required=true,motor_active=false;
  std::uint64_t connection=0,gateway_session=0,allocation_session=0;
  std::uint64_t allocation_update=0;
  std::array<int,8> rpm{};
};
struct Actions { bool enable=false,disable=false,zero=false,input=false,release=false; };
std::optional<Command> parse(std::string_view line,std::string &error);
bool valid_levels(const Levels &levels);
std::string hex_detail(std::string_view detail);
std::string result_line(const Result &result);
class LineBuffer {
public:
  bool append(std::string_view bytes,std::vector<std::string> &lines,std::string &error);
  void reset() { pending_.clear(); }
private:
  std::string pending_;
};
class OutputBuffer {
public:
  bool push(std::string_view bytes,Millis at);
  void consume(std::size_t size);
  const std::string &bytes() const { return bytes_; }
  bool stalled(Millis at) const { return !bytes_.empty() && at-started_>=500; }
  void reset() { bytes_.clear(); }
private:
  std::string bytes_;
  Millis started_=0;
};

// Transport and ROS-free authority state machine. All methods run on one executor.
// The adapter executes actions; a timer never republishes stored nonzero input.
class Session {
public:
  void connect();
  void clear_completed_fault();
  void disconnect(Millis at);
  // Enable returns admission (session=0), NOT the final wire RESULT. The ROS
  // adapter defers that response until Enabled with a real zero-ACK session.
  Result handle(const Command &command,Millis at,bool ready);
  void observe(const Observation &observation,Millis at);
  void tick(Millis at);
  void enable_result(bool ok,const std::string &detail,Millis at);
  void disable_result(bool ok,Millis at);
  void disable_result(bool ok,Millis at,std::uint64_t generation);
  void stop(const std::string &detail,bool fault,Millis at);
  Actions take_actions();
  State state() const { return state_; }
  bool owns_publisher() const { return owned_; }
  bool enable_pending() const { return enable_pending_; }
  std::uint64_t disable_generation() const { return disable_generation_; }
  std::uint64_t session() const { return session_; }
  std::uint64_t last_input_sequence() const { return last_input_sequence_; }
  const Levels &levels() const { return levels_; }
  const std::string &detail() const { return detail_; }
private:
  void finish_stop();
  State state_=State::Disabled;
  bool connected_=false,owned_=false,enable_pending_=false,disable_confirmed_=false,fault_=false;
  std::uint64_t sequence_=0,session_=0,baseline_session_=0,connection_=0,last_input_sequence_=0;
  std::uint64_t input_observation_=0;
  std::uint64_t disable_generation_=0;
  Millis started_=0,input_at_=0,disable_at_=0;
  Levels levels_{};
  Observation observation_{};
  Actions actions_{};
  std::string detail_="disabled";
};
std::string state_line(const Session &session,std::uint64_t counter,const Observation &observation);
} // namespace underwater::control::remote
