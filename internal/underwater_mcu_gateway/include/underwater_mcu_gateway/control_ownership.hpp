#pragma once

#include <chrono>
#include <cstdint>
#include <string>

namespace underwater {

enum class ControllerOwner : std::uint8_t { None = 0, Nano = 1, Gui = 2, GuiManual = 3 };

const char * controller_name(ControllerOwner owner) noexcept;

struct OwnershipResult {
  bool success = false;
  ControllerOwner owner = ControllerOwner::None;
  std::string token;
  std::string message;
};

// Raw GUI owns UART writes; GUI_MANUAL delegates bounded ROS output to Nano.
// Both leases are bound to their management TCP session, never interchangeable.
class ControlOwnership {
public:
  using Clock = std::chrono::steady_clock;

  explicit ControlOwnership(ControllerOwner initial = ControllerOwner::Nano);
  OwnershipResult claim_nano(bool takeover);
  OwnershipResult release_nano();
  OwnershipResult claim_gui(int session, bool takeover, bool manual = false);
  OwnershipResult release_gui(int session);
  bool heartbeat_gui(int session, const std::string &token);
  bool disconnect_gui(int session);
  bool expire_gui(Clock::time_point now = Clock::now());
  std::uint64_t validate_manual_lease(const std::string &token,
                                    Clock::time_point now = Clock::now());
  bool manual_authorized(Clock::time_point now = Clock::now()) const noexcept;
  std::uint64_t generation() const noexcept { return generation_; }

  ControllerOwner owner() const noexcept { return owner_; }
  const std::string &gui_token() const noexcept { return token_; }
  int gui_session() const noexcept { return gui_session_; }

private:
  std::string new_token();
  void set_owner(ControllerOwner owner, int session = -1);

  ControllerOwner owner_ = ControllerOwner::Nano;
  int gui_session_ = -1;
  std::string token_;
  Clock::time_point last_heartbeat_{};
  Clock::time_point manual_until_{};
  std::uint64_t generation_ = 1;
  std::uint64_t token_sequence_ = 0;
};

}  // namespace underwater
