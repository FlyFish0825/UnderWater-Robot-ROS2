#pragma once

#include "underwater_mcu_gateway/control_ownership.hpp"
#include "underwater_mcu_gateway/motor_commands.hpp"

namespace underwater {
// Pure policy shared by the gateway and offline tests. The caller performs the
// best-effort UART stop when synchronize() returns true; no I/O occurs here.
class MotorAuthority {
public:
  using Clock = ControlOwnership::Clock;
  struct LeaseRenewal {
    std::uint64_t generation = 0;
    bool stop_required = false;
  };
  MotorAuthority(ControlOwnership &ownership, wire::MotorGate &gate)
      : ownership_(ownership), gate_(gate), generation_(ownership.generation()) {}

  LeaseRenewal renew_manual_lease(const std::string &token, Clock::time_point now = Clock::now()) {
    // One sample for expiry and renewal. A callback that crosses 500ms must
    // never use a pre-expiry synchronization with a post-expiry proof grant.
    const bool stop = synchronize(now);
    return {ownership_.validate_manual_lease(token, now), stop};
  }

  bool synchronize(Clock::time_point now = Clock::now()) {
    ownership_.expire_gui(now);
    const bool authorized = ownership_.manual_authorized(now);
    const bool changed = generation_ != ownership_.generation();
    const bool expired = manual_authorized_ && !authorized;
    generation_ = ownership_.generation();
    manual_authorized_ = authorized;
    if (!changed && !expired) return false;
    gate_.reset();
    return true;
  }

  bool ros_motor_allowed(Clock::time_point now = Clock::now()) const {
    return ownership_.owner() == ControllerOwner::Nano || ownership_.manual_authorized(now);
  }

private:
  ControlOwnership &ownership_;
  wire::MotorGate &gate_;
  std::uint64_t generation_;
  bool manual_authorized_ = false;
};
}  // namespace underwater
