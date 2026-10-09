#include "underwater_mcu_gateway/control_ownership.hpp"

#include <array>
#include <random>
#include <sstream>
#include <stdexcept>

namespace underwater {

const char *controller_name(const ControllerOwner owner) noexcept {
  switch (owner) {
  case ControllerOwner::Nano: return "NANO";
  case ControllerOwner::Gui: return "GUI";
  case ControllerOwner::None: return "NONE";
  }
  return "NONE";
}

ControlOwnership::ControlOwnership(const ControllerOwner initial) : owner_(initial) {}

std::string ControlOwnership::new_token() {
  std::random_device random;
  std::ostringstream text;
  for (int i = 0; i < 4; ++i) {
    const auto value = static_cast<std::uint32_t>(random()) ^
        static_cast<std::uint32_t>(++token_sequence_ * 0x9E3779B9U);
    text.width(8);
    text.fill('0');
    text << std::hex << value;
  }
  return text.str();
}

void ControlOwnership::set_owner(const ControllerOwner owner, const int session) {
  owner_ = owner;
  gui_session_ = owner == ControllerOwner::Gui ? session : -1;
  token_ = owner == ControllerOwner::Gui ? new_token() : std::string{};
  last_heartbeat_ = owner == ControllerOwner::Gui ? Clock::now() : Clock::time_point{};
}

OwnershipResult ControlOwnership::claim_nano(const bool takeover) {
  if (owner_ == ControllerOwner::Nano)
    return {true, owner_, {}, "Nano already owns robot control"};
  if (owner_ != ControllerOwner::None && !takeover)
    return {false, owner_, {}, "Robot control is occupied; use TAKEOVER to preempt the current owner"};
  set_owner(ControllerOwner::Nano);
  return {true, owner_, {}, takeover ? "Nano took over robot control" : "Nano acquired robot control"};
}

OwnershipResult ControlOwnership::release_nano() {
  if (owner_ != ControllerOwner::Nano)
    return {false, owner_, {}, std::string("Nano cannot release control owned by ") + controller_name(owner_)};
  set_owner(ControllerOwner::None);
  return {true, owner_, {}, "Nano released robot control"};
}

OwnershipResult ControlOwnership::claim_gui(const int session, const bool takeover) {
  if (session < 0)
    return {false, owner_, {}, "Invalid GUI control session"};
  if (owner_ == ControllerOwner::Gui && gui_session_ == session)
    return {true, owner_, token_, "GUI already owns robot control"};
  if (owner_ != ControllerOwner::None && !takeover)
    return {false, owner_, {}, std::string("Robot control is occupied by ") + controller_name(owner_)
        + "; use TAKEOVER to preempt the current owner"};
  set_owner(ControllerOwner::Gui, session);
  return {true, owner_, token_, takeover ? "GUI took over robot control" : "GUI acquired robot control"};
}

OwnershipResult ControlOwnership::release_gui(const int session) {
  if (owner_ != ControllerOwner::Gui || gui_session_ != session)
    return {false, owner_, {}, std::string("GUI does not own robot control; current owner is ")
        + controller_name(owner_)};
  set_owner(ControllerOwner::Nano);
  return {true, owner_, {}, "GUI released robot control"};
}

bool ControlOwnership::heartbeat_gui(const int session, const std::string &token) {
  if (owner_ != ControllerOwner::Gui || gui_session_ != session || token != token_)
    return false;
  last_heartbeat_ = Clock::now();
  return true;
}

bool ControlOwnership::disconnect_gui(const int session) {
  if (owner_ != ControllerOwner::Gui || gui_session_ != session)
    return false;
  set_owner(ControllerOwner::Nano);
  return true;
}

bool ControlOwnership::expire_gui(const Clock::time_point now) {
  if (owner_ != ControllerOwner::Gui || now - last_heartbeat_ <= std::chrono::seconds(3))
    return false;
  set_owner(ControllerOwner::Nano);
  return true;
}

}  // namespace underwater
