#include "underwater_mcu_gateway/control_ownership.hpp"

#include <array>
#include <random>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace underwater {
namespace {
bool gui_owner(ControllerOwner owner) {
  return owner == ControllerOwner::Gui || owner == ControllerOwner::GuiManual;
}
}

const char *controller_name(const ControllerOwner owner) noexcept {
  switch (owner) {
  case ControllerOwner::Nano: return "NANO";
  case ControllerOwner::Gui: return "GUI";
  case ControllerOwner::GuiManual: return "GUI_MANUAL";
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
  auto next_token = gui_owner(owner) ? new_token() : std::string{};
  owner_ = owner;
  gui_session_ = gui_owner(owner) ? session : -1;
  token_ = std::move(next_token);
  last_heartbeat_ = gui_owner(owner) ? Clock::now() : Clock::time_point{};
  manual_until_ = Clock::time_point{};
  if (++generation_ == 0) ++generation_;
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

OwnershipResult ControlOwnership::claim_gui(const int session, const bool takeover, const bool manual) {
  if (session < 0)
    return {false, owner_, {}, "Invalid GUI control session"};
  expire_gui();
  const auto requested = manual ? ControllerOwner::GuiManual : ControllerOwner::Gui;
  if (owner_ == requested && gui_session_ == session && !takeover)
    return {true, owner_, token_, "GUI already owns robot control"};
  if (owner_ != ControllerOwner::None && !takeover)
    return {false, owner_, {}, std::string("Robot control is occupied by ") + controller_name(owner_)
        + "; use TAKEOVER to preempt the current owner"};
  set_owner(requested, session);
  return {true, owner_, token_, takeover ? "GUI took over robot control" : "GUI acquired robot control"};
}

OwnershipResult ControlOwnership::release_gui(const int session) {
  if (!gui_owner(owner_) || gui_session_ != session)
    return {false, owner_, {}, std::string("GUI does not own robot control; current owner is ")
        + controller_name(owner_)};
  set_owner(ControllerOwner::Nano);
  return {true, owner_, {}, "GUI released robot control"};
}

bool ControlOwnership::heartbeat_gui(const int session, const std::string &token) {
  const auto now = Clock::now();
  if (!gui_owner(owner_) || gui_session_ != session || token.empty() || token != token_
      || now - last_heartbeat_ >= std::chrono::seconds(3))
    return false;
  last_heartbeat_ = now;
  return true;
}

bool ControlOwnership::disconnect_gui(const int session) {
  if (!gui_owner(owner_) || gui_session_ != session)
    return false;
  set_owner(ControllerOwner::Nano);
  return true;
}

bool ControlOwnership::expire_gui(const Clock::time_point now) {
  if (!gui_owner(owner_) || now - last_heartbeat_ < std::chrono::seconds(3))
    return false;
  set_owner(ControllerOwner::Nano);
  return true;
}

std::uint64_t ControlOwnership::validate_manual_lease(const std::string &token,
                                                     const Clock::time_point now) {
  if (owner_ != ControllerOwner::GuiManual || gui_session_ < 0 || token.empty()
      || token != token_ || now - last_heartbeat_ >= std::chrono::seconds(3))
    return 0;
  // Invalid requests neither revoke nor extend a valid holder's short proof.
  manual_until_ = now + std::chrono::milliseconds(500);
  return generation_;
}

bool ControlOwnership::manual_authorized(const Clock::time_point now) const noexcept {
  return owner_ == ControllerOwner::GuiManual && gui_session_ >= 0 && !token_.empty()
      && now - last_heartbeat_ < std::chrono::seconds(3) && now < manual_until_;
}

}  // namespace underwater
