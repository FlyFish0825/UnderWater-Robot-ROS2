// 控制权仲裁实现：同一时刻只有一个所有者，串口写权限随之互斥。
// GUI 的 token 与其控制 TCP 会话（fd）绑定，心跳丢失 3s 后自动交还 Nano。
// 假定由节点主线程调用，内部无锁；now 可注入以便测试。
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

// 枚举转协议字符串；未知取值按 NONE 处理。
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

// 32 位随机数与递增计数器异或后拼成 32 位十六进制，避免 token 可预测或重复。
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

// 所有权限变更的唯一出口：只有 GUI 会持有 token 与心跳时间。
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
  if (owner_ == ControllerOwner::Nano) // 幂等：已是 Nano 直接成功
    return {true, owner_, {}, "Nano already owns robot control"};
  if (owner_ != ControllerOwner::None && !takeover) // 被占用时必须显式 takeover
    return {false, owner_, {}, "Robot control is occupied; use TAKEOVER to preempt the current owner"};
  set_owner(ControllerOwner::Nano);
  return {true, owner_, {}, takeover ? "Nano took over robot control" : "Nano acquired robot control"};
}

// 只有持有者本人能释放；释放后控制权变为 None。
OwnershipResult ControlOwnership::release_nano() {
  if (owner_ != ControllerOwner::Nano)
    return {false, owner_, {}, std::string("Nano cannot release control owned by ") + controller_name(owner_)};
  set_owner(ControllerOwner::None);
  return {true, owner_, {}, "Nano released robot control"};
}

// 同一 GUI 会话重复声明是幂等的（返回原 token）；被占用时需 takeover 抢占；manual=true 获取 GUI_MANUAL 权限。
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

// 会话与 token 都匹配才续期；失败说明权限已被抢占或回收。
bool ControlOwnership::heartbeat_gui(const int session, const std::string &token) {
  const auto now = Clock::now();
  if (!gui_owner(owner_) || gui_session_ != session || token.empty() || token != token_
      || now - last_heartbeat_ >= std::chrono::seconds(3))
    return false;
  last_heartbeat_ = now;
  return true;
}

// 控制 TCP 断开时调用，把控制权交还 Nano。
bool ControlOwnership::disconnect_gui(const int session) {
  if (!gui_owner(owner_) || gui_session_ != session)
    return false;
  set_owner(ControllerOwner::Nano);
  return true;
}

// 心跳超过 3s 未更新即收回控制权（GUI 崩溃/断网时保证 Nano 能停桨）。
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
