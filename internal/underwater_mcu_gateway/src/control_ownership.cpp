// ==============================================================================
// internal/underwater_mcu_gateway/src/control_ownership.cpp
// ==============================================================================
// 控制权仲裁实现：同一时刻只有一个所有者，串口写权限随之互斥。
// - GUI 的 token 与其控制 TCP 会话 (fd) 绑定，心跳丢失 3s 后自动交还 Nano；
// - 假定由节点主线程单线程调用，内部无锁；
// - 支持 now 注入以便进行高精度时序单元测试。
// ==============================================================================
// 初学者导读：
// 1) 本文件实现 control_ownership.hpp：控制权仲裁的全部状态迁移。
// 2) 交互：由 control_protocol.cpp（9002 管理通道）调用；结果影响 TcpSerialBridge 能否写串口，
//    以及 MotorAuthority 是否放行 ROS 电机话题。
// 3) ROS 2 概念：本文件无 ROS API，纯 C++；ROS 侧只消费它的状态。
// 4) C++ 概念：std::random_device 提供随机数用于 token；std::ostringstream 做十六进制格式化；
//    noexcept 承诺不抛异常；std::chrono 时长比较；Clock::time_point 注入 now 便于测试。
#include "underwater_mcu_gateway/control_ownership.hpp"

#include <array>
#include <random>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace underwater {

namespace {

// 辅助判断：Gui 与 GuiManual 都算"GUI 持有"。
bool gui_owner(ControllerOwner owner) {
  return owner == ControllerOwner::Gui || owner == ControllerOwner::GuiManual;
}

}  // namespace

// 枚举转协议字符串；未知取值按 NONE 处理
// 把枚举转成协议字符串（GUI 端直接显示这个字段）。
const char *controller_name(const ControllerOwner owner) noexcept {
  switch (owner) {
    case ControllerOwner::Nano:      return "NANO";
    case ControllerOwner::Gui:       return "GUI";
    case ControllerOwner::GuiManual: return "GUI_MANUAL";
    case ControllerOwner::None:      return "NONE";
  }
  return "NONE";
}

// 构造：默认由 Nano(机载 ROS) 持有控制权。
ControlOwnership::ControlOwnership(const ControllerOwner initial)
    : owner_(initial) {}

// 32 位随机数与递增计数器异或后拼成 32 位十六进制，避免 token 可预测或重复
// token 由随机数与递增计数器混合生成，既不可预测也不会重复。
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

// 所有权限变更的唯一出口：只有 GUI 会持有 token 与心跳时间
// 控制权变更的唯一出口：负责生成/清空 token、记录会话与心跳、递增 generation_。
void ControlOwnership::set_owner(const ControllerOwner owner, const int session) {
  auto next_token = gui_owner(owner) ? new_token() : std::string{};
  owner_ = owner;
  gui_session_ = gui_owner(owner) ? session : -1;
  token_ = std::move(next_token);
  last_heartbeat_ = gui_owner(owner) ? Clock::now() : Clock::time_point{};
  manual_until_ = Clock::time_point{};

  if (++generation_ == 0) {
    ++generation_;
  }
}

// Nano 申请：若发现被占用且未显式 takeover，则拒绝，让上层提示操作员。
OwnershipResult ControlOwnership::claim_nano(const bool takeover) {
  if (owner_ == ControllerOwner::Nano) {
    // 幂等：已是 Nano 直接成功
    return {true, owner_, {}, "Nano already owns robot control"};
  }
  if (owner_ != ControllerOwner::None && !takeover) {
    // 被占用时必须显式 takeover 抢占
    return {false, owner_, {}, "Robot control is occupied; use TAKEOVER to preempt the current owner"};
  }
  set_owner(ControllerOwner::Nano);
  return {true, owner_, {}, takeover ? "Nano took over robot control" : "Nano acquired robot control"};
}

// 只有持有者本人能释放；释放后控制权变为 None
OwnershipResult ControlOwnership::release_nano() {
  if (owner_ != ControllerOwner::Nano) {
    return {false, owner_, {}, std::string("Nano cannot release control owned by ") + controller_name(owner_)};
  }
  set_owner(ControllerOwner::None);
  return {true, owner_, {}, "Nano released robot control"};
}

// 同一 GUI 会话重复声明是幂等的 (返回原 token)；被占用时需 takeover 抢占；manual=true 获取 GUI_MANUAL 权限
// GUI 申请：同一会话重复声明是幂等的，会返回原 token。
OwnershipResult ControlOwnership::claim_gui(const int session, const bool takeover, const bool manual) {
  if (session < 0) {
    return {false, owner_, {}, "Invalid GUI control session"};
  }
  expire_gui();
  const auto requested = manual ? ControllerOwner::GuiManual : ControllerOwner::Gui;
  if (owner_ == requested && gui_session_ == session && !takeover) {
    return {true, owner_, token_, "GUI already owns robot control"};
  }
  if (owner_ != ControllerOwner::None && !takeover) {
    return {false, owner_, {}, std::string("Robot control is occupied by ") + controller_name(owner_) +
            "; use TAKEOVER to preempt the current owner"};
  }
  set_owner(requested, session);
  return {true, owner_, token_, takeover ? "GUI took over robot control" : "GUI acquired robot control"};
}

OwnershipResult ControlOwnership::release_gui(const int session) {
  if (!gui_owner(owner_) || gui_session_ != session) {
    return {false, owner_, {}, std::string("GUI does not own robot control; current owner is ") +
            controller_name(owner_)};
  }
  set_owner(ControllerOwner::Nano);
  return {true, owner_, {}, "GUI released robot control"};
}

// 会话与 token 都匹配才续期；失败说明权限已被抢占或回收
// 心跳：会话或 token 任一不匹配都返回 false，提示 GUI 权限已被回收/抢占。
bool ControlOwnership::heartbeat_gui(const int session, const std::string &token) {
  const auto now = Clock::now();
  if (!gui_owner(owner_) || gui_session_ != session || token.empty() || token != token_ ||
      now - last_heartbeat_ >= std::chrono::seconds(3)) {
    return false;
  }
  last_heartbeat_ = now;
  return true;
}

// 控制 TCP 断开时调用，把控制权安全交还 Nano
bool ControlOwnership::disconnect_gui(const int session) {
  if (!gui_owner(owner_) || gui_session_ != session) {
    return false;
  }
  set_owner(ControllerOwner::Nano);
  return true;
}

// 心跳超过 3s 未更新即收回控制权 (GUI 崩溃/断网时保证 Nano 能停桨)
bool ControlOwnership::expire_gui(const Clock::time_point now) {
  if (!gui_owner(owner_) || (now - last_heartbeat_ < std::chrono::seconds(3))) {
    return false;
  }
  set_owner(ControllerOwner::Nano);
  return true;
}

// GUI_MANUAL 是"短租约"：每次校验成功只续 500ms，调用方需持续续期才持续授权。
std::uint64_t ControlOwnership::validate_manual_lease(
    const std::string &token,
    const Clock::time_point now) {
  if (owner_ != ControllerOwner::GuiManual || gui_session_ < 0 || token.empty() ||
      token != token_ || (now - last_heartbeat_ >= std::chrono::seconds(3))) {
    return 0;
  }
  // 500ms 短期有效凭证
  manual_until_ = now + std::chrono::milliseconds(500);
  return generation_;
}

// 纯查询、不修改状态，所以可以加 const noexcept。
bool ControlOwnership::manual_authorized(const Clock::time_point now) const noexcept {
  return owner_ == ControllerOwner::GuiManual &&
         gui_session_ >= 0 &&
         !token_.empty() &&
         (now - last_heartbeat_ < std::chrono::seconds(3)) &&
         (now < manual_until_);
}

}  // namespace underwater
