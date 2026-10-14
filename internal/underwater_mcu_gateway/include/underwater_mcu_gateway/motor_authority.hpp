// ==============================================================================
// internal/underwater_mcu_gateway/include/underwater_mcu_gateway/motor_authority.hpp
// ==============================================================================
// 网关电机控制下发权限决策策略：
// 纯策略逻辑，由网关节点与离线测试共享；
// 当 synchronize() 返回 true 时调用方负责下发串口全零停机指令。
// ==============================================================================
#pragma once

#include "underwater_mcu_gateway/control_ownership.hpp"
#include "underwater_mcu_gateway/motor_commands.hpp"

namespace underwater {

class MotorAuthority {
public:
  using Clock = ControlOwnership::Clock;

  struct LeaseRenewal {
    std::uint64_t generation = 0;
    bool stop_required = false;
  };

  MotorAuthority(ControlOwnership &ownership, wire::MotorGate &gate)
      : ownership_(ownership),
        gate_(gate),
        generation_(ownership.generation()) {}

  // 续期手动租约：单次采样保证时钟严密性，避免在过期跨越中授权失步
  LeaseRenewal renew_manual_lease(const std::string &token, Clock::time_point now = Clock::now()) {
    const bool stop = synchronize(now);
    return {ownership_.validate_manual_lease(token, now), stop};
  }

  // 同步所有权状态：若所有权或代际发生切换，触发 gate_ 复位并要求停机
  bool synchronize(Clock::time_point now = Clock::now()) {
    ownership_.expire_gui(now);
    const bool authorized = ownership_.manual_authorized(now);
    const bool changed = (generation_ != ownership_.generation());
    const bool expired = (manual_authorized_ && !authorized);

    generation_ = ownership_.generation();
    manual_authorized_ = authorized;

    if (!changed && !expired) {
      return false;
    }
    gate_.reset();
    return true;
  }

  // 查询当前是否允许通过 ROS 话题下发电机指令
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
