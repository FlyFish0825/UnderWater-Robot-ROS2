// ==============================================================================
// internal/underwater_mcu_gateway/include/underwater_mcu_gateway/motor_authority.hpp
// ==============================================================================
// 网关电机控制下发权限决策策略：
// 纯策略逻辑，由网关节点与离线测试共享；
// 当 synchronize() 返回 true 时调用方负责下发串口全零停机指令。
// ==============================================================================
// 初学者导读：
// 1) 所属：internal/underwater_mcu_gateway。它是"要不要允许把电机指令下发给推进器"的策略类。
// 2) 交互：网关节点每轮循环调用 synchronize()/ros_motor_allowed()；当需要停机时，
//    由节点负责往串口写全零停机帧。本类只做判断，不自己发字节。
// 3) ROS 2 概念：本文件不直接写 ROS API，但 ros_motor_allowed() 的返回值决定
//    订阅到的电机话题消息能否被采纳（Nano 或有效的 GUI_MANUAL 才放行）。
// 4) C++ 概念：引用成员（ControlOwnership& / wire::MotorGate&）表示"只借用、不拥有"；
//    嵌套 struct LeaseRenewal 把一次续期的多个返回值打包。
#pragma once

#include "underwater_mcu_gateway/control_ownership.hpp"
#include "underwater_mcu_gateway/motor_commands.hpp"

namespace underwater {

// 电机权限策略：把"控制权状态"翻译成"是否允许下发 + 是否需要写停机帧"。
// 只依赖 ControlOwnership 和 MotorGate 两个引用对象，便于离线单元测试。
class MotorAuthority {
public:
  using Clock = ControlOwnership::Clock;

// renew_manual_lease() 的返回：generation 为续期后代际号(0 表示失败)，stop_required 表示需要下发停机。
  struct LeaseRenewal {
    std::uint64_t generation = 0;
    bool stop_required = false;
  };

  MotorAuthority(ControlOwnership &ownership, wire::MotorGate &gate)
      : ownership_(ownership),
        gate_(gate),
        generation_(ownership.generation()) {}

  // 续期手动租约：单次采样保证时钟严密性，避免在过期跨越中授权失步
// 续期 GUI_MANUAL 手动租约：同一次采样内先同步所有权再校验租约，避免跨过过期瞬间导致误授权。
  LeaseRenewal renew_manual_lease(const std::string &token, Clock::time_point now = Clock::now()) {
    const bool stop = synchronize(now);
    return {ownership_.validate_manual_lease(token, now), stop};
  }

  // 同步所有权状态：若所有权或代际发生切换，触发 gate_ 复位并要求停机
// 同步控制权：先让 ownership 处理心跳超时，再比较代际号/授权状态；
// 一旦换人或授权失效，就复位电机闸门并返回 true，表示调用方必须写全零停机帧。
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
// 是否允许 ROS 话题指令：默认 Nano 可下；GUI 模式下必须持有有效的 GUI_MANUAL 租约。
  bool ros_motor_allowed(Clock::time_point now = Clock::now()) const {
    return ownership_.owner() == ControllerOwner::Nano || ownership_.manual_authorized(now);
  }

private:
// 借用外部对象（不负责它们的生命周期）；generation_/manual_authorized_ 缓存上一轮状态用于检测变化。
  ControlOwnership &ownership_;
  wire::MotorGate &gate_;
  std::uint64_t generation_;
  bool manual_authorized_ = false;
};

}  // namespace underwater
