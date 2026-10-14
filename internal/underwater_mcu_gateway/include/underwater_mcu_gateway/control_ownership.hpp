// ==============================================================================
// internal/underwater_mcu_gateway/include/underwater_mcu_gateway/control_ownership.hpp
// ==============================================================================
// 控制权仲裁：串口写权限在 Nano 与 GUI 之间互斥转让。
// GUI 的 session token 与其控制 TCP 连接绑定，心跳 (PING) 丢失超过 3s 自动交还 Nano。
// 所有方法都假定由节点主线程调用，内部不加锁。
// ==============================================================================
#pragma once

#include <chrono>
#include <cstdint>
#include <string>

namespace underwater {

// 控制权持有者枚举
enum class ControllerOwner : std::uint8_t {
  None = 0,
  Nano = 1,
  Gui = 2,
  GuiManual = 3
};

// 获取控制权名称字符串
const char *controller_name(ControllerOwner owner) noexcept;

// 所有权转移执行结果
struct OwnershipResult {
  bool success = false;
  ControllerOwner owner = ControllerOwner::None;
  std::string token;
  std::string message;
};

// Raw GUI 独占底层串口写权限；GUI_MANUAL 将受控的 ROS 输出委托给 Nano。
// 两种租约均与其管理 TCP 会话紧密绑定，互不可替代。
class ControlOwnership {
public:
  using Clock = std::chrono::steady_clock;

  explicit ControlOwnership(ControllerOwner initial = ControllerOwner::Nano);

  // takeover=false 时只在空闲 (None) 下才能获取，否则返回 success=false
  OwnershipResult claim_nano(bool takeover);
  OwnershipResult release_nano();

  // session 为控制 TCP 的 fd；同一 session 重复声明是幂等的，并返回原 token。manual=true 表示 GUI_MANUAL 业务控制权
  OwnershipResult claim_gui(int session, bool takeover, bool manual = false);
  OwnershipResult release_gui(int session);

  // 校验 session + token 后续期心跳；token 不符返回 false
  bool heartbeat_gui(int session, const std::string &token);
  bool disconnect_gui(int session);

  // 心跳超过 3s 未更新则收回控制权；now 可注入以便测试
  bool expire_gui(Clock::time_point now = Clock::now());

  // 校验 GUI_MANUAL 租约并续期，成功返回非零代际号
  std::uint64_t validate_manual_lease(
      const std::string &token,
      Clock::time_point now = Clock::now());

  // 判定手动租约是否仍在有效期内
  bool manual_authorized(Clock::time_point now = Clock::now()) const noexcept;

  // 属性与状态查询
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
