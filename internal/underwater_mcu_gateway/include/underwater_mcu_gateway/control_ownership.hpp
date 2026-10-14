// ==============================================================================
// internal/underwater_mcu_gateway/include/underwater_mcu_gateway/control_ownership.hpp
// ==============================================================================
// 控制权仲裁：串口写权限在 Nano 与 GUI 之间互斥转让。
// GUI 的 session token 与其控制 TCP 连接绑定，心跳 (PING) 丢失超过 3s 自动交还 Nano。
// 所有方法都假定由节点主线程调用，内部不加锁。
// ==============================================================================
// 初学者导读：
// 1) 所属：internal/underwater_mcu_gateway（水下机器人 MCU 串口网关包）。整条数据链是
//    传感器 -> STM32 -> 串口 -> 本网关 -> ROS 2 devices/control -> 推进器；本文件只管"谁能写串口"。
// 2) 交互：由网关节点与 9002 管理 TCP 服务端（TcpControlServer / TcpSerialBridge）调用；
//    仲裁结果决定串口写权限归 Nano(机载 ROS) 还是 GUI(地面控制端)，本文件不直接收发 ROS 话题。
// 3) ROS 2 概念：这里没有 rclcpp 代码，但 ControllerOwner 会被节点用来判断
//    "现在是否允许通过 ROS 话题下发推进器指令"（见 motor_authority.hpp）。
// 4) C++ 概念：enum class 是强类型枚举（必须写 ControllerOwner::Nano）；
//    std::chrono::steady_clock 是单调时钟，测心跳不受系统改时间影响；
//    noexcept 表示承诺不抛异常；token 用随机数生成，防止别人冒用控制权。
#pragma once

#include <chrono>
#include <cstdint>
#include <string>

namespace underwater {

// 控制权持有者枚举
// enum class：强类型枚举，比普通 enum 更安全。
// 四个取值分别是：无人持有 / Nano 持有 / GUI 原始串口持有 / GUI_MANUAL 仅委托 ROS 输出。
enum class ControllerOwner : std::uint8_t {
  None = 0,
  Nano = 1,
  Gui = 2,
  GuiManual = 3
};

// 获取控制权名称字符串
const char *controller_name(ControllerOwner owner) noexcept;

// 所有权转移执行结果
// 一次控制权操作（申请/释放）的统一返回值：
// success=是否成功；owner=操作后的持有者；token=GUI 专用凭证（Nano 时为空）；message=给人看的说明。
struct OwnershipResult {
  bool success = false;
  ControllerOwner owner = ControllerOwner::None;
  std::string token;
  std::string message;
};

// Raw GUI 独占底层串口写权限；GUI_MANUAL 将受控的 ROS 输出委托给 Nano。
// 两种租约均与其管理 TCP 会话紧密绑定，互不可替代。
// 控制权仲裁核心类：任意时刻只有一个所有者，从而保证串口写权限互斥。
// 谁用：TcpControlServer(9002) 处理 CLAIM/TAKEOVER/RELEASE 时调用；
//       MotorAuthority 通过 generation()/manual_authorized() 判断是否放行 ROS 电机话题。
// 坑：类内不加锁，必须始终由同一线程（节点主线程）调用，否则需外部加锁。
class ControlOwnership {
public:
// Clock：统一使用单调时钟测量心跳时长，测试时可以注入 now 来模拟时间流逝。
  using Clock = std::chrono::steady_clock;

  explicit ControlOwnership(ControllerOwner initial = ControllerOwner::Nano);

  // takeover=false 时只在空闲 (None) 下才能获取，否则返回 success=false
// 申请 Nano 控制权：Nano 是默认所有者，也是所有超时/断连后回退的目标。
  OwnershipResult claim_nano(bool takeover);
  OwnershipResult release_nano();

  // session 为控制 TCP 的 fd；同一 session 重复声明是幂等的，并返回原 token。manual=true 表示 GUI_MANUAL 业务控制权
// 申请 GUI 控制权并生成 token；manual=true 时登记为 GUI_MANUAL（只委托 ROS 话题，不直接写串口）。
  OwnershipResult claim_gui(int session, bool takeover, bool manual = false);
  OwnershipResult release_gui(int session);

  // 校验 session + token 后续期心跳；token 不符返回 false
// GUI 心跳续期：必须同时满足"同一 TCP 会话 fd + 同一 token"才算数，否则权限已被抢走。
  bool heartbeat_gui(int session, const std::string &token);
  bool disconnect_gui(int session);

  // 心跳超过 3s 未更新则收回控制权；now 可注入以便测试
  bool expire_gui(Clock::time_point now = Clock::now());

  // 校验 GUI_MANUAL 租约并续期，成功返回非零代际号
// GUI_MANUAL 租约校验：成功返回非零代际号 generation，调用方据此判断权限是否已经换代。
  std::uint64_t validate_manual_lease(
      const std::string &token,
      Clock::time_point now = Clock::now());

  // 判定手动租约是否仍在有效期内
// 只读判断：GUI_MANUAL 租约此刻是否仍有效（心跳未超 3s 且 500ms 短凭证未过期）。
  bool manual_authorized(Clock::time_point now = Clock::now()) const noexcept;

  // 属性与状态查询
// 属性查询：generation 是控制权每次变更就自增的"代际号"，用来发现权限悄悄换了人。
  std::uint64_t generation() const noexcept { return generation_; }
  ControllerOwner owner() const noexcept { return owner_; }
  const std::string &gui_token() const noexcept { return token_; }
  int gui_session() const noexcept { return gui_session_; }

private:
// 生成随机 token（随机数与递增计数器混合），避免 GUI 凭证被猜出或重复。
  std::string new_token();
// 内部唯一入口：所有控制权变更都走这里，负责清/发 token、重置心跳并递增 generation_。
  void set_owner(ControllerOwner owner, int session = -1);

// 内部状态：owner_ 当前持有者；gui_session_ GUI 控制 TCP 的 fd；token_ GUI 凭证；
// last_heartbeat_ 最近心跳；manual_until_ GUI_MANUAL 短租约到期点；generation_ 代际号。
  ControllerOwner owner_ = ControllerOwner::Nano;
  int gui_session_ = -1;
  std::string token_;
  Clock::time_point last_heartbeat_{};
  Clock::time_point manual_until_{};
  std::uint64_t generation_ = 1;
  std::uint64_t token_sequence_ = 0;
};

}  // namespace underwater
