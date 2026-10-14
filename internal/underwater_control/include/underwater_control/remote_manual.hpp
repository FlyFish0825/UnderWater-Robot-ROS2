// ==============================================================================
// internal/underwater_control/include/underwater_control/remote_manual.hpp
// ==============================================================================
// 远程手动控制协议与会话状态机：
// 实现基于文本行的轻量化通信协议 (v2 规范)，包含 ENABLE、DISABLE、INPUT、AUTH 命令。
// 状态机脱离 ROS/网络传输层独立运作，统一在单线程执行上下文中保证时序严密性。
// ==============================================================================
#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace underwater::control::remote {

// 六自由度输入档位 [Fx, Fy, Fz, Mx, My, Mz]，取值范围通常为 -100 到 100
using Levels = std::array<int, 6>;

// 仅使用单调递增的本地稳态时钟 (毫秒)，严禁使用对端可能存在时钟漂移的墙上时钟
using Millis = std::int64_t;

// 协议命令种类
enum class Kind {
  Enable,   // 申请激活控制权 (ENABLE <seq> <lease_token>)
  Disable,  // 主动下线/停用控制权 (DISABLE <seq>)
  Input,    // 手动档位输入 (INPUT <seq> <session> <l0>..<l5>)
  Auth      // 租约鉴权校验 (AUTH <seq> <lease_token>)
};

// 会话生命周期状态机状态
enum class State {
  Disabled,  // 未就绪/已停用：安全静止状态
  Enabling,  // 正在握手使能：等待零位确认与硬件服务应答
  Enabled,   // 正常使能：允许接收并执行控制档位
  Stopping,  // 正在受控停机：下发零位刹车并等待各节点注销
  Fault      // 故障锁定状态：发生超时、冲突或硬件异常
};

// 解析后的协议命令结构
struct Command {
  Kind kind;
  std::uint64_t sequence = 0;       // 客户端单调递增的命令序号
  std::uint64_t session = 0;        // 握手分配的会话 ID
  Levels levels{};                  // 各轴控制档位
  std::string lease_token;          // 鉴权使用的租约 Token (32 字符十六进制)
};

// 命令执行返回结果结构
struct Result {
  std::uint64_t sequence = 0;       // 对应的客户端命令序号
  bool ok = false;                  // 命令是否被成功接受
  std::uint64_t session = 0;        // 会话代际/ID (未建立时为 0)
  std::string detail;               // 状态明细信息 (以十六进制编码传输)
};

// 底层各节点反馈的观测状态快照
struct Observation {
  bool gateway_ready = false;       // MCU 串口网关是否在线且允许下发控制
  bool allocation_fresh = false;    // 推力分配器状态数据是否在有效心跳内
  bool armed = false;               // 推力分配器当前是否处于使能(Armed)状态
  bool input_fresh = false;         // 控制输入是否在保活超时窗口内
  bool solver_ok = false;           // 推力分配 QP 求解器是否正常收敛
  bool rearm_required = true;       // 是否因保护触发而需要重新执行武装流程
  bool motor_active = false;        // 当前是否有推进器处于非零运转状态
  std::uint64_t connection = 0;     // 硬件串口物理连接代际标识
  std::uint64_t gateway_session = 0;// 网关当前绑定的会话 ID
  std::uint64_t allocation_session = 0; // 分配器当前绑定的会话 ID
  std::uint64_t allocation_update = 0;  // 分配器数据帧更新计数
  std::array<int, 8> rpm{};         // 8 路推进器当前实时转速
};

// 状态机指示 ROS 适配层执行的具体动作指令集
struct Actions {
  bool enable = false;   // 请求调用使能服务
  bool disable = false;  // 请求调用禁用服务
  bool zero = false;     // 请求下发全零位停机推力
  bool input = false;    // 请求下发实际输入档位
  bool release = false;  // 释放独占发布权
};

// 协议解析与辅助函数
std::optional<Command> parse(std::string_view line, std::string &error);
bool valid_levels(const Levels &levels);
std::string hex_detail(std::string_view detail);
std::string result_line(const Result &result);

// 文本行接收缓冲区：负责粘包分包处理并严格防御畸变字符及超长攻击
class LineBuffer {
public:
  bool append(std::string_view bytes, std::vector<std::string> &lines, std::string &error);
  void reset() { pending_.clear(); }

private:
  std::string pending_;
};

// 文本发送缓冲区：实现非阻塞发送排队与写入超时停滞检测
class OutputBuffer {
public:
  bool push(std::string_view bytes, Millis at);
  void consume(std::size_t size);
  const std::string &bytes() const { return bytes_; }
  bool stalled(Millis at) const { return !bytes_.empty() && (at - started_ >= 500); }
  void reset() { bytes_.clear(); }

private:
  std::string bytes_;
  Millis started_ = 0;
};

// 会话状态机：核心控制权调度单元，保证零位安全起步及单轴输入互锁
class Session {
public:
  void connect();
  void clear_completed_fault();
  void disconnect(Millis at);

  // 处理接收到的命令。注意：ENABLE 命令仅返回准入确认 (session=0)，
  // 最终的激活确认需等到底层完成零位握手并进入 Enabled 状态后才下发。
  Result handle(const Command &command, Millis at, bool ready);

  // 观测底层状态变更并驱动内部状态迁移
  void observe(const Observation &observation, Millis at);

  // 周期时钟节拍：检查握手超时(2500ms)、输入超时(500ms)以及停机重试
  void tick(Millis at);

  // 外部异步服务的执行结果回调通知
  void enable_result(bool ok, const std::string &detail, Millis at);
  void disable_result(bool ok, Millis at);
  void disable_result(bool ok, Millis at, std::uint64_t generation);

  // 触发受控停机或故障紧急切断
  void stop(const std::string &detail, bool fault, Millis at);

  // 提取待执行动作集（每次调用清空内部累积指令）
  Actions take_actions();

  // 状态与属性查询接口
  State state() const { return state_; }
  bool owns_publisher() const { return owned_; }
  bool enable_pending() const { return enable_pending_; }
  std::uint64_t disable_generation() const { return disable_generation_; }
  std::uint64_t session() const { return session_; }
  std::uint64_t last_input_sequence() const { return last_input_sequence_; }
  const Levels &levels() const { return levels_; }
  const std::string &detail() const { return detail_; }

private:
  void finish_stop();

  State state_ = State::Disabled;
  bool connected_ = false;
  bool owned_ = false;
  bool enable_pending_ = false;
  bool disable_confirmed_ = false;
  bool fault_ = false;

  std::uint64_t sequence_ = 0;
  std::uint64_t session_ = 0;
  std::uint64_t baseline_session_ = 0;
  std::uint64_t connection_ = 0;
  std::uint64_t last_input_sequence_ = 0;
  std::uint64_t input_observation_ = 0;
  std::uint64_t disable_generation_ = 0;

  Millis started_ = 0;
  Millis input_at_ = 0;
  Millis disable_at_ = 0;

  Levels levels_{};
  Observation observation_{};
  Actions actions_{};
  std::string detail_ = "disabled";
};

// 生成向客户端周期广播的 STATE 状态行字符串
std::string state_line(const Session &session, std::uint64_t counter, const Observation &observation);

}  // namespace underwater::control::remote
