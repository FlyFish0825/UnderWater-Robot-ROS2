// ==============================================================================
// internal/underwater_control/include/underwater_control/remote_manual.hpp
// ==============================================================================
// 远程手动控制协议与会话状态机：
// 实现基于文本行的轻量化通信协议 (v2 规范)，包含 ENABLE、DISABLE、INPUT、AUTH 命令。
// 状态机脱离 ROS/网络传输层独立运作，统一在单线程执行上下文中保证时序严密性。
// ==============================================================================
// ------------------------------------------------------------------------------
// 【初学者导读】
// 所属包/目录：underwater_control 的 include 公共头文件，声明“远程手动控制协议与会话状态机”。
// 角色：纯逻辑层，不依赖 ROS、也不直接收发网络；它只做两件事：(1) 把一行文本命令解析成
//      结构体；(2) 用状态机决定什么时候允许开电机、什么时候必须停下来。
// 与谁交互：上层由 remote_manual_node.cpp 调用（节点把 ROS 话题/服务与 TCP 字节流转成这里的
//          Command/Observation）；底层硬件状态也由节点填进 Observation 喂回来。
// ROS 2 概念：本文件本身不出现 ROS API。但 Observation 里的 gateway_ready、allocation_fresh
//            等字段来自节点订阅的 gateway/status 与 control/allocation 话题，状态机据此做看门狗判断。
// C++ 概念速览：
//   namespace 命名空间：给名字加前缀避免重名，这里展开是 underwater::control::remote。
//   enum class 强类型枚举：Kind::Enable 不会隐式变成整数，比普通 enum 更安全。
//   std::optional<T>：可能“有值”也可能“没有值”，用来表示解析失败。
//   std::array<int,6>：长度固定的数组，比 C 数组更安全（带 .size()）。
//   struct：默认成员公开的类，这里只用来装数据。
// ------------------------------------------------------------------------------
#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace underwater::control::remote {

// C++ 小知识：std::array<int,6> 是固定长度数组，用 [] 下标访问，元素类型统一。
// Levels 表示 6 个自由度的“操作档位”，不是直接推力大小。
// 六自由度输入档位 [Fx, Fy, Fz, Mx, My, Mz]，取值范围通常为 -100 到 100
using Levels = std::array<int, 6>;

// Millis 是“开机以来经过的毫秒数”。使用单调时钟可避免系统对时/时区调整让时间倒流，
// 这类看门狗超时判断只有基于单调时钟才可靠。
// 仅使用单调递增的本地稳态时钟 (毫秒)，严禁使用对端可能存在时钟漂移的墙上时钟
using Millis = std::int64_t;

// C++ 小知识：enum class（强类型枚举）里的取值必须写成 Kind::Enable，
// 不会隐式转换成整数，能避免和别的常量混淆。
// 协议命令种类
enum class Kind {
  Enable,   // 申请激活控制权 (ENABLE <seq> <lease_token>)
  Disable,  // 主动下线/停用控制权 (DISABLE <seq>)
  Input,    // 手动档位输入 (INPUT <seq> <session> <l0>..<l5>)
  Auth      // 租约鉴权校验 (AUTH <seq> <lease_token>)
};

// State 描述“会话现在走到哪一步”，所有控制权限判断都围绕它展开。
// 会话生命周期状态机状态
enum class State {
  Disabled,  // 未就绪/已停用：安全静止状态
  Enabling,  // 正在握手使能：等待零位确认与硬件服务应答
  Enabled,   // 正常使能：允许接收并执行控制档位
  Stopping,  // 正在受控停机：下发零位刹车并等待各节点注销
  Fault      // 故障锁定状态：发生超时、冲突或硬件异常
};

// Command 是“一行命令解析后”的结果，会交给 Session::handle() 去执行。
// 解析后的协议命令结构
struct Command {
  Kind kind;
  std::uint64_t sequence = 0;       // 客户端单调递增的命令序号
  std::uint64_t session = 0;        // 握手分配的会话 ID
  Levels levels{};                  // 各轴控制档位
  std::string lease_token;          // 鉴权使用的租约 Token (32 字符十六进制)
};

// Result 是要回给客户端的处理结果（由 result_line 序列化成文本行）。
// 命令执行返回结果结构
struct Result {
  std::uint64_t sequence = 0;       // 对应的客户端命令序号
  bool ok = false;                  // 命令是否被成功接受
  std::uint64_t session = 0;        // 会话代际/ID (未建立时为 0)
  std::string detail;               // 状态明细信息 (以十六进制编码传输)
};

// Observation 是“底层硬件这一刻怎么样”的快照，由 ROS 节点填好后喂给状态机。
// 这里的每个 bool 都对应一路心跳/健康判断，任一关键项超时都会触发安全停机。
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

// Actions 是“状态机想请 ROS 适配层帮忙做的事”，例如调用使能服务、发布全零推力。
// 状态机只下指令、不做真正的网络/ROS 调用，因此可以脱离 ROS 单独测试。
// 状态机指示 ROS 适配层执行的具体动作指令集
struct Actions {
  bool enable = false;   // 请求调用使能服务
  bool disable = false;  // 请求调用禁用服务
  bool zero = false;     // 请求下发全零位停机推力
  bool input = false;    // 请求下发实际输入档位
  bool release = false;  // 释放独占发布权
};

// 以下是纯工具函数：解析文本行、校验档位、把明细转成十六进制、拼装 RESULT 行。
// 协议解析与辅助函数
std::optional<Command> parse(std::string_view line, std::string &error);
bool valid_levels(const Levels &levels);
std::string hex_detail(std::string_view detail);
std::string result_line(const Result &result);

// ROS 2 小知识：TCP 是字节流，没有消息边界，可能“一次读到半行”或“一次读到两行半”，
// 这叫粘包/分包。LineBuffer 专门缓存数据并切出完整的行。
// 文本行接收缓冲区：负责粘包分包处理并严格防御畸变字符及超长攻击
class LineBuffer {
public:
  bool append(std::string_view bytes, std::vector<std::string> &lines, std::string &error);
  void reset() { pending_.clear(); }

private:
  std::string pending_;
};

// OutputBuffer 暂存还没发出去的字节；若长时间发不掉（链路阻塞）就判定停滞。
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

// C++ 小知识：类把“数据”和“操作数据的方法”打包在一起；private: 之后的内容只有本类能访问，
// 外部只能通过 public: 接口使用，这叫封装。
// Session 是核心状态机：它记住当前状态，并保证任何异常都先归零再断开。
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

// tick 由节点的定时器周期调用，专门处理“超时”类安全判断。
  // 周期时钟节拍：检查握手超时(2500ms)、输入超时(500ms)以及停机重试
  void tick(Millis at);

  // 外部异步服务的执行结果回调通知
  void enable_result(bool ok, const std::string &detail, Millis at);
  void disable_result(bool ok, Millis at);
  void disable_result(bool ok, Millis at, std::uint64_t generation);

  // 触发受控停机或故障紧急切断
  void stop(const std::string &detail, bool fault, Millis at);

// 状态机把动作攒在内部，这里“取走并清空”，避免同一个动作被重复执行。
  // 提取待执行动作集（每次调用清空内部累积指令）
  Actions take_actions();

  // 状态与属性查询接口
// 下面这些是只读查询接口，方便节点和测试了解状态机内部情况。
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

// state_line 生成周期性广播给客户端的 STATE 行：状态、档位、转速、明细。
// 生成向客户端周期广播的 STATE 状态行字符串
std::string state_line(const Session &session, std::uint64_t counter, const Observation &observation);

}  // namespace underwater::control::remote
