// ==============================================================================
// internal/underwater_control/src/remote/remote_manual_node.cpp
// ==============================================================================
// 远程手动控制服务节点：
// - 建立 TCP 9003 监听端口接入远程业务控制端
// - 依托单线程 Executor 管理 TCP 套接字事件、内部状态机以及 ROS 服务/话题调用
// - 校验并维护 GUI 租约 (ValidateManualLease) 的生命周期与 400ms 心跳超时
// - 向 control/manual_request 独占发布 WrenchStamped 档位指令
// ==============================================================================
// ------------------------------------------------------------------------------
// 【初学者导读】
// 所属包/目录：underwater_control 的 remote 源文件，是最重要的 ROS 2 适配节点。
// 角色：把“TCP 文本协议”“会话状态机”“ROS 2 话题/服务”三者粘在一起。
// 与谁交互：
//   订阅 gateway/status 与 control/allocation（底层网关、推力分配器状态）。
//   向 control/manual_request 独占发布 WrenchStamped 档位指令。
//   调用 control/enable（std_srvs/SetBool）启停底层，调用 gateway/validate_manual_lease 校验租约。
//   同时监听 TCP 9003 端口与远程控制端通信。
// ROS 2 概念速览：
//   Node（节点）：一个能收发话题、调用服务的独立小程序；本类继承 rclcpp::Node。
//   Publisher/Subscription：发布者发消息，订阅者收到后执行回调。
//   Service/Client：服务是“一问一答”的远程调用；async_send_request 异步发送，结果进回调。
//   Parameter（参数）：节点的可配置项，可由 launch 文件设置。
//   QoS：消息的传输质量策略；QoS(1) 表示队列深度为 1（keep_last），只排队 1 条消息。
//   Executor/spin：spin 让执行器不断取出并运行回调；本节点用单线程 + 定时器保证时序简单。
// C++ 概念速览：
//   unique_ptr/shared_ptr：智能指针，离开作用域自动释放；shared_ptr 可共享所有权。
//   lambda：匿名函数，[] 捕获外部变量，(参数){函数体}，常作为回调传入。
//   override：写在重写的虚函数后，编译期检查是否确实覆盖了基类函数。
//   静态成员函数 static：不依赖对象实例即可调用。
// ------------------------------------------------------------------------------
#include "underwater_control/remote_manual.hpp"
#include "underwater_control/manual_authority.hpp"
#include "underwater_control/remote_tcp.hpp"

#include <chrono>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <underwater_interfaces/msg/allocation_status.hpp>
#include <underwater_interfaces/msg/gateway_status.hpp>
#include <underwater_interfaces/srv/validate_manual_lease.hpp>

namespace rm = underwater::control::remote;
namespace ui = underwater_interfaces::msg;
using Clock = std::chrono::steady_clock;
using Enable = std_srvs::srv::SetBool;
using ValidateLease = underwater_interfaces::srv::ValidateManualLease;
using namespace std::chrono_literals;

// RemoteManualNode 是远程手动控制的 ROS 2 节点；final 表示不允许再被继承。
class RemoteManualNode final : public rclcpp::Node {
public:
// 构造函数里完成参数声明、TCP 监听、客户端/订阅/定时器的创建。
  RemoteManualNode() : Node("remote_manual") {
// Parameter（参数）是节点的可配置项；这里声明为 read_only，运行期不允许改。
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;

    const auto address = declare_parameter<std::string>("bind_address", "0.0.0.0", descriptor);
    const int port = declare_parameter<int>("port", 9003, descriptor);
    if (port < 1 || port > 65535) {
      throw std::invalid_argument("remote manual port must be 1..65535");
    }

// std::make_unique 创建“独占所有权”的智能指针，对象随成员析构自动释放。
    // 初始化 TCP 监听套接字
    server_ = std::make_unique<rm::TcpServer>(address, static_cast<std::uint16_t>(port));

// create_client<服务类型>(服务名)：创建 ROS 2 服务客户端，可异步发请求。
    // 创建 ROS 客户端：使能底层推力分配与校验网关租约
    enable_ = create_client<Enable>("control/enable");
    lease_ = create_client<ValidateLease>("gateway/validate_manual_lease");

// create_subscription<消息类型>(话题名, QoS, 回调)：每来一条消息执行一次回调（这里是 lambda）。
// QoS(1) 表示队列深度为 1，只关心最新状态。
    // 订阅串口网关状态与推力分配器状态
    gateway_sub_ = create_subscription<ui::GatewayStatus>(
        "gateway/status",
        rclcpp::QoS(1),
        [this](ui::GatewayStatus::ConstSharedPtr s) {
          gateway_ = *s;
          gateway_at_ = millis();
        });

    allocation_sub_ = create_subscription<ui::AllocationStatus>(
        "control/allocation",
        rclcpp::QoS(1),
        [this](ui::AllocationStatus::ConstSharedPtr s) {
          allocation_ = *s;
          allocation_at_ = millis();
          ++allocation_update_;
        });

// create_wall_timer(周期, 回调)：按真实时间周期性触发，这里 20ms 一次（约 50Hz）。
    // 20ms 主循环定时器 (50Hz 轮询套接字与心跳)
    timer_ = create_wall_timer(20ms, [this] { tick(); });

    RCLCPP_INFO(
        get_logger(),
        "Business manual TCP %s:%d; DISABLED, no serial takeover, no input publisher until ENABLE",
        address.c_str(),
        port);
  }

// 析构时尽力安全收尾：先发一次全零，再调用底层停用服务（若 ROS 还在运行）。
  ~RemoteManualNode() override {
    // 尽最大努力安全关闭：撤销独占发布并向底层下发停用服务请求
    try {
      if (session_.owns_publisher() && rclcpp::ok()) {
        publish({});
        if (enable_->service_is_ready()) {
          auto req = std::make_shared<Enable::Request>();
          req->data = false;
          enable_->async_send_request(req);
        }
      }
    } catch (...) {}
  }

private:
  // 获取单调时钟毫秒数
// 从单调时钟取毫秒，用于所有超时判断。
  static rm::Millis millis() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        Clock::now().time_since_epoch()).count();
  }

  // 校验 ROS 消息时间戳新鲜度：允许未来 100ms 内的时间容差，以及不超过 bound_ms 的过去延时
// 消息时间戳新鲜度检查：ROS 时间可能因仿真/对时抖动，允许 100ms 未来容差，
// 过去延时不得超过 bound_ms。
  bool stamp_fresh(const builtin_interfaces::msg::Time &stamp, std::int64_t bound_ms) {
    const auto age = (now() - rclcpp::Time(stamp, get_clock()->get_clock_type())).nanoseconds();
    return age >= -100000000LL && age < bound_ms * 1000000LL;
  }

  // 统计当前计算图中全部手动档位与 wrench 请求话题的发布者数量总和
// 统计计算图上可能“抢方向盘”的发布者数量；多于 1 个说明还有别的源在发控制。
  std::size_t sources() const {
    return count_publishers("control/manual_request") + count_publishers("control/wrench_request");
  }

  // 生成当前时刻系统各模块的健康与就绪观测快照
// 汇总网关与分配器的最新状态，形成状态机需要的 Observation 快照。
  rm::Observation observation(rm::Millis at) {
    rm::Observation o;
    if (gateway_) {
      const auto &g = *gateway_;
// 网关就绪必须同时满足：本地收包新鲜、消息时间戳新鲜、串口连接、未在稳定期、
// TX 允许、控制权属于 GUI_MANUAL、租约有效、心跳年龄正常。
      o.gateway_ready = (at - gateway_at_ < 2500) &&
                        stamp_fresh(g.header.stamp, 2500) &&
                        g.connected &&
                        !g.settling &&
                        g.tx_enabled &&
                        (g.control_owner == "GUI_MANUAL") &&
                        proof_.valid(at) &&
                        (g.heartbeat_age_ms >= 0) &&
                        (g.heartbeat_age_ms < 2500);
      o.connection = g.connection_id;
      o.gateway_session = g.motor_session_id;
      o.rearm_required = g.motor_rearm_required;
      o.motor_active = g.motor_command_active;
    }

    if (allocation_) {
      const auto &a = *allocation_;
      o.allocation_fresh = (at - allocation_at_ < 500) && stamp_fresh(a.header.stamp, 500);
      o.allocation_session = a.session_id;
      o.armed = a.armed;
      o.allocation_update = allocation_update_;
      o.input_fresh = o.allocation_fresh && a.input_fresh;
      o.solver_ok = a.solver_ok;
      for (std::size_t i = 0; i < 8; ++i) {
        o.rpm[i] = a.proposed_motor_rpm[i];
      }
    }

    // 若检测到计算图存在冲突发布者，立即置位不可用
// 独占发布后若发现还有其他发布者，立即把网关标记为不可用，触发安全停机。
    if (session_.owns_publisher() && sources() > 1) {
      o.gateway_ready = false;
    }
    return o;
  }

  // 向 TCP 客户端发送文本行，若发送缓冲区溢出则触发紧急熔断
// 通过 TCP 发送文本行；一旦缓冲区溢出，说明链路异常，立即熔断停机。
  void queue(const std::string &line, rm::Millis at) {
    if (!server_->connected()) {
      return;
    }
    if (!server_->queue(line, at)) {
      cancel_authority();
      server_->drop();
      session_.stop("TCP output exceeded bounded queue", true, at);
      session_.disconnect(at);
    }
  }

  // 向 control/manual_request 话题发布控制档位 (WrenchStamped 格式)
// 把 6 个档位组成 WrenchStamped（力 + 力矩）发布出去，具体 8 路转速由分配器换算。
  void publish(const rm::Levels &v) {
    if (!input_) {
      return;
    }
    auto stamp = now();
// 保证发布的时间戳严格递增；检测到时钟明显倒退就判故障。
    // 保证时间戳严格单调递增，防御时钟倒退
    if (last_stamp_ && stamp <= *last_stamp_) {
      if ((*last_stamp_ - stamp).nanoseconds() > 100000000LL) {
        session_.stop("ROS clock moved backwards", true, millis());
        return;
      }
      stamp = *last_stamp_ + rclcpp::Duration::from_nanoseconds(1);
    }
    last_stamp_ = stamp;

    geometry_msgs::msg::WrenchStamped w;
    w.header.stamp = stamp;
    w.header.frame_id = "thruster_center";
    w.wrench.force.x = v[0];
    w.wrench.force.y = v[1];
    w.wrench.force.z = v[2];
    w.wrench.torque.x = v[3];
    w.wrench.torque.y = v[4];
    w.wrench.torque.z = v[5];
    input_->publish(w);
  }

  // 执行状态机生成的外部动作指令
// 状态机把“想做的事”攒在 Actions 里，这里逐项翻译成真正的 ROS/TCP 调用。
  void actions(rm::Millis at) {
    const auto a = session_.take_actions();
    if (a.enable) {
// 使能动作：此时才创建 control/manual_request 发布者，取得“独占发布”身份。
      input_ = create_publisher<geometry_msgs::msg::WrenchStamped>("control/manual_request", rclcpp::QoS(1));
      publisher_at_ = at;
      waiting_enable_ = true;
      last_stamp_.reset();
    }
// zero 动作：发布一次全零，确保停机时先归零。
    if (a.zero) {
      publish({});
    }
    if (a.input && session_.state() == rm::State::Enabled) {
      publish(session_.levels());
    }
    if (a.disable) {
      wanted_disable_ = true;
    }
    if (a.release) {
      input_.reset();
      waiting_enable_ = false;
      wanted_disable_ = false;
      last_stamp_.reset();
      cancel_authority();
    }
  }

  // 异步发起底层使能服务调用
// 异步请求底层使能服务，并注册回调接收结果。
  void request_enable(rm::Millis at) {
    if (!waiting_enable_) {
      return;
    }
    if (session_.state() != rm::State::Enabling) {
      waiting_enable_ = false;
      session_.enable_result(false, "enable cancelled before service request", at);
      return;
    }

    // 给予 DDS 发现期 1000ms 宽限时间，确保计算图拓扑收敛
// 新发布者需要时间让 DDS 发现，先等 1000ms 让计算图收敛后再发服务请求。
    if (at - publisher_at_ < 1000) {
      return;
    }
    waiting_enable_ = false;

    const auto o = observation(at);
// 发送前再次确认前置条件：网关就绪、分配器新鲜、未武装、恰好一个发布者、服务可用。
    if (!o.gateway_ready || !o.allocation_fresh || o.armed || sources() != 1 || !enable_->service_is_ready()) {
      session_.enable_result(false, "enable prerequisite/source/service unavailable", at);
      return;
    }

    auto req = std::make_shared<Enable::Request>();
    req->data = true;
    try {
// async_send_request(请求, 回调)：非阻塞发送，结果到达时执行 lambda（future.get() 取响应）。
      enable_->async_send_request(
          req,
          [this](rclcpp::Client<Enable>::SharedFuture future) {
            const auto at = millis();
            try {
              const auto response = future.get();
              session_.enable_result(response->success, response->message, at);
            } catch (const std::exception &e) {
              session_.stop(std::string("uncertain enable response: ") + e.what(), true, at);
            }
            actions(at);
          });
    } catch (const std::exception &e) {
      session_.stop(std::string("uncertain enable submission: ") + e.what(), true, at);
    }
  }

  // 异步发起底层停机服务调用
// 异步请求停用服务；带 generation 用于忽略过期的旧回包。
  void request_disable(rm::Millis at) {
    if (!wanted_disable_ || disable_pending_ || !session_.owns_publisher() || !enable_->service_is_ready()) {
      return;
    }
    wanted_disable_ = false;
    disable_pending_ = true;

    auto req = std::make_shared<Enable::Request>();
    req->data = false;
    const auto generation = session_.disable_generation();

    try {
      enable_->async_send_request(
          req,
          [this, generation](rclcpp::Client<Enable>::SharedFuture future) {
            disable_pending_ = false;
            bool ok = false;
            try {
              ok = future.get()->success;
            } catch (const std::exception &) {}
            session_.disable_result(ok, millis(), generation);
            actions(millis());
          });
    } catch (const std::exception &) {
      disable_pending_ = false;
      session_.disable_result(false, at, generation);
    }
  }

  // 取消当前租约授权状态
// 取消租约：让所有在途校验请求失效；preserve_live=false 时才清掉本地证明与 token。
  void cancel_authority(bool preserve_live = false) {
    ++lease_request_generation_;
    if (lease_request_id_) {
      lease_->remove_pending_request(*lease_request_id_);
    }
    lease_request_id_.reset();
    authenticating_.reset();
    proof_.cancel_request();

    if (!preserve_live) {
      proof_.clear();
      lease_token_.clear();
    }
    server_->reserve(preserve_live || session_.owns_publisher());
  }

  // 处理租约鉴权失败，关闭客户端连接并安全停机
// 鉴权失败时的统一处理：若有在途鉴权则回一条失败 RESULT，然后撤销租约、停机并断开连接。
  void authority_failed(const std::string &reason, rm::Millis at) {
    if (authenticating_) {
      queue(rm::result_line({authenticating_->sequence, false, 0, reason}), at);
    }
    cancel_authority();
    session_.stop(reason, true, at);
    actions(at);
    server_->drop();
    session_.disconnect(at);
    actions(at);
  }

  // 向网关发起异步租约校验
// 向网关异步校验租约 token；成功后才允许 ENABLE/INPUT。
  void validate_authority(rm::Millis at) {
    if (proof_.pending() || lease_token_.empty()) {
      return;
    }
    if (!lease_->service_is_ready()) {
      authority_failed("manual lease validation service unavailable", at);
      return;
    }

    const auto request = proof_.begin(at);
// 记录本次请求的代际和 TCP 连接代际，回调里用来识别过期或中途换连接的情况。
    const auto local_generation = ++lease_request_generation_;
    const auto tcp_generation = server_->generation();
    lease_at_ = at;

    auto req = std::make_shared<ValidateLease::Request>();
    req->token = lease_token_;

    try {
// 回调捕获了 request/local_generation/tcp_generation，用于校验回包是否仍然有效。
      const auto pending = lease_->async_send_request(
          req,
          [this, request, local_generation, tcp_generation](rclcpp::Client<ValidateLease>::SharedFuture future) {
            if (local_generation != lease_request_generation_) {
              return;
            }
            lease_request_id_.reset();
            const auto at = millis();

            if (!server_->connected() || server_->generation() != tcp_generation) {
              authority_failed("manual TCP connection changed during authorization", at);
              return;
            }

            bool accepted = false;
            try {
              const auto response = future.get();
              accepted = proof_.complete(request, response->valid, response->generation, at);
            } catch (const std::exception &) {}

            if (!accepted) {
              authority_failed("GUI manual lease invalid, changed or expired", at);
              return;
            }

            server_->reserve(true);
            session_.clear_completed_fault();

            if (authenticating_) {
              const auto command = *authenticating_;
              authenticating_.reset();
              if (command.kind == rm::Kind::Auth) {
                queue(rm::result_line({command.sequence, true, 0, "manual authority verified; motors remain unchanged"}), at);
                return;
              }
              session_.observe(observation(at), at);
              admit(command, at);
              if (!session_.owns_publisher()) {
                cancel_authority();
              }
            }
          });
      lease_request_id_ = pending.request_id;
    } catch (const std::exception &) {
      authority_failed("manual lease validation submission failed", at);
    }
  }

  // 准入命令并交由状态机处理
// ready 表示此刻允许使能：服务就绪、没有其他发布者、租约有效。
  void admit(const rm::Command &c, rm::Millis at) {
    const bool ready = enable_->service_is_ready() && (sources() == 0) && proof_.valid(at);
    const auto result = session_.handle(c, at, ready);

    if (c.kind == rm::Kind::Enable && result.ok) {
      pending_result_ = c.sequence;
      pending_generation_ = server_->generation();
    } else if (c.kind != rm::Kind::Input || !result.ok) {
      queue(rm::result_line(result), at);
    }
    actions(at);
  }

  // 处理接收到的文本指令行
// 把一行文本解析成命令；解析失败按协议错误处理：停机并断开连接。
  void command(const std::string &line, rm::Millis at) {
    std::string error;
    const auto c = rm::parse(line, error);
    if (!c) {
      cancel_authority();
      session_.stop(error, true, at);
      server_->drop();
      session_.disconnect(at);
      actions(at);
      return;
    }

// 命令序号必须严格递增，重复/乱序一律判为鉴权失败。
    if (c->sequence <= received_sequence_) {
      authority_failed("duplicate or out-of-order command sequence", at);
      return;
    }
    received_sequence_ = c->sequence;

// ENABLE/AUTH 需要先过租约校验；已有有效租约时 AUTH 可直接确认。
    if (c->kind == rm::Kind::Enable || c->kind == rm::Kind::Auth) {
      if (c->kind == rm::Kind::Auth && c->lease_token == lease_token_ && proof_.valid(at)) {
        queue(rm::result_line({c->sequence, true, session_.session(), "manual authority remains valid"}), at);
        return;
      }
      if (session_.owns_publisher()) {
        queue(rm::result_line({c->sequence, false, session_.session(), "previous enable/session still pending"}), at);
        return;
      }

      const bool preserve = (c->kind == rm::Kind::Enable && c->lease_token == lease_token_ && proof_.valid(at));
      cancel_authority(preserve);
      authenticating_ = *c;
      lease_token_ = c->lease_token;
      validate_authority(at);
      return;
    }

    if (c->kind == rm::Kind::Disable) {
      cancel_authority();
    } else if (authenticating_ || !proof_.valid(at)) {
      authority_failed("input has no confirmed live GUI lease", at);
      server_->drop();
      session_.disconnect(at);
      actions(at);
      return;
    }

    admit(*c, at);
  }

  // 20ms 主循环定时任务
// 主循环：每 20ms 执行一次，处理租约超时、状态机节拍、TCP 事件、服务进度与状态广播。
  void tick() {
    const auto at = millis();

    // 租约请求超时巡查
// 租约校验请求超时巡查。
    if (proof_.expired_request(at)) {
      authority_failed("manual lease validation timed out", at);
    }
    if (!lease_token_.empty() && !authenticating_) {
      if (!proof_.valid(at)) {
        authority_failed("manual lease proof expired", at);
      } else if (at - lease_at_ >= 100) {
        validate_authority(at);
      }
    }

    // 状态机周期时钟节拍
    session_.observe(observation(at), at);
    session_.tick(at);
    actions(at);

    // 轮询 TCP 事件与协议命令
// 轮询 TCP 事件：接入、断开、收到整行命令。
    server_->poll(at, [this, at](rm::TcpServer::Event event, const std::string &value) {
      if (event == rm::TcpServer::Event::Connected) {
        cancel_authority();
        session_.connect();
        counter_ = 0;
        received_sequence_ = 0;
        queue(rm::state_line(session_, ++counter_, observation(at)), at);
      } else if (event == rm::TcpServer::Event::Disconnected) {
        cancel_authority();
        session_.stop(value, true, at);
        session_.disconnect(at);
        actions(at);
      } else {
        command(value, at);
      }
    });

    // 处理使能与禁用申请
    request_enable(at);
    actions(at);
    request_disable(at);

    const auto o = observation(at);
    session_.observe(o, at);
    actions(at);

    // 回复握手就绪后的终态 RESULT
// 握手结束后补发一条最终 RESULT，因为 ENABLE 的确认要等状态机离开 Enabling。
    if (pending_result_ && session_.state() != rm::State::Enabling) {
      if (server_->connected() && server_->generation() == pending_generation_) {
        const bool ok = (session_.state() == rm::State::Enabled);
        queue(rm::result_line({*pending_result_, ok, session_.session(), session_.detail()}), at);
      }
      pending_result_.reset();
    }

    // 100ms 周期广播 STATE 状态行
// 每 100ms 广播一次 STATE，让客户端同步显示状态、档位与转速。
    if (at - state_at_ >= 100) {
      state_at_ = at;
      queue(rm::state_line(session_, ++counter_, o), at);
    }
  }

  // 状态机与租约凭证
// ---------- 成员变量区域 ----------
  rm::Session session_;
  rm::LeaseProof proof_;
  std::optional<rm::Command> authenticating_;
  std::string lease_token_;
  std::optional<std::int64_t> lease_request_id_;
  std::uint64_t lease_request_generation_ = 0;
  rm::Millis lease_at_ = 0;

  // 网络与 ROS 通信组件
  std::unique_ptr<rm::TcpServer> server_;
  std::optional<ui::GatewayStatus> gateway_;
  std::optional<ui::AllocationStatus> allocation_;
  std::optional<rclcpp::Time> last_stamp_;

  // 序号、统计计数与状态时标
  std::optional<std::uint64_t> pending_result_;
  std::uint64_t pending_generation_ = 0;
  std::uint64_t counter_ = 0;
  std::uint64_t allocation_update_ = 0;
  std::uint64_t received_sequence_ = 0;

  rm::Millis gateway_at_ = 0;
  rm::Millis allocation_at_ = 0;
  rm::Millis publisher_at_ = 0;
  rm::Millis state_at_ = 0;

  // 异步操作握手标记
  bool waiting_enable_ = false;
  bool wanted_disable_ = false;
  bool disable_pending_ = false;

  // ROS 发布者、服务客户端与定时器
// ROS 发布者、服务客户端与订阅句柄；SharedPtr 是共享所有权的智能指针。
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr input_;
  rclcpp::Client<Enable>::SharedPtr enable_;
  rclcpp::Client<ValidateLease>::SharedPtr lease_;
  rclcpp::Subscription<ui::GatewayStatus>::SharedPtr gateway_sub_;
  rclcpp::Subscription<ui::AllocationStatus>::SharedPtr allocation_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

// main：初始化 ROS 2，创建节点并 spin（让执行器不断处理回调），退出时 shutdown。
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<RemoteManualNode>());
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("remote_manual"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
