// ==============================================================================
// internal/underwater_mcu_gateway/src/nodes/mcu_gateway_node.cpp
// ==============================================================================
// MCU 串口网关核心节点：
// - 管理下位机硬件串口 (115200 8N1 raw 独占模式) 的连接生命周期与自动重连；
// - 提供 9001 端口 (原始串口字节透传) 与 9002 端口 (所有权仲裁控制通道)；
// 勘误：原始串口字节透传的实际端口由 tcp_port 参数决定，bringup 默认 9000（9001 是相机）。
// - 解码 MCU 上行 0x55(CAN), 0x58(心跳), 0x59(通用), 0x5b(传感器) 报文并发布至 ROS；
// - 严格执行电机下发安全门禁 (MotorGate & MotorAuthority)，心跳超时自动停机；
// - 承载深度计水面零点自动去皮标定服务 (sensors/depth/zero)。
// ==============================================================================
// ------------------------------------------------------------------------------
// 【初学者导读】
// 1. 所属与角色：本文件属于 internal/underwater_mcu_gateway 包，是整个软件栈的
//    “串口网关”核心节点（ROS 2 节点名 mcu_gateway）。数据链路大致为：
//    传感器/STM32 -> 串口 -> 本网关解析成 ROS 话题 -> control 层 -> 推进器指令。
// 2. 与谁交互：
//    - 通过串口与下位机 STM32 MCU 双向通信（115200 8N1）；
//    - 订阅话题 actuators/thrusters/setpoint（电机设定点，来自 control 层）；
//    - 发布 gateway/frames（解析后的 MCU 报文）、gateway/raw_rx（原始字节，仅 publish_raw=true）、
//      gateway/status（网关状态）；
//    - 提供 3 个服务：gateway/control（控制权申请/切换）、
//      gateway/validate_manual_lease（GUI 手动租约校验）、
//      sensors/depth/zero（深度计水面零点标定）。
// 3. 涉及的 ROS 2 概念：
//    - Node：一个能收发话题、提供服务的独立程序；继承 rclcpp::Node 后即可使用
//      create_publisher / create_subscription / create_service / create_wall_timer。
//    - Publisher/Subscription：发布者/订阅者，按话题名通信，消息类型写在 <...> 里。
//    - Service：请求-应答式接口，客户端调用后等待一次响应。
//    - Parameter：节点启动参数（declare_parameter），可在 launch/YAML 中配置。
//    - QoS：服务质量策略，SensorDataQoS 适合高频传感器数据。
//    - Executor/spin：rclcpp::spin 让节点循环处理回调，直到 shutdown。
//    - 命名空间：话题/服务名按节点命名空间解析，便于多机或多实例隔离。
// 4. 涉及的 C++ 概念：
//    - std::chrono 时钟与时间字面量（10ms、1s）用于定时；
//    - std::optional 表示“可能有值”，std::array 是固定长度数组；
//    - lambda 是匿名函数，[this]{...} 捕获本对象，常作为回调传入；
//    - override 表示重写基类虚函数；
//    - 智能指针 SharedPtr/unique_ptr 自动管理对象生命周期，无需手动 delete。
// ------------------------------------------------------------------------------
#include "underwater_mcu_gateway/protocol.hpp"
#include "underwater_mcu_gateway/serial_io.hpp"
#include "underwater_mcu_gateway/tcp_serial_bridge.hpp"
#include "underwater_mcu_gateway/motor_commands.hpp"
#include "underwater_mcu_gateway/depth_zero.hpp"
#include "underwater_mcu_gateway/control_ownership.hpp"
#include "underwater_mcu_gateway/tcp_control_server.hpp"
#include "underwater_mcu_gateway/motor_authority.hpp"

#include <chrono>
#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <underwater_interfaces/msg/gateway_frame.hpp>
#include <underwater_interfaces/msg/gateway_status.hpp>
#include <underwater_interfaces/msg/raw_serial.hpp>
#include <underwater_interfaces/msg/motor_setpoint.hpp>
#include <underwater_interfaces/srv/control_ownership.hpp>
#include <underwater_interfaces/srv/validate_manual_lease.hpp>
#include <std_srvs/srv/trigger.hpp>

using Clock = std::chrono::steady_clock;
using underwater::ControllerOwner;
using underwater::OwnershipResult;
using underwater::controller_name;
using namespace std::chrono_literals;

namespace ui = underwater_interfaces::msg;
namespace srv = underwater_interfaces::srv;

// ROS 2 小知识：继承 rclcpp::Node 的类就是一个节点；final 表示它不允许再被继承。
// GatewayNode 把“串口收发 + 协议解析 + 控制权仲裁 + 话题/服务”集中在一个节点里。
// 下面所有方法基本都是对回调、定时器或底层组件的封装。
class GatewayNode final : public rclcpp::Node {
public:
  GatewayNode() : Node("mcu_gateway") {
// 构造函数：节点名注册为 "mcu_gateway"，启动时一次性声明参数、创建发布者/订阅者/服务与两个定时器。
// Parameter 描述符 read_only=true 表示这些参数声明后运行期不能再被动态修改。
    rcl_interfaces::msg::ParameterDescriptor d;
    d.read_only = true;

// declare_parameter<类型>(名字, 默认值, 描述符)：声明并读取 ROS 2 参数；
// 若 launch/YAML 提供了同名参数就用它，否则用默认值。
    port_ = declare_parameter<std::string>(
        "port",
        "/dev/serial/by-id/usb-STMicroelectronics_Lamost_UnderWater_Robot_ComPort_3588346E3134-if00",
        d);
    settle_ms_ = declare_parameter<int>("settle_ms", 1000, d);
    max_lag_us_ = declare_parameter<int>("max_device_lag_ms", 500, d) * 1000LL;
    raw_enabled_ = declare_parameter<bool>("publish_raw", false, d);
    motor_tx_ = declare_parameter<bool>("enable_motor_tx", false, d);
    depth_commands_ = declare_parameter<bool>("enable_depth_commands", true, d);
    tcp_port_ = declare_parameter<int>("tcp_port", 0, d);
    tcp_control_port_ = declare_parameter<int>("tcp_control_port", 9002, d);

    const auto tcp_bind = declare_parameter<std::string>("tcp_bind_address", "0.0.0.0", d);
    tcp_allow_tx_ = declare_parameter<bool>("tcp_allow_tx", false, d);

    if (tcp_port_ < 0 || tcp_port_ > 65535 ||
        tcp_control_port_ < 0 || tcp_control_port_ > 65535 ||
        (tcp_port_ && tcp_control_port_ && tcp_port_ == tcp_control_port_)) {
      throw std::invalid_argument("TCP data/control ports must be distinct values in 0..65535");
    }

    if (tcp_allow_tx_) {
      RCLCPP_WARN(
          get_logger(),
          "tcp_allow_tx is deprecated and does not grant control; use an explicit ownership takeover");
    }

    if (tcp_port_) {
      tcp_.open(tcp_bind, static_cast<std::uint16_t>(tcp_port_));
      if (tcp_control_port_) {
        control_server_.open(tcp_bind, static_cast<std::uint16_t>(tcp_control_port_));
        RCLCPP_INFO(get_logger(), "Control ownership server %s:%d", tcp_bind.c_str(), tcp_control_port_);
      }
      RCLCPP_INFO(
          get_logger(),
          "Raw serial TCP server %s:%d (Nano initially owns; GUI writes require an explicit takeover and session token)",
          tcp_bind.c_str(),
          tcp_port_);
    }

    command_timeout_ms_ = declare_parameter<int>("motor_command_timeout_ms", 500, d);
    if (command_timeout_ms_ < 100 || command_timeout_ms_ > 1000) {
      throw std::invalid_argument("motor command timeout must be 100..1000 ms");
    }

    if (settle_ms_ < 0 || settle_ms_ > 10000 || max_lag_us_ <= 0 || max_lag_us_ > 10000000) {
      throw std::invalid_argument("Invalid settling or freshness parameter");
    }

// QoS（服务质量）：SensorDataQoS() 适合传感器流，keep_last(100) 表示队列最多缓存 100 条。
// 下面创建 3 个发布者：gateway/frames（解析后的帧）、gateway/raw_rx（原始字节，仅 publish_raw=true 时真正发布）、gateway/status。
    auto q = rclcpp::SensorDataQoS().keep_last(100);
    frames_ = create_publisher<ui::GatewayFrame>("gateway/frames", q);
    raw_ = create_publisher<ui::RawSerial>("gateway/raw_rx", rclcpp::SensorDataQoS().keep_last(10));
    status_ = create_publisher<ui::GatewayStatus>("gateway/status", 10);

// create_subscription<消息类型>(话题名, QoS, 回调)：每收到一条话题消息就执行一次回调；
// 这里用 lambda 把消息转交给 command() 处理。话题名不以 '/' 开头时是相对名字。
    // 订阅电机设定点话题
    commands_ = create_subscription<ui::MotorSetpoint>(
        "actuators/thrusters/setpoint",
        rclcpp::QoS(1),
        [this](ui::MotorSetpoint::ConstSharedPtr m) { command(*m); });

// create_service<服务类型>(服务名, 回调)：客户端请求到来时执行回调，在回调里填充 response。
// 请求里的 action 字段区分 CLAIM（申请）/ TAKEOVER（抢占）/ RELEASE（释放）。
    // 提供控制权申请/切换服务
    ownership_service_ = create_service<srv::ControlOwnership>(
        "gateway/control",
        [this](
            const std::shared_ptr<srv::ControlOwnership::Request> request,
            std::shared_ptr<srv::ControlOwnership::Response> response) {
          OwnershipResult result;
          if (request->action == srv::ControlOwnership::Request::CLAIM) {
            result = ownership_.claim_nano(false);
          } else if (request->action == srv::ControlOwnership::Request::TAKEOVER) {
            result = ownership_.claim_nano(true);
          } else if (request->action == srv::ControlOwnership::Request::RELEASE) {
            result = ownership_.release_nano();
          } else {
            result = {false, ownership_.owner(), {}, "Unknown control action"};
          }
          response->success = result.success;
          response->owner = static_cast<std::uint8_t>(result.owner);
          response->message = result.message;
        });

// ValidateManualLease 服务：GUI 定期用 token 续租，网关返回 generation 证明其仍有权限。
// 若续租结果要求停机，则立刻调用 apply_authority_change() 安全刹停。
    // 提供 GUI 手动业务租约校验服务
    manual_lease_service_ = create_service<srv::ValidateManualLease>(
        "gateway/validate_manual_lease",
        [this](
            const std::shared_ptr<srv::ValidateManualLease::Request> request,
            std::shared_ptr<srv::ValidateManualLease::Response> response) {
          const auto renewal = motor_authority_.renew_manual_lease(request->token);
          if (renewal.stop_required) {
            apply_authority_change();
          }
          response->generation = renewal.generation;
          response->valid = (response->generation != 0);
          response->detail = response->valid ? "Current GUI business lease verified"
                                             : "GUI business lease unavailable or expired";
        });

// 深度零点服务使用标准 std_srvs/srv/Trigger（空请求）。
// 这个回调是“异步”的：先校验前置条件、记录请求头，之后由状态机分步完成，最后用 send_response 回包。
    // 深度去皮标定触发服务
    zero_service_ = create_service<std_srvs::srv::Trigger>(
        "sensors/depth/zero",
        [this](std::shared_ptr<rmw_request_id_t> header, std_srvs::srv::Trigger::Request::SharedPtr) {
          if (ownership_.owner() != ControllerOwner::Nano ||
              !depth_commands_ ||
              !serial_.connected() ||
              (Clock::now() - opened_ < std::chrono::milliseconds(settle_ms_))) {
            zero_reply(
                header,
                false,
                ownership_.owner() == ControllerOwner::Gui
                    ? "GUI owns robot control; use TAKEOVER to return control to Nano"
                    : "Depth commands disabled, disconnected, or gateway still settling");
            return;
          }

          if (zero_request_ || depth_zero_.busy()) {
            zero_reply(header, false, "Depth zero is already in progress");
            return;
          }

          zero_request_ = header;
          depth_zero_.start(monotonic_ms());
          try {
            process_depth_zero();
          } catch (const std::exception &e) {
            serial_failure(e.what());
          }
        });

// create_wall_timer(周期, 回调)：按固定周期触发。10ms 主轮询负责串口收发与解析，
// 1s 定时器周期性发布网关状态；两个回调都在同一个 executor 线程里串行执行。
    // 10ms 主轮询定时器与 1s 状态报告定时器
    tick_ = create_wall_timer(10ms, [this] { poll(); });
    report_ = create_wall_timer(1s, [this] { publish_status(); });
  }

// 析构：节点退出时尽量向串口写一帧“全零停机”，保证推进器不被残留指令驱动。
// try/catch(...) 吞掉异常，因为析构函数里不应再抛出异常。
  ~GatewayNode() override {
    try {
      if (serial_.connected()) {
        send_motor({}, 0);
      }
    } catch (...) {}
  }

private:
  // 异步应答深度去皮标定请求
// 异步回复深度标定服务：把结果封装成 Trigger::Response 发回去。
// send_response 需要用到最初保存的请求头；异常时只记录警告，不影响主循环。
  void zero_reply(const std::shared_ptr<rmw_request_id_t> &header, bool success, const std::string &message) {
    std_srvs::srv::Trigger::Response response;
    response.success = success;
    response.message = message;
    try {
      zero_service_->send_response(*header, response);
    } catch (const std::exception &e) {
      RCLCPP_WARN(get_logger(), "Depth zero response: %s", e.what());
    }
  }

  // 推进深度去皮状态机执行
// 推进深度去皮状态机，每轮 poll 调用一次：
// tick() 驱动状态机；take_request() 若返回要下发的字节就写入串口；
// take_result() 若返回最终结果就回复服务请求。std::optional 便于表达“本步暂无内容”。
  void process_depth_zero() {
    depth_zero_.tick(monotonic_ms());
    if (auto bytes = depth_zero_.take_request()) {
      if (ownership_.owner() != ControllerOwner::Nano) {
        depth_zero_.cancel("Control ownership changed before the depth request was sent");
      } else {
        serial_.write_all(*bytes);
        tx_bytes_ += bytes->size();
        ++tx_frames_;
      }
    }

    if (auto result = depth_zero_.take_result()) {
      if (zero_request_) {
        zero_reply(zero_request_, result->success, result->message);
        zero_request_.reset();
      }
      RCLCPP_INFO(get_logger(), "Depth reference: %s", result->message.c_str());
    }
  }

// 单调时钟毫秒时间戳：只用于计算时间差，不受系统时间被人为调整的影响。
  static std::uint64_t monotonic_ms() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        Clock::now().time_since_epoch()).count();
  }

  // 下发电机双帧事务
// 下发一次电机事务：motor_transaction 会生成“双帧”CAN 透传报文，这里逐帧写入串口。
// tx_sequence_ 是递增的事务序号，用于下位机防重放。
  void send_motor(const underwater::wire::MotorRpm &rpm, std::uint8_t run) {
    for (const auto &bytes : underwater::wire::motor_transaction(rpm, run, tx_sequence_)) {
      serial_.write_all(bytes);
      tx_bytes_ += bytes.size();
      ++tx_frames_;
    }
  }

  // 接收并执行电机设定点
// 收到电机设定点话题后的处理入口，也是电机安全门禁的汇聚点：
// 依次校验 ROS 层授权、使能开关、串口连接、连接代次、稳定期、消息时间戳新鲜度，
// 再交给 MotorGate 做会话握手/防重放校验，全部通过才真正写串口。
  void command(const ui::MotorSetpoint &m) {
    synchronize_owner();

    if (!motor_authority_.ros_motor_allowed() ||
        !motor_tx_ ||
        !serial_.connected() ||
        m.connection_id != connection_ ||
        (Clock::now() - opened_ < std::chrono::milliseconds(settle_ms_))) {
      return;
    }

// 时间戳新鲜度：header.stamp 是 ROS 时间，与当前时间相差过大（超前 100ms 或滞后 250ms）
// 说明是过期或异常指令，直接丢弃，避免用旧指令驱动推进器。
    const auto age = (this->now() - rclcpp::Time(m.header.stamp)).nanoseconds();
    if (age < -100000000LL || age > 250000000LL) {
      return;
    }

// gate_.accept 是“安全闸门”：新会话首次必须全零握手，之后同会话序号必须递增；
// 返回 false 表示本次指令非法，直接忽略。
    const auto previous_session = gate_.session();
    if (!gate_.accept(m.session_id, m.sequence, m.armed, m.node_mask, m.run_mask, m.rpm, monotonic_ms())) {
      return;
    }

    try {
      send_motor(m.rpm, m.run_mask);
      command_detail_ = m.run_mask ? "motor vectors written; feedback must confirm motion"
                                   : "RUN_VECTOR stop written";
      if (previous_session != gate_.session()) {
        publish_status();
      }
    } catch (const std::exception &e) {
      serial_failure(e.what());
    }
  }

// 把底层 MotorAuthority 的授权状态同步到网关：若发生控制权或租约变化，则执行安全停机。
  void synchronize_owner() {
    if (!motor_authority_.synchronize()) {
      return;
    }
    apply_authority_change();
  }

  // 控制权发生变更时，重置下发状态并向串口安全刹停
// 控制权发生变更（抢占/释放/租约过期）时的统一处理：
// 向串口写全零停机、取消未完成的深度标定、通知 TCP 层更新持有者，并发布会话状态。
  void apply_authority_change() {
    const auto owner = ownership_.owner();
    const auto token = ownership_.gui_token();

    if (serial_.connected()) {
      try {
        send_motor({}, 0);
        command_detail_ = "control authority changed or expired: stop transaction written";
      } catch (const std::exception &e) {
        detail_ = e.what();
      }
    }

    if (owner != ControllerOwner::Nano && depth_zero_.busy()) {
      depth_zero_.cancel("Control ownership changed; Nano will not continue the transaction");
      process_depth_zero();
    }

    tcp_.set_controller(owner, token);
    publish_status();
    RCLCPP_INFO(get_logger(), "Robot control owner is now %s", controller_name(owner));
  }

  // 串口硬件断链处理
// 串口硬件断链的统一处理：关闭串口、复位解析器与状态机、取消深度标定、
// 复位电机闸门并要求显式重新使能，避免断线后自动恢复驱动。
  void serial_failure(const std::string &reason) {
    detail_ = reason;
    RCLCPP_WARN(get_logger(), "%s", detail_.c_str());

    tcp_.set_serial_ready(false);
    tcp_.take_disconnect();
    serial_.close();
    parser_.reset();
    heartbeat_.reset();
    buffers_valid_ = false;

    depth_zero_.cancel("Serial disconnected during depth zero; not retried");
    process_depth_zero();

    gate_.reset();
    command_detail_ = "disconnected; explicit rearm required";
    publish_status();
  }

  // 10ms 主轮询循环
// 10ms 主轮询循环，是整个网关的“心脏”：
// 1) 轮询控制服务并同步控制权；2) 串口未连接时尝试每秒重连；
// 3) 在 TCP 与串口之间转发数据；4) 解析 MCU 上报帧并发布 ROS 话题；
// 5) 检查电机指令看门狗是否超时并停机。
  void poll() {
    const auto now = Clock::now();
    control_server_.poll();
    synchronize_owner();

// 自动重连：next_open_ 控制重试节流，避免每秒疯狂 open 设备。
    // 串口未连接时尝试每秒重新打开
    if (!serial_.connected()) {
      tcp_.set_serial_ready(false);
      tcp_.poll();
      tcp_.take_disconnect();

      if (now < next_open_) {
        return;
      }
      next_open_ = now + 1s;

      const char *stage = "open/configure";
      try {
        serial_.open(port_, motor_tx_ || depth_commands_ || tcp_port_ > 0);
        parser_.reset();
        high_water_.reset();
        heartbeat_.reset();
        gate_.reset();

        opened_ = now;
        ++connection_;
        detail_ = "connected, settling";

        if (motor_tx_ || tcp_port_ > 0) {
          stage = "startup stop TX";
          send_motor({}, 0);
          command_detail_ = "startup stop written; awaiting explicit commands";
        }
        RCLCPP_INFO(get_logger(), "Serial opened: %s; motor TX=%s", port_.c_str(), motor_tx_ ? "enabled" : "disabled");
      } catch (const std::exception &e) {
        detail_ = std::string(stage) + ": " + e.what();
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            5000,
            "Serial %s failed on %s: %s",
            stage,
            port_.c_str(),
            e.what());
        serial_.close();
        gate_.reset();
        return;
      }
    }

    try {
// 串口稳定期（settle_ms_）过后才告诉 TCP 层“可转发”，避免启动瞬间的乱码被下发。
      tcp_.set_serial_ready(now - opened_ >= std::chrono::milliseconds(settle_ms_));
      const auto remote = tcp_.poll();
      tcp_.take_disconnect();

      if (tcp_.take_write_disconnect() && ownership_.owner() == ControllerOwner::Gui) {
        ownership_.claim_nano(true);
        synchronize_owner();
        command_detail_ = "GUI serial session disconnected: control returned to Nano and stop sent";
      }

      // 透传 TCP 客户端发来的数据至串口
      if (!remote.empty()) {
        if (tcp_.write_authorized() && ownership_.owner() == ControllerOwner::Gui) {
          serial_.write_all(remote);
          tx_bytes_ += remote.size();
        } else {
          RCLCPP_ERROR_THROTTLE(
              get_logger(),
              *get_clock(),
              2000,
              "TCP write rejected; robot control is owned by %s. Use the explicit TAKEOVER action.",
              controller_name(ownership_.owner()));
        }
      }

      // 从串口读取字节
      auto bytes = serial_.read_available();
      rx_ += bytes.size();
      tcp_.forward_rx(bytes);
      tcp_.take_disconnect();

      if (tcp_.take_write_disconnect() && ownership_.owner() == ControllerOwner::Gui) {
        ownership_.claim_nano(true);
        synchronize_owner();
        command_detail_ = "GUI serial backpressure/disconnect: control returned to Nano and stop sent";
      }

      auto stamp = this->now();
      if (raw_enabled_ && !bytes.empty()) {
        ui::RawSerial raw;
        raw.header.stamp = stamp;
        raw.connection_id = connection_;
        raw.data = bytes;
        raw_->publish(raw);
      }

      const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
// Parser 是增量流解析器：喂入原始字节，返回本轮解析出的完整帧列表。
// 每帧做时间戳单调性检查：用 0x80000000 半量程判断前进/倒退，并特判恰好半量程的歧义帧。
      for (auto &f : parser_.feed(bytes, ms)) {
        // 稳定期抛弃早期堆积报文
        if (now - opened_ < std::chrono::milliseconds(settle_ms_)) {
          ++stale_;
          high_water_.reset();
          continue;
        }

        bool stale = false;
        if (f.timestamp_valid) {
          if (!high_water_) {
            high_water_ = f.timestamp_us;
          } else {
            const auto delta = static_cast<std::uint32_t>(f.timestamp_us - *high_water_);
            if (delta == 0x80000000u) {
              stale = true;
            } else if (delta < 0x80000000u) {
              *high_water_ = f.timestamp_us;
            } else {
              stale = (static_cast<std::uint32_t>(*high_water_ - f.timestamp_us) > max_lag_us_);
            }
          }
        }
        if (stale) {
          ++stale_;
          continue;
        }

// 把每帧交给深度标定状态机观察，状态机据此推进事务。
        depth_zero_.observe(f, ms);

// 把协议帧字段拷贝到 ROS 消息 GatewayFrame；payload 用 std::move 转移所有权，避免大数组拷贝。
        ui::GatewayFrame m;
        m.header.stamp = stamp;
        m.header.frame_id = "mcu_gateway";
        m.connection_id = connection_;
        m.family = f.family;
        m.command = f.command;
        m.flags = f.flags;
        m.target = f.target;
        m.sequence = f.sequence;
        m.can_id = f.can_id;
        m.device_timestamp_valid = f.timestamp_valid;
        m.device_timestamp_us = f.timestamp_us;
        m.payload = std::move(f.payload);
        frames_->publish(m);

// 0x58 是心跳帧；payload 恰为 5 字节时这 5 个字节是各缓冲区占用率（payload 为 0 时仅表示心跳）。
        if (m.family == 0x58) {
          heartbeat_ = now;
          buffers_valid_ = (m.payload.size() == 5);
          if (buffers_valid_) {
            std::copy(m.payload.begin(), m.payload.end(), buffers_.begin());
          }
        }
      }

      process_depth_zero();

// 看门狗：超过 command_timeout_ms_ 没收到新指令就主动停机，防止失控。
      // 电机指令看门狗超时停机
      if (motor_tx_ && gate_.expired(monotonic_ms(), command_timeout_ms_)) {
        send_motor({}, 0);
        command_detail_ = "command timeout: stopped; new enable session required";
        publish_status();
      }

      detail_ = tcp_port_ ? tcp_.detail() : (motor_tx_ ? "motor TX enabled, bounded setpoints only"
                                                       : "RX-only; motor TX disabled");
    } catch (const std::exception &e) {
      serial_failure(e.what());
    }
  }

  // 1s 周期发布网关运行状态
// 周期发布网关运行状态：串口连接、字节/帧计数器、解析器统计、心跳年龄、控制权等，
// 供上层监控与诊断使用。
  void publish_status() {
    ui::GatewayStatus s;
    s.header.stamp = now();
    s.port = port_;
    s.connected = serial_.connected();
    s.settling = s.connected && (Clock::now() - opened_ < std::chrono::milliseconds(settle_ms_));
    s.tx_enabled = motor_tx_;
    s.connection_id = connection_;
    s.rx_bytes = rx_;
    s.stale_frames = stale_;
    s.tx_bytes = tx_bytes_;
    s.tx_frames = tx_frames_;
    s.motor_session_id = gate_.session();
    s.motor_command_active = gate_.active();
    s.motor_rearm_required = gate_.blocked();
    s.motor_command_detail = command_detail_;
    s.control_owner = controller_name(ownership_.owner());
    s.tcp_control_port = control_server_.port();

    const auto &c = parser_.counters();
    s.valid_frames = c.valid;
    s.crc_errors = c.crc_errors;
    s.shape_errors = c.shape_errors;
    s.discarded_bytes = c.discarded;

    s.heartbeat_age_ms = heartbeat_
        ? std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - *heartbeat_).count()
        : -1;
    s.buffer_status_valid = buffers_valid_ && s.heartbeat_age_ms >= 0 && s.heartbeat_age_ms < 2500;
    s.buffers_percent = buffers_;
    s.detail = detail_;
    status_->publish(s);
  }

// 以下是节点的成员对象，按依赖顺序构造：串口、TCP 桥、控制权、控制服务、
// 协议解析器、电机闸门、电机授权（组合了控制权与闸门）。
  // 底层通信与控制组件
  underwater::SerialIo serial_;
  underwater::TcpSerialBridge tcp_;
  underwater::ControlOwnership ownership_;
  underwater::TcpControlServer control_server_{ownership_};
  underwater::wire::Parser parser_;
  underwater::wire::MotorGate gate_;
  underwater::MotorAuthority motor_authority_{ownership_, gate_};

  int tcp_port_ = 0;
  int tcp_control_port_ = 9002;
  bool tcp_allow_tx_ = false;

  std::string port_;
  std::string detail_ = "not connected";
  std::string command_detail_ = "motor TX disabled";

  int settle_ms_ = 1000;
  std::int64_t max_lag_us_ = 500000;
  int command_timeout_ms_ = 500;

  bool raw_enabled_ = false;
  bool buffers_valid_ = false;
  bool motor_tx_ = false;
  bool depth_commands_ = true;

  underwater::wire::DepthZero depth_zero_{static_cast<std::uint32_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(Clock::now().time_since_epoch()).count())};

  std::shared_ptr<rmw_request_id_t> zero_request_;
  std::uint16_t tx_sequence_ = 0;
  std::uint64_t tx_bytes_ = 0;
  std::uint64_t tx_frames_ = 0;
  std::uint64_t connection_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
      Clock::now().time_since_epoch()).count();
  std::uint64_t rx_ = 0;
  std::uint64_t stale_ = 0;

  std::array<std::uint8_t, 5> buffers_{};
  std::optional<std::uint32_t> high_water_;
  std::optional<Clock::time_point> heartbeat_;
  Clock::time_point opened_{};
  Clock::time_point next_open_{};

// ROS 2 句柄用 SharedPtr 保存，防止在节点运行期间被回收；定时器同理。
  // ROS 发布者、订阅者与服务
  rclcpp::Publisher<ui::GatewayFrame>::SharedPtr frames_;
  rclcpp::Publisher<ui::RawSerial>::SharedPtr raw_;
  rclcpp::Publisher<ui::GatewayStatus>::SharedPtr status_;
  rclcpp::Subscription<ui::MotorSetpoint>::SharedPtr commands_;
  rclcpp::Service<srv::ControlOwnership>::SharedPtr ownership_service_;
  rclcpp::Service<srv::ValidateManualLease>::SharedPtr manual_lease_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr zero_service_;
  rclcpp::TimerBase::SharedPtr tick_;
  rclcpp::TimerBase::SharedPtr report_;
};

// 程序入口：rclcpp::init 初始化 ROS 2，rclcpp::spin 启动执行器循环处理回调。
// 构造节点时若参数非法会抛异常，这里捕获后打印错误并 shutdown。
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<GatewayNode>());
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("mcu_gateway"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
