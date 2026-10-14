// ==============================================================================
// internal/underwater_mcu_gateway/src/nodes/mcu_gateway_node.cpp
// ==============================================================================
// MCU 串口网关核心节点：
// - 管理下位机硬件串口 (115200 8N1 raw 独占模式) 的连接生命周期与自动重连；
// - 提供 9001 端口 (原始串口字节透传) 与 9002 端口 (所有权仲裁控制通道)；
// - 解码 MCU 上行 0x55(CAN), 0x58(心跳), 0x59(通用), 0x5b(传感器) 报文并发布至 ROS；
// - 严格执行电机下发安全门禁 (MotorGate & MotorAuthority)，心跳超时自动停机；
// - 承载深度计水面零点自动去皮标定服务 (sensors/depth/zero)。
// ==============================================================================
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

class GatewayNode final : public rclcpp::Node {
public:
  GatewayNode() : Node("mcu_gateway") {
    rcl_interfaces::msg::ParameterDescriptor d;
    d.read_only = true;

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

    auto q = rclcpp::SensorDataQoS().keep_last(100);
    frames_ = create_publisher<ui::GatewayFrame>("gateway/frames", q);
    raw_ = create_publisher<ui::RawSerial>("gateway/raw_rx", rclcpp::SensorDataQoS().keep_last(10));
    status_ = create_publisher<ui::GatewayStatus>("gateway/status", 10);

    // 订阅电机设定点话题
    commands_ = create_subscription<ui::MotorSetpoint>(
        "actuators/thrusters/setpoint",
        rclcpp::QoS(1),
        [this](ui::MotorSetpoint::ConstSharedPtr m) { command(*m); });

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

    // 10ms 主轮询定时器与 1s 状态报告定时器
    tick_ = create_wall_timer(10ms, [this] { poll(); });
    report_ = create_wall_timer(1s, [this] { publish_status(); });
  }

  ~GatewayNode() override {
    try {
      if (serial_.connected()) {
        send_motor({}, 0);
      }
    } catch (...) {}
  }

private:
  // 异步应答深度去皮标定请求
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

  static std::uint64_t monotonic_ms() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        Clock::now().time_since_epoch()).count();
  }

  // 下发电机双帧事务
  void send_motor(const underwater::wire::MotorRpm &rpm, std::uint8_t run) {
    for (const auto &bytes : underwater::wire::motor_transaction(rpm, run, tx_sequence_)) {
      serial_.write_all(bytes);
      tx_bytes_ += bytes.size();
      ++tx_frames_;
    }
  }

  // 接收并执行电机设定点
  void command(const ui::MotorSetpoint &m) {
    synchronize_owner();

    if (!motor_authority_.ros_motor_allowed() ||
        !motor_tx_ ||
        !serial_.connected() ||
        m.connection_id != connection_ ||
        (Clock::now() - opened_ < std::chrono::milliseconds(settle_ms_))) {
      return;
    }

    const auto age = (this->now() - rclcpp::Time(m.header.stamp)).nanoseconds();
    if (age < -100000000LL || age > 250000000LL) {
      return;
    }

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

  void synchronize_owner() {
    if (!motor_authority_.synchronize()) {
      return;
    }
    apply_authority_change();
  }

  // 控制权发生变更时，重置下发状态并向串口安全刹停
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
  void poll() {
    const auto now = Clock::now();
    control_server_.poll();
    synchronize_owner();

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

        depth_zero_.observe(f, ms);

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

        if (m.family == 0x58) {
          heartbeat_ = now;
          buffers_valid_ = (m.payload.size() == 5);
          if (buffers_valid_) {
            std::copy(m.payload.begin(), m.payload.end(), buffers_.begin());
          }
        }
      }

      process_depth_zero();

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
