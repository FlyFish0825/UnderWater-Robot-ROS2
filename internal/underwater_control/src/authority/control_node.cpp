// ==============================================================================
// internal/underwater_control/src/authority/control_node.cpp
// ==============================================================================
// 推力分配节点：把 geometry_msgs/WrenchStamped 请求（物理 wrench 或手动档位）
// 经 Allocator 求解后发布 MotorSetpoint，并向网关确认会话健康后才允许输出。
// 安全门限：必须显式 control/enable、网关已连接且 tx_enabled、零点握手完成、
// wrench 输入新鲜且只有一个发布者；任一条件失效立即解除武装并发送停机指令。
// ==============================================================================
// ------------------------------------------------------------------------------
// 【初学者导读】本文件是推力分配的 ROS 2 节点（可执行程序），类名 ControlNode，
// 节点名 "thrust_allocator"。所属：internal/underwater_control。
// 它做什么：订阅上层发来的期望 wrench（物理量）或手动档位，调用 Allocator 求解，
//          再向执行机构发布每台电机的目标转速。
// 与谁交互（ROS 2 话题/服务）：
//   订阅 control/wrench_request  (geometry_msgs/WrenchStamped)，物理力/力矩输入；
//   订阅 control/manual_request  (geometry_msgs/WrenchStamped)，手动档位输入；
//   订阅 gateway/status          (underwater_interfaces/GatewayStatus)，MCU 网关健康状态；
//   发布 actuators/thrusters/setpoint (MotorSetpoint)，给电机的最终指令；
//   发布 control/allocation      (AllocationStatus)，调试与可视化状态；
//   提供服务 control/enable      (std_srvs/SetBool)，操作手使能/紧急停止。
// ROS 2 概念白话：
//   Node（节点）＝一个能收发话题、调用服务的独立小程序；
//   Publisher/Subscription＝发布/订阅某类消息的通道；
//   Service＝一次请求-应答的调用，适合“开/关”这类动作；
//   Parameter＝启动时可配置的参数（只读参数用 read_only 标记）；
//   QoS(1)＝队列深度 1，只保留最新一条，适合实时指令；
//   Executor/spin＝事件循环，负责把回调依次取出来执行；
//   Timer＝周期性触发的回调（这里 20ms 一次，同时兼作看门狗）。
// C++ 概念白话：
//   std::unique_ptr＝独占所有权的智能指针，离开作用域自动释放；
//   std::shared_ptr＝引用计数共享所有权；
//   lambda＝匿名函数，[] 捕获外部变量，(参数){函数体}，常作为回调；
//   std::optional＝“可能有值也可能没有”的容器，用 if(x) 判断是否存在；
//   override＝明确表示重写了基类的虚函数；
//   RAII＝构造时获取资源、析构时自动释放，是 C++ 管理资源的惯用法。
// 安全设计：必须显式 control/enable、网关就绪、零点握手完成、输入新鲜且发布者唯一，
//          任一条件失效立即解除武装并下发零转速，属于“失效即安全”。
// ------------------------------------------------------------------------------
#include "underwater_control/allocator.hpp"
// 本包公共头文件：分配器与手动权限校验。
#include "underwater_control/manual_authority.hpp"

#include <algorithm>
#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <vector>

// ROS 2 客户端库 rclcpp 与各类消息头文件。
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <underwater_interfaces/msg/allocation_status.hpp>
#include <underwater_interfaces/msg/gateway_status.hpp>
#include <underwater_interfaces/msg/motor_setpoint.hpp>

// 起别名让代码更短：uc=内部控制命名空间，ui=消息命名空间。
// Clock=steady_clock 单调时钟（不受系统时间调整影响），适合测超时；
// using namespace std::chrono_literals 让 "20ms" 这种写法可用。
namespace uc = underwater::control;
namespace ui = underwater_interfaces::msg;
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;

// ControlNode：推力分配节点本体。继承 rclcpp::Node；final 表示不再被继承。
class ControlNode final : public rclcpp::Node {
public:
// 构造函数：声明参数、创建话题/服务/定时器，并进入默认安全状态（未武装）。
// 坑位：参数尺寸不对或数值非法会抛异常，导致整个节点启动失败。
  ControlNode() : Node("thrust_allocator") {
    rcl_interfaces::msg::ParameterDescriptor d;
    d.read_only = true;

    uc::Config cfg;

// 小工具 lambda：把 Config 里的数组声明成只读 ROS parameter 并返回其值。
// declare_parameter 会在参数未提供时使用 initial 作为默认值。
    auto numbers = [&](const char *name, const auto &initial) {
      return declare_parameter<std::vector<double>>(
          name, std::vector<double>(initial.begin(), initial.end()), d);
    };

    std::vector<double> positions, axes;
    for (std::size_t i = 0; i < 8; ++i) {
      positions.insert(positions.end(), cfg.positions[i].begin(), cfg.positions[i].end());
      axes.insert(axes.end(), cfg.axes[i].begin(), cfg.axes[i].end());
    }

    // 24 个数：8 桨 × (x, y, z)，单位 m
    positions = declare_parameter<std::vector<double>>("positions_m", positions, d);
    // 24 个数：8 桨 × 单位方向
    axes = declare_parameter<std::vector<double>>("axes", axes, d);

    auto force = numbers("nominal_max_thrust_n", cfg.nominal_max_thrust_n);  // 8 个额定推力 (N)
    auto weights = numbers("wrench_weights", cfg.weights);                   // 6 个 wrench 权重
    auto ids = declare_parameter<std::vector<std::int64_t>>("node_ids", {1, 2, 3, 4, 5, 6, 7, 8}, d);
    auto dirs = declare_parameter<std::vector<std::int64_t>>(
        "motor_direction",
        std::vector<std::int64_t>(cfg.motor_direction.begin(), cfg.motor_direction.end()),
        d);

    // 校验数组尺寸完整性：尺寸不符即拒绝启动，避免半配置运行
    if (positions.size() != 24 || axes.size() != 24 || force.size() != 8 ||
        weights.size() != 6 || ids.size() != 8 || dirs.size() != 8) {
      throw std::invalid_argument("allocation configuration array size mismatch");
    }

    for (std::size_t i = 0; i < 8; ++i) {
      if (ids[i] < 1 || ids[i] > 8 || (dirs[i] != 1 && dirs[i] != -1)) {
        throw std::invalid_argument("invalid node map");
      }
      cfg.node_ids[i] = ids[i];
      cfg.motor_direction[i] = dirs[i];
      cfg.nominal_max_thrust_n[i] = force[i];
      for (int j = 0; j < 3; ++j) {
        cfg.positions[i][j] = positions[3 * i + j];
        cfg.axes[i][j] = axes[3 * i + j];
      }
    }

    std::copy(weights.begin(), weights.end(), cfg.weights.begin());
    cfg.min_rpm = declare_parameter<int>("min_rpm", 1000, d);
    cfg.max_rpm = declare_parameter<int>("max_rpm", 6000, d);
    cfg.manual_start_rpm = declare_parameter<int>("manual_start_rpm", 1200, d);

    const auto manual_signs = declare_parameter<std::vector<std::int64_t>>(
        "manual_axis_signs",
        std::vector<std::int64_t>(cfg.manual_axis_signs.begin(), cfg.manual_axis_signs.end()),
        d);
    if (manual_signs.size() != 6) {
      throw std::invalid_argument("manual_axis_signs must contain six signs");
    }
    for (std::size_t j = 0; j < 6; ++j) {
      if (manual_signs[j] != 1 && manual_signs[j] != -1) {
        throw std::invalid_argument("manual_axis_signs must be +/-1");
      }
      cfg.manual_axis_signs[j] = manual_signs[j];
    }

    cfg.effort_weight = declare_parameter<double>("effort_weight", 1e-6, d);
    timeout_ = std::chrono::milliseconds(declare_parameter<int>("input_timeout_ms", 500, d));
    if (timeout_.count() < 100 || timeout_.count() > 1000) {
      throw std::invalid_argument("input timeout must be 100..1000 ms");
    }

    frame_id_ = declare_parameter<std::string>("wrench_frame_id", "thruster_center", d);

    // 构造几何分配器
// 用配置构造 Allocator；unique_ptr 表示本节点独占拥有它，析构时自动释放。
    allocator_ = std::make_unique<uc::Allocator>(cfg);

    // 创建电机控制指令与分配状态发布者
// 发布者：output_ 是发给电机的指令（QoS 1 实时优先），status_ 是状态广播。
    output_ = create_publisher<ui::MotorSetpoint>("actuators/thrusters/setpoint", rclcpp::QoS(1));
    status_ = create_publisher<ui::AllocationStatus>("control/allocation", 10);

    // 订阅物理 wrench 与手动档位输入
// 订阅物理 wrench 输入；回调用 lambda 捕获 this，以便调用成员函数 wrench()。
    input_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
        "control/wrench_request",
        rclcpp::QoS(1),
        [this](geometry_msgs::msg::WrenchStamped::ConstSharedPtr w) {
          wrench(*w, false);
        });

// 手动档位输入走另一个话题，回调第二个参数 true 表示手动模式。
    manual_input_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
        "control/manual_request",
        rclcpp::QoS(1),
        [this](geometry_msgs::msg::WrenchStamped::ConstSharedPtr w) {
          wrench(*w, true);
        });

    // 订阅 MCU 网关状态
// 订阅网关状态，用于判断串口连接与控制权是否发生变化。
    gateway_sub_ = create_subscription<ui::GatewayStatus>(
        "gateway/status",
        10,
        [this](ui::GatewayStatus::ConstSharedPtr s) {
          // 串口物理连接断开或控制归属发生变更时强制解除武装
          if (gateway_ && (gateway_->connection_id != s->connection_id || gateway_->control_owner != s->control_owner)) {
            disable("serial connection or control authority changed");
          }
          gateway_ = *s;
          gateway_at_ = Clock::now();

          if (armed_ && (!s->connected || !s->tx_enabled)) {
            disable("gateway unavailable");
          }

          if (armed_ && s->motor_session_id == session_) {
            if (s->motor_rearm_required && acknowledged_) {
              disable("gateway requires rearm");
            } else if (!s->motor_rearm_required) {
              acknowledged_ = true;  // 零点握手确认完成
            }
          }
        });

    // 使能控制服务
// 使能服务：data=true 请求使能；data=false 表示操作手主动停车。
    enable_ = create_service<std_srvs::srv::SetBool>(
        "control/enable",
        [this](std_srvs::srv::SetBool::Request::SharedPtr req, std_srvs::srv::SetBool::Response::SharedPtr res) {
          if (!req->data) {
            disable("operator disabled");
            publish_command();
            res->success = true;
            res->message = "Disabled; stop requested";
            return;
          }

          if (!gateway_ready()) {
            res->success = false;
            res->message = "Gateway must be connected, settled, heartbeating and enable_motor_tx=true";
            return;
          }

          if (command_sources() > 1) {
            res->success = false;
            res->message = "More than one manual/wrench publisher";
            return;
          }

          // 生成全局唯一且单调递增的会话 ID
          session_ = std::max(
              session_ + 1,
              static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count()));
          sequence_ = 0;
          armed_ = true;
          acknowledged_ = false;
          received_after_enable_ = false;
          last_stamp_.reset();
          input_valid_ = false;
          enabled_at_ = Clock::now();
          enabled_stamp_ = now();
          detail_ = "enabled; zero handshake and fresh wrench required";

          publish_command();
          res->success = true;
          res->message = detail_;
        });

    // 20ms 看门狗与高频指令心跳定时器
// 20ms 周期定时器：既周期性重发指令，也充当看门狗检测超时。
    timer_ = create_wall_timer(20ms, [this] { tick(); });

    RCLCPP_INFO(get_logger(), "Allocator ready: actual URDF geometry in NED, confirmed motor polarity, provisional linear RPM curve, disarmed");
    RCLCPP_INFO(get_logger(), "Manual axes use configured signs and balanced startup; physical wrench remains NED");
    const auto &center = allocator_->wrench_origin();
    RCLCPP_INFO(get_logger(), "Wrench origin: thruster centroid [%.6f %.6f %.6f] m in base_link", center[0], center[1], center[2]);
  }

// 析构函数：进程退出前尽力停车（disable 并补发零指令），保证安全。
  ~ControlNode() override {
    try {
      disable("shutdown");
      if (rclcpp::ok()) {
        publish_command();
      }
    } catch (...) {}
  }

private:
  // 计算输入话题发布者总数
// command_sources：统计两个输入话题上的发布者总数，用于排他性检查。
  std::size_t command_sources() const {
    return count_publishers("control/wrench_request") + count_publishers("control/manual_request");
  }

  // 判定 MCU 网关是否处于稳定就绪状态 (心跳 < 2.5s，已完成串口稳定期)
// gateway_ready：网关是否“已连接且稳定且有心跳且允许发送且授权通过”。
  bool gateway_ready() const {
    return gateway_ &&
           (Clock::now() - gateway_at_ < 2500ms) &&
           gateway_->connected &&
           !gateway_->settling &&
           gateway_->tx_enabled &&
           (gateway_->heartbeat_age_ms >= 0) &&
           (gateway_->heartbeat_age_ms < 2500) &&
           source_authorized();
  }

  // 判定控制源排他授权合法性
// source_authorized：收集发布者信息后交给纯逻辑函数 authorized_source 判定。
  bool source_authorized() const {
    if (!gateway_) {
      return false;
    }
    const auto manual = get_publishers_info_by_topic("control/manual_request");
    const auto wrench = count_publishers("control/wrench_request");
    const bool remote_only = (manual.size() == 1) &&
                             (manual.front().node_name() == "remote_manual") &&
                             (manual.front().node_namespace() == get_namespace());
    return uc::remote::authorized_source(gateway_->control_owner, manual.size(), wrench, remote_only);
  }

  // 判定 wrench 输入是否处于新鲜期内
// fresh：输入是否仍在 input_timeout_ms 新鲜窗口内。
  bool fresh() const {
    return input_valid_ && (Clock::now() - input_at_ < timeout_);
  }

  // 安全解除武装
// disable：解除武装（armed_=false），此后只发零转速，直到再次使能。
  void disable(const std::string &reason) {
    armed_ = false;
    acknowledged_ = false;
    received_after_enable_ = false;
    detail_ = reason;
  }

  // 接收并解算 wrench 或手动档位输入
// wrench：两个输入话题共用的处理函数。manual=true 走手动档位，否则走物理 wrench。
// 先做严格校验（坐标系/时间戳/发布者数量/授权），任一不满足就停机。
  void wrench(const geometry_msgs::msg::WrenchStamped &w, bool manual) {
    const rclcpp::Time stamp(w.header.stamp);
    const auto age = (now() - stamp).nanoseconds();

    // 严密校验：坐标系、时钟容差 (未来 100ms / 过去 250ms)、单调递增性与排他性
    if (w.header.frame_id != frame_id_ ||
        age < -100000000LL ||
        age > 250000000LL ||
        (last_stamp_ && stamp <= *last_stamp_) ||
        command_sources() > 1 ||
        !source_authorized() ||
        (gateway_->control_owner == "GUI_MANUAL" && !manual)) {
      input_valid_ = false;
      disable("invalid frame, stale/duplicate timestamp or multiple publishers");
      publish_command();
      return;
    }

    last_stamp_ = stamp;
    desired_ = {
        w.wrench.force.x,
        w.wrench.force.y,
        w.wrench.force.z,
        w.wrench.torque.x,
        w.wrench.torque.y,
        w.wrench.torque.z};
    manual_mode_ = manual;

    // 分配求解
    solver_ok_ = manual ? allocator_->solve_manual(desired_, result_) : allocator_->solve(desired_, result_);
    if (solver_ok_) {
      desired_ = result_.effective_request;
    }
    input_valid_ = solver_ok_;
    input_at_ = Clock::now() - std::chrono::nanoseconds(std::max<std::int64_t>(0, age));

    if (!solver_ok_) {
      disable(manual ? "invalid manual request or balanced startup infeasible within RPM bounds"
                     : "invalid wrench or QP solve failed");
      publish_command();
      return;
    }

    if (armed_ && stamp >= enabled_stamp_) {
      received_after_enable_ = true;
    }
    if (!armed_) {
      detail_ = manual ? "manual preview; output is stopped" : "wrench preview; output is stopped";
    }
  }

  // 发布底层电机转速指令
// publish_command：组装并发布 MotorSetpoint。只有所有安全门限都满足时，
// 才会填入非零 RPM；任意一条不满足就发布零/未武装指令。
  void publish_command() {
    if (!gateway_) {
      return;
    }

    ui::MotorSetpoint m;
    m.header.stamp = now();
    m.header.frame_id = frame_id_;
    m.connection_id = gateway_->connection_id;
    m.session_id = session_;
    m.sequence = ++sequence_;
    m.node_mask = 0xff;
    m.armed = armed_;

    // 只有所有安全门限全部就绪时，才下发非零运转转速
    if (armed_ && acknowledged_ && received_after_enable_ && fresh() && gateway_ready()) {
      m.rpm = result_.rpm;
      for (std::size_t i = 0; i < 8; ++i) {
        if (m.rpm[i] != 0) {
          m.run_mask |= (1u << i);
        }
      }
      detail_ = m.run_mask ? (manual_mode_ ? "streaming manual throttle RPM" : "streaming allocated RPM")
                           : "all outputs inside deadband or zero";
    }
    output_->publish(m);
  }

  // 20ms 看门狗轮询与状态广播
// tick：定时器回调，先做看门狗检查，再发指令，最后每隔 100ms 广播一次状态。
  void tick() {
    if (armed_) {
      if (!gateway_ready()) {
        disable("gateway heartbeat/connection expired");
      } else if (!acknowledged_ && Clock::now() - enabled_at_ > 2500ms) {
        disable("zero handshake timed out");
      } else if (received_after_enable_ && !fresh()) {
        disable("wrench input timed out; enable again");
      } else if (command_sources() > 1) {
        disable("multiple command publishers");
      }
    }

    publish_command();

    // 每 5 个周期 (100ms) 发布一次状态
    if (++ticks_ % 5) {
      return;
    }

    ui::AllocationStatus s;
    s.header.stamp = now();
    s.header.frame_id = frame_id_;
    s.armed = armed_;
    s.input_fresh = fresh();
    s.gateway_ready = gateway_ready();
    s.solver_ok = solver_ok_;
    s.session_id = session_;
    s.requested_wrench = desired_;
    s.thruster_force_n = result_.thrust;
    s.allocated_wrench = result_.allocated;
    s.modeled_rpm_wrench = result_.modeled_rpm;
    s.proposed_motor_rpm = result_.rpm;
    s.rpm_deadband_applied = result_.deadband;
    s.rpm_saturated = result_.saturated;
    s.detail = detail_;
    status_->publish(s);
  }

// 以下为节点成员变量：求解器、最近输入、时间戳与各种安全标志。
// 注意 session_ 用开机以来的纳秒数初始化，保证会话 ID 单调且唯一。
  std::unique_ptr<uc::Allocator> allocator_;
  uc::Result result_;
  uc::Wrench desired_{};
  std::string frame_id_;
  std::string detail_ = "disarmed";
  std::chrono::milliseconds timeout_{500};

  std::optional<ui::GatewayStatus> gateway_;
  std::optional<rclcpp::Time> last_stamp_;
  rclcpp::Time enabled_stamp_{0, 0, RCL_ROS_TIME};

  Clock::time_point gateway_at_{};
  Clock::time_point input_at_{};
  Clock::time_point enabled_at_{};

  std::uint64_t session_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
                               Clock::now().time_since_epoch()).count();
  std::uint64_t sequence_ = 0;

  bool armed_ = false;
  bool acknowledged_ = false;
  bool received_after_enable_ = false;
  bool input_valid_ = false;
  bool solver_ok_ = false;
  bool manual_mode_ = false;

  unsigned ticks_ = 0;

  rclcpp::Publisher<ui::MotorSetpoint>::SharedPtr output_;
  rclcpp::Publisher<ui::AllocationStatus>::SharedPtr status_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr input_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr manual_input_;
  rclcpp::Subscription<ui::GatewayStatus>::SharedPtr gateway_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_;
  rclcpp::TimerBase::SharedPtr timer_;
};

// 程序入口：初始化 rclcpp，创建节点并 spin（进入事件循环），直到 Ctrl+C 退出。
// 构造节点时若参数非法会抛异常，这里捕获并打印后返回错误码。
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ControlNode>());
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("thrust_allocator"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
