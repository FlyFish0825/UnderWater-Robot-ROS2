// ==============================================================================
// src/sensors_cpp/src/manual.cpp
// ==============================================================================
// 手动控制与实验激励入口实现：
// - manual_control: 键盘交互档位输入，发布到 <ns>/control/manual_request；
// - forces_pub: 预设时变物理力曲线激励，发布到 <ns>/control/wrench_request；
// - preview=true 模式下只观察推力分配，不发起底层使能请求；
// - 退出时先发全零指令再下发 disable，防止后端电机残余运转。
// ==============================================================================
#include "sensors_cpp/entry.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <thread>

#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <msg_pkg/msg/forces.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <underwater_interfaces/msg/allocation_status.hpp>

namespace compatibility {

using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;
using Enable = std_srvs::srv::SetBool;

// RAII 终端设置：配置非规范模式并关闭回显，析构自动恢复
struct Terminal {
  termios old{};
  bool changed = false;

  explicit Terminal(bool needed) {
    if (!needed) {
      return;
    }
    if (!isatty(STDIN_FILENO) || tcgetattr(STDIN_FILENO, &old) != 0) {
      throw std::runtime_error("manual_control needs an interactive terminal");
    }
    auto cfg = old;
    cfg.c_lflag &= ~(ICANON | ECHO);
    cfg.c_cc[VMIN] = 0;
    cfg.c_cc[VTIME] = 0;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &cfg) != 0) {
      throw std::runtime_error("Cannot configure keyboard");
    }
    changed = true;
  }

  ~Terminal() {
    if (changed) {
      tcsetattr(STDIN_FILENO, TCSANOW, &old);
    }
  }
};

int manual(const std::string &entry, const std::shared_ptr<rclcpp::Node> &node) {
  const bool excitation = (entry == "forces_pub");
  const bool preview = node->get_parameter("preview").as_bool();
  const auto ns = backend_namespace(node);

  // 给予 DDS 发现期 1 秒宽限，避免误判控制源
  for (int i = 0; i < 10 && running(); ++i) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(100ms);
  }

  if (node->count_publishers(ns + "/control/manual_request") ||
      node->count_publishers(ns + "/control/wrench_request")) {
    throw std::runtime_error("Another wrench source is already running; no control takeover was attempted");
  }

  Terminal terminal(!excitation);
  auto pub = node->create_publisher<geometry_msgs::msg::WrenchStamped>(
      ns + (excitation ? "/control/wrench_request" : "/control/manual_request"),
      rclcpp::QoS(1));

  auto force_pub = node->create_publisher<msg_pkg::msg::Forces>("/forces", 10);
  auto enable = node->create_client<Enable>(ns + "/control/enable");

  std::array<double, 6> desired{};
  bool active = false;
  bool seen_armed = false;
  bool wanted = excitation;
  bool quit = false;
  std::optional<rclcpp::Client<Enable>::SharedFuture> pending;

  auto state = node->create_subscription<underwater_interfaces::msg::AllocationStatus>(
      ns + "/control/allocation",
      10,
      [&](underwater_interfaces::msg::AllocationStatus::ConstSharedPtr m) {
        if (!excitation && m->input_fresh && m->solver_ok) {
          msg_pkg::msg::Forces f;
          f.header = m->header;
          f.fx = m->requested_wrench[0];
          f.fy = m->requested_wrench[1];
          f.fz = m->requested_wrench[2];
          f.tyaw = m->requested_wrench[5];
          force_pub->publish(f);

          const auto &r = m->proposed_motor_rpm;
          RCLCPP_INFO_THROTTLE(
              node->get_logger(),
              *node->get_clock(),
              400,
              "Proposed RPM: [%d %d %d %d %d %d %d %d] armed=%d",
              static_cast<int>(r[0]),
              static_cast<int>(r[1]),
              static_cast<int>(r[2]),
              static_cast<int>(r[3]),
              static_cast<int>(r[4]),
              static_cast<int>(r[5]),
              static_cast<int>(r[6]),
              static_cast<int>(r[7]),
              m->armed);
        }

        if (preview || !active) {
          return;
        }

        if (m->armed) {
          seen_armed = true;
        } else if (seen_armed) {
          active = false;
          seen_armed = false;
          desired.fill(0);
          RCLCPP_WARN(
              node->get_logger(),
              "Control disarmed: %s. Press a direction key to request a new session.",
              m->detail.c_str());
        }
      });

  auto send = [&](const std::array<double, 6> &v) {
    geometry_msgs::msg::WrenchStamped w;
    w.header.stamp = node->now();
    w.header.frame_id = "thruster_center";
    w.wrench.force.x = v[0];
    w.wrench.force.y = v[1];
    w.wrench.force.z = v[2];
    w.wrench.torque.x = v[3];
    w.wrench.torque.y = v[4];
    w.wrench.torque.z = v[5];
    pub->publish(w);

    if (excitation) {
      msg_pkg::msg::Forces f;
      f.header = w.header;
      f.fx = v[0];
      f.fy = v[1];
      f.fz = v[2];
      f.tyaw = v[5];
      force_pub->publish(f);
    }
  };

  const auto start = Clock::now();
  double duration = node->get_parameter("duration_s").as_double();
  if (!std::isfinite(duration) || duration < 0) {
    throw std::invalid_argument("duration_s must be nonnegative");
  }
  if (excitation && duration == 0) {
    duration = 100;
  }

  RCLCPP_INFO(
      node->get_logger(),
      "Mode: %s. W/S Fx, A/D yaw, I/J Fz, T/F Fy, Y/G roll, U/H pitch; Q exits.",
      preview ? "PREVIEW (no enable request)" : "CONTROL (first direction key requests enable)");

  while (running() && !quit &&
         (duration == 0 || std::chrono::duration<double>(Clock::now() - start).count() < duration)) {
    rclcpp::spin_some(node);

    if (!excitation) {
      pollfd input{STDIN_FILENO, POLLIN, 0};
      if (poll(&input, 1, 0) > 0) {
        if (input.revents & (POLLHUP | POLLERR | POLLNVAL)) {
          quit = true;
          break;
        }
        char c = 0;
        if (read(STDIN_FILENO, &c, 1) == 1) {
          bool direction = true;
          switch (std::toupper(static_cast<unsigned char>(c))) {
            case 'W': desired[0] += 1; break;
            case 'S': desired[0] -= 1; break;
            case 'A': desired[5] += 1; break;
            case 'D': desired[5] -= 1; break;
            case 'I': desired[2] += 1; break;
            case 'J': desired[2] -= 1; break;
            case 'T': desired[1] += 1; break;
            case 'F': desired[1] -= 1; break;
            case 'Y': desired[3] += 1; break;
            case 'G': desired[3] -= 1; break;
            case 'U': desired[4] += 1; break;
            case 'H': desired[4] -= 1; break;
            case 'Q': quit = true; direction = false; break;
            default:
              direction = false;
              if (c >= '0' && c <= '7') {
                RCLCPP_WARN(node->get_logger(), "Mode key %c is unavailable; no MCU flag sent.", c);
              }
              break;
          }
          for (auto &v : desired) {
            v = std::clamp(v, -100.0, 100.0);
          }
          if (direction && !active && !pending) {
            wanted = true;
          }
        }
      }
    } else {
      const double t = std::chrono::duration<double>(Clock::now() - start).count();
      desired[0] = (t < 25) ? 10 * std::sin(0.5 * t)
                            : (t < 50) ? 2.0
                            : (t < 75) ? 2.0 + 0.2 * (t - 40)
                            : (t < 100) ? -3.0
                            : 0.0;
    }

    if (quit) {
      break;
    }

    if (wanted && !active && !pending) {
      wanted = false;
      if (preview) {
        active = true;
      } else if (enable->service_is_ready()) {
        auto req = std::make_shared<Enable::Request>();
        req->data = true;
        pending = enable->async_send_request(req).future.share();
      } else {
        if (excitation) {
          RCLCPP_ERROR(node->get_logger(), "Enable service is unavailable");
          return 2;
        }
        desired.fill(0);
        RCLCPP_WARN(node->get_logger(), "Enable service is unavailable; wait for backend and press a direction key again");
      }
    }

    if (pending && pending->wait_for(0ms) == std::future_status::ready) {
      const auto response = pending->get();
      pending.reset();
      active = response->success;
      seen_armed = false;
      if (!active) {
        desired.fill(0);
        RCLCPP_ERROR(node->get_logger(), "Enable rejected: %s", response->message.c_str());
        if (excitation) {
          return 2;
        }
      } else {
        RCLCPP_INFO(node->get_logger(), "Control session ready");
      }
    }

    if (active) {
      send(desired);
      RCLCPP_INFO_THROTTLE(
          node->get_logger(),
          *node->get_clock(),
          200,
          "%s Fx=%.1f Fy=%.1f Fz=%.1f K=%.1f M=%.1f N=%.1f",
          excitation ? "Wrench" : "Manual levels",
          desired[0], desired[1], desired[2], desired[3], desired[4], desired[5]);
    }

    std::this_thread::sleep_for(20ms);
  }

  // 退出清理
  if (active || pending) {
    send({});
    if (!preview && rclcpp::ok() && enable->service_is_ready()) {
      auto req = std::make_shared<Enable::Request>();
      req->data = false;
      auto future = enable->async_send_request(req);
      if (rclcpp::spin_until_future_complete(node, future, 1000ms) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_WARN(node->get_logger(), "Disable response unavailable; gateway command timeout remains in effect");
      }
    }
  }

  RCLCPP_INFO(node->get_logger(), "Control entry closed");
  return 0;
}

}  // namespace compatibility
