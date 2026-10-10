// 手动控制与实验激励入口：manual_control（键盘档位，发 <ns>/control/manual_request）与
// forces_pub（按预设时间曲线输出物理力，发 <ns>/control/wrench_request）。
// 方向键/激励启动时会请求 <ns>/control/enable；preview=true 时只观察，不发任何 enable 请求。
// 退出时先发空 wrench 再请求 disable，避免后端残留 armed 状态；激励模式无 enable 服务返回 2。
#include "sensors_cpp/entry.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <underwater_interfaces/msg/allocation_status.hpp>
#include <msg_pkg/msg/forces.hpp>
#include <termios.h>
#include <poll.h>
#include <unistd.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cctype>
#include <iostream>
#include <optional>
#include <thread>
namespace compatibility {
using namespace std::chrono_literals;
using Clock=std::chrono::steady_clock;
using Enable=std_srvs::srv::SetBool;
// RAII 终端设置：关闭 ICANON/ECHO 并设为非阻塞读，析构时恢复原 termios。
struct Terminal {
  termios old{};bool changed=false;
  explicit Terminal(bool needed) {
    if(!needed)return;
    if(!isatty(STDIN_FILENO)||tcgetattr(STDIN_FILENO,&old)!=0)throw std::runtime_error("manual_control needs an interactive terminal");
    auto cfg=old;cfg.c_lflag&=~(ICANON|ECHO);cfg.c_cc[VMIN]=0;cfg.c_cc[VTIME]=0;
    if(tcsetattr(STDIN_FILENO,TCSANOW,&cfg)!=0)throw std::runtime_error("Cannot configure keyboard");
    changed=true;
  }
  ~Terminal(){if(changed)tcsetattr(STDIN_FILENO,TCSANOW,&old);}
};
// 控制主循环：读取键盘档位或生成激励曲线，必要时请求 enable，然后周期下发 wrench。
int manual(const std::string &entry,const std::shared_ptr<rclcpp::Node> &node) {
  const bool excitation=entry=="forces_pub"; // forces_pub 为激励模式：无需终端，按时间曲线输出
  const bool preview=node->get_parameter("preview").as_bool(); // true=只观察分配结果，不请求 enable
  const auto ns=backend_namespace(node);
  // Allow DDS discovery before registering another command publisher.
  for(int i=0;i<10&&running();++i){rclcpp::spin_some(node);std::this_thread::sleep_for(100ms);} // 等约 1s DDS 发现，避免误判已有 wrench 源
  if(node->count_publishers(ns+"/control/wrench_request")+node->count_publishers(ns+"/control/manual_request")>0)
    throw std::runtime_error("Another wrench source is already running; no control takeover was attempted"); // 已有控制源则拒绝，不抢占
  Terminal terminal(!excitation); // 激励模式不需要键盘
  auto pub=node->create_publisher<geometry_msgs::msg::WrenchStamped>(ns+(excitation?"/control/wrench_request":"/control/manual_request"),rclcpp::QoS(1)); // 激励/手动走不同话题
  auto force_pub=node->create_publisher<msg_pkg::msg::Forces>("/forces",10); // 兼容的 /forces 话题
  auto enable=node->create_client<Enable>(ns+"/control/enable");
  std::array<double,6> desired{};
  bool active=false,seen_armed=false,wanted=excitation,quit=false; // 激励模式启动即请求 enable
  std::optional<rclcpp::Client<Enable>::SharedFuture> pending;
  auto state=node->create_subscription<underwater_interfaces::msg::AllocationStatus>(ns+"/control/allocation",10,
    [&](underwater_interfaces::msg::AllocationStatus::ConstSharedPtr m){
      if(!excitation && m->input_fresh && m->solver_ok) { // 手动模式回显后端分配结果与建议转速
        msg_pkg::msg::Forces f;f.header=m->header;f.fx=m->requested_wrench[0];f.fy=m->requested_wrench[1];
        f.fz=m->requested_wrench[2];f.tyaw=m->requested_wrench[5];force_pub->publish(f);
        const auto &r=m->proposed_motor_rpm;
        RCLCPP_INFO_THROTTLE(node->get_logger(),*node->get_clock(),400,
          "Proposed RPM: [%d %d %d %d %d %d %d %d] armed=%d",
          int(r[0]),int(r[1]),int(r[2]),int(r[3]),int(r[4]),int(r[5]),int(r[6]),int(r[7]),m->armed);
      }
      if(preview||!active)return; // preview 或未激活时不跟踪 armed 状态
      if(m->armed)seen_armed=true;
      else if(seen_armed){active=false;seen_armed=false;desired.fill(0);RCLCPP_WARN(node->get_logger(),"Control disarmed: %s. Press a direction key to request a new session.",m->detail.c_str());} // armed→disarmed 视为会话结束，清档位并要求重新按键
    });
  auto send=[&](const std::array<double,6> &v) { // 下发一帧 wrench（顺序 Fx,Fy,Fz,K,M,N），激励模式同时发 /forces
    geometry_msgs::msg::WrenchStamped w;w.header.stamp=node->now();w.header.frame_id="thruster_center";
    w.wrench.force.x=v[0];w.wrench.force.y=v[1];w.wrench.force.z=v[2];
    w.wrench.torque.x=v[3];w.wrench.torque.y=v[4];w.wrench.torque.z=v[5];pub->publish(w);
    if(excitation){msg_pkg::msg::Forces f;f.header=w.header;f.fx=v[0];f.fy=v[1];f.fz=v[2];f.tyaw=v[5];force_pub->publish(f);}
  };
  const auto start=Clock::now();
  double duration=node->get_parameter("duration_s").as_double();
  if(!std::isfinite(duration)||duration<0)throw std::invalid_argument("duration_s must be nonnegative");
  if(excitation&&duration==0)duration=100; // 激励模式默认时长 100 秒；手动模式 0 表示不限时
  RCLCPP_INFO(node->get_logger(),"Mode: %s. W/S Fx, A/D yaw, I/J Fz, T/F Fy, Y/G roll, U/H pitch; Q exits.",preview?"PREVIEW (no enable request)":"CONTROL (first direction key requests enable)");
  RCLCPP_INFO(node->get_logger(),"Keys 0..7 are not MCU mode commands. Keyboard values are levels: 0=stop, 1=start, 100=maximum; forces_pub retains physical-wrench units.");
  while(running()&&!quit&&(duration==0||std::chrono::duration<double>(Clock::now()-start).count()<duration)) { // duration_s=0 表示不限时
    rclcpp::spin_some(node);
    if(!excitation) {
      pollfd input{STDIN_FILENO,POLLIN,0};
      if(poll(&input,1,0)>0) {
        if(input.revents&(POLLHUP|POLLERR|POLLNVAL)){quit=true;break;}
        char c=0;
        if(read(STDIN_FILENO,&c,1)==1) {
          bool direction=true;
          switch(std::toupper(static_cast<unsigned char>(c))) {
            case 'W':desired[0]+=1;break;case 'S':desired[0]-=1;break;
            case 'A':desired[5]+=1;break;case 'D':desired[5]-=1;break;
            case 'I':desired[2]+=1;break;case 'J':desired[2]-=1;break;
            case 'T':desired[1]+=1;break;case 'F':desired[1]-=1;break;
            case 'Y':desired[3]+=1;break;case 'G':desired[3]-=1;break;
            case 'U':desired[4]+=1;break;case 'H':desired[4]-=1;break;
            case 'Q':quit=true;direction=false;break;
            default:direction=false;if(c>='0'&&c<='7')RCLCPP_WARN(node->get_logger(),"Mode key %c is unavailable; no MCU flag sent.",c);break;
          }
          for(auto &v:desired)v=std::clamp(v,-100.0,100.0); // 档位限幅 [-100,100]
          if(direction&&!active&&!pending)wanted=true; // 有新方向输入且未激活时，请求一次 enable
        }
      }
    } else {
      const double t=std::chrono::duration<double>(Clock::now()-start).count();
      desired[0]=t<25?10*std::sin(0.5*t):t<50?2:t<75?2+0.2*(t-40):t<100?-3:0; // 激励曲线：0-25s 正弦、25-50s 常值、50-75s 斜坡、75-100s -3
    }
    if(quit)break;
    if(wanted&&!active&&!pending) {
      wanted=false;
      if(preview)active=true; // preview 视为已激活，但不真正请求 enable
      else if(enable->service_is_ready()) {
        auto req=std::make_shared<Enable::Request>();req->data=true;
        pending=enable->async_send_request(req).future.share();
      } else {
        if(excitation){RCLCPP_ERROR(node->get_logger(),"Enable service is unavailable");return 2;} // 激励模式缺 enable 服务属致命错误，退出码 2
        desired.fill(0);RCLCPP_WARN(node->get_logger(),"Enable service is unavailable; wait for backend and press a direction key again");
      }
    }
    if(pending&&pending->wait_for(0ms)==std::future_status::ready) { // 非阻塞检查异步 enable 响应
      const auto response=pending->get();pending.reset();active=response->success;seen_armed=false;
      if(!active){desired.fill(0);RCLCPP_ERROR(node->get_logger(),"Enable rejected: %s",response->message.c_str());if(excitation)return 2;}
      else RCLCPP_INFO(node->get_logger(),"Control session ready");
    }
    if(active) {
      send(desired);
      RCLCPP_INFO_THROTTLE(node->get_logger(),*node->get_clock(),200,"%s Fx=%.1f Fy=%.1f Fz=%.1f K=%.1f M=%.1f N=%.1f",excitation?"Wrench":"Manual levels",desired[0],desired[1],desired[2],desired[3],desired[4],desired[5]);
    }
    std::this_thread::sleep_for(20ms); // 约 50 Hz 控制节拍
  }
  // A pending enable is followed by disable on the same service; never leave it armed on exit.
  if(active||pending) { // 退出清理：先发空 wrench，再（非 preview）请求 disable
    send({});
    if(!preview&&rclcpp::ok()&&enable->service_is_ready()) {
      auto req=std::make_shared<Enable::Request>();req->data=false;
      auto future=enable->async_send_request(req);
      if(rclcpp::spin_until_future_complete(node,future,1000ms)!=rclcpp::FutureReturnCode::SUCCESS) // 最多等 1s；不确定时依赖 gateway 自身的命令超时
        RCLCPP_WARN(node->get_logger(),"Disable response unavailable; gateway command timeout remains in effect");
    }
  }
  RCLCPP_INFO(node->get_logger(),"Control entry closed");
  return 0;
}
}
