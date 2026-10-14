// Real ROS nodes + POSIX pseudo terminal. Never opens the physical robot serial port.
// 验证内容：真实 ROS 2 节点 + POSIX 伪终端（pty）下的串口控制链路——空闲预览、
//   使能、输入超时停桨并解除武装、显式重新武装、串口重连、手动模式启动/归零/超时，
//   以及 manual 与 wrench 两路输入的互斥。
// 运行方式：bash tools/validate.sh allocation（等价于 ros2 run underwater_bringup serial_control_probe），
//   需先 source install/setup.bash 并已 colcon build。
// 通过标准：进程退出码 0，末尾打印 "PASS serial control integration"；失败打印 FAIL 并返回 1。
// 是否需要真实硬件：不需要。用 pty 伪造串口，真机串口不会被打开。
#include "underwater_mcu_gateway/protocol.hpp"
#include <array>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <fcntl.h>
#include <filesystem>
#include <future>
#include <iostream>
#include <pty.h>
#include <stdexcept>
#include <string>
#include <sys/wait.h>
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <underwater_interfaces/msg/gateway_status.hpp>
#include <underwater_interfaces/msg/allocation_status.hpp>
namespace wire=underwater::wire;
namespace ui=underwater_interfaces::msg;
using Clock=std::chrono::steady_clock;
using namespace std::chrono_literals;
void require(bool b, const char *reason) {
  if (!b) {
    throw std::runtime_error(reason);
  }
} // 断言失败即抛异常，由 main 捕获后返回 1
// 伪串口夹具：创建 pty 并用子进程拉起 manual.launch.xml，析构时结束子进程并清理临时目录。
struct Fixture {
  std::string directory,port,ns;
  int master=-1;
  pid_t child=-1;
  Fixture() {
    char temp[]="/tmp/underwater-allocator-XXXXXX";
    const char * p=mkdtemp(temp); require(p,"mkdtemp failed");
    directory=p; port=directory+"/serial"; ns="allocation_test_"+std::to_string(getpid());
    reconnect();
    setenv("ROS_DOMAIN_ID","190",1); setenv("ROS_LOCALHOST_ONLY","1",1); // 独占 DDS 域，避免与真机或其他测试互相发现
    std::filesystem::create_directories("log/allocation-serial-probe");
    child=fork(); require(child>=0,"fork failed");
    if(child==0) {
      setsid();
      int log=open("log/allocation-serial-probe/launch.log",O_WRONLY|O_CREAT|O_TRUNC,0600); // 子进程输出重定向到日志文件
      if(log>=0) { dup2(log,STDOUT_FILENO); dup2(log,STDERR_FILENO); close(log); }
      const std::string arg_port="serial_port:="+port,arg_ns="namespace:="+ns;
      // 用伪串口路径和独立命名空间启动 bringup；关相机与 TCP，只留电机发送。
      execlp("ros2","ros2","launch","underwater_bringup","manual.launch.xml",
        arg_port.c_str(),arg_ns.c_str(),"motor_tx:=true","camera_enabled:=false","tcp_port:=0",static_cast<char *>(nullptr));
      _exit(127);
    }
  }
  void disconnect() {
    if (master >= 0) {
      close(master);
      master = -1;
    }
  }
  void reconnect() {
    disconnect(); int slave=-1; char name[128]{};
    require(openpty(&master,&slave,name,nullptr,nullptr)==0,"openpty failed");
    close(slave); fcntl(master,F_SETFL,fcntl(master,F_GETFL)|O_NONBLOCK); // 主端设为非阻塞，step() 读串口永不挂起
    fcntl(master,F_SETFD,FD_CLOEXEC);
    unlink(port.c_str()); require(symlink(name,port.c_str())==0,"symlink failed"); // 用符号链接把 pty 路径伪装成串口设备
  }
  ~Fixture() {
    if(child>0) {
      kill(-child,SIGINT); // 先给整个进程组 SIGINT，最多等 5 秒（100 × 50ms）
      bool exited=false;
      for(int i=0;i<100;++i) {
        if(waitpid(child,nullptr,WNOHANG)==child) { exited=true; break; }
        usleep(50000);
      }
      if(!exited) { kill(-child,SIGKILL); waitpid(child,nullptr,0); } // 仍不退出则强杀兜底
    }
    disconnect(); unlink(port.c_str()); rmdir(directory.c_str());
  }
};
// 探针节点：发布控制请求、订阅 gateway/allocation 状态，并解析伪串口上收到的协议帧。
class Probe:public rclcpp::Node {
public:
  explicit Probe(Fixture & fixture):Node("serial_control_probe","/"+fixture.ns),fixture_(fixture) {
    pub_=create_publisher<geometry_msgs::msg::WrenchStamped>("control/wrench_request",rclcpp::QoS(1));
    client_=create_client<std_srvs::srv::SetBool>("control/enable");
    gateway_sub_=create_subscription<ui::GatewayStatus>("gateway/status",10,
      [this](ui::GatewayStatus::ConstSharedPtr m){ gateway=*m; gateway_seen=true; });
    allocation_sub_=create_subscription<ui::AllocationStatus>("control/allocation",10,
      [this](ui::AllocationStatus::ConstSharedPtr m){ allocation=*m; allocation_seen=true; });
  }
  // 单步推进：spin 一次、抽干伪串口数据、按周期补发心跳与 wrench。
  void step(bool wrench) {
    rclcpp::spin_some(shared_from_this());
    const auto now=Clock::now();
    if(fixture_.master>=0) {
      std::uint8_t data[4096];
      for(;;) {
        auto n=read(fixture_.master,data,sizeof(data));
        if(n<=0) break;
        for(const auto & f:parser_.feed(wire::Bytes(data,data+n),milliseconds())) {
          require(f.family==0x55 && f.can_id==0x100 && f.flags==2 && !f.timestamp_valid,"bad CAN envelope"); // 0x55 家族 + CAN ID 0x100 + 无时间戳
          require(f.payload.size()==24 && f.payload[0]==1 && f.payload[2]==255,"bad motor payload"); // 电机帧固定 24 字节载荷
          ++tx_frames; tx_seen_=true;
          if(f.payload[1]==0x10) { // 0x10 = 转速（RPM）帧
            for(int i=0;i<8;++i) {
              const auto raw=wire::u16(f.payload,8+2*i); int rpm=raw<0x8000?int(raw):int(raw)-65536; // 小端 u16 转有符号
              require(rpm==0 || (std::abs(rpm)>=1000 && std::abs(rpm)<=6000),"RPM outside allowed range"); // 非零 RPM 必须落在 1000..6000
              last_rpm[i]=rpm;
            }
          } else {
            require(f.payload[1]==0x11,"unexpected motor command"); // 0x11 = 使能/停止帧
            last_mask=f.payload[3];
            if(last_mask) { require(last_mask==0x0f || last_mask==0xf0 || last_mask==0xff,"request enabled wrong nodes"); ++nonzero_runs; } // 只允许水平/垂直/全节点三种位图
            else ++stop_runs;
          }
        }
      }
      // Start input only after observing the startup stop (the slave is now in raw mode).
      if(tx_seen_ && now-last_heartbeat_>=100ms) { // 心跳周期 100ms，低于网关看门狗阈值
        auto b=heartbeat(); const auto n=write(fixture_.master,b.data(),b.size());
        require(n==static_cast<ssize_t>(b.size()) || (n<0 && (errno==EAGAIN||errno==EIO)),"pseudo heartbeat write failed"); // pty 缓冲满或已关闭属正常
        last_heartbeat_=now;
      }
    }
    if(wrench && now-last_wrench_>=50ms) { // wrench 以 20Hz 发布，需快于控制超时
      geometry_msgs::msg::WrenchStamped w; w.header.stamp=this->now(); w.header.frame_id="thruster_center";
      if(manual_mode && !zero_input) {
        w.wrench.force.x=manual_levels[0];w.wrench.force.y=manual_levels[1];w.wrench.force.z=manual_levels[2];
        w.wrench.torque.x=manual_levels[3];w.wrench.torque.y=manual_levels[4];w.wrench.torque.z=manual_levels[5];
      }
      else if(!manual_mode)w.wrench.force.z=500; // Actual URDF geometry: all four vertical nodes clear RPM deadband.
      pub_->publish(w); last_wrench_=now;
    }
    rclcpp::sleep_for(5ms);
  }
  void run_for(double seconds,bool wrench=false) {
    auto start=Clock::now(); while(std::chrono::duration<double>(Clock::now()-start).count()<seconds) step(wrench);
  }
  // 等待 gateway 报告已连接、不再 settling、tx 使能且使能服务就绪，超时 15 秒则失败。
  void wait_ready() {
    auto deadline=Clock::now()+15s;
    while(Clock::now()<deadline) {
      step(false);
      if(gateway_seen && gateway.connected && !gateway.settling && gateway.tx_enabled &&
         gateway.heartbeat_age_ms>=0 && client_->service_is_ready()) return;
    }
    throw std::runtime_error("ROS gateway/control readiness timed out");
  }
  // 调用 control/enable 服务武装控制器，等待响应最多 3 秒。
  void enable() {
    auto request=std::make_shared<std_srvs::srv::SetBool::Request>(); request->data=true;
    auto future=client_->async_send_request(request);
    auto deadline=Clock::now()+3s;
    while(future.wait_for(0ms)!=std::future_status::ready && Clock::now()<deadline) step(false);
    require(future.wait_for(0ms)==std::future_status::ready,"enable service timeout");
    auto response=future.get(); if(!response->success) throw std::runtime_error(response->message);
  }
  void reset_rx() { parser_.reset(); tx_seen_=false; } // 重连后清空解析器与首帧标志，重新开始统计
  // 切到手动输入话题：先释放 wrench 发布者，避免两路输入同时在线。
  void use_manual() {
    pub_.reset();run_for(0.5,false);
    pub_=create_publisher<geometry_msgs::msg::WrenchStamped>("control/manual_request",rclcpp::QoS(1));
    manual_mode=true;expected_mask=0x0f;
  }
  // 只在一个轴上给 ±1.0 电平，并给出该轴启动时应当参与的电机节点位图。
  void set_manual_axis(int axis,int sign) {
    manual_levels.fill(0);manual_levels[axis]=sign;
    expected_mask=(axis<2 || axis==5)?0x0f:0xf0;
  }
  void add_conflicting_source() {
    conflict_=create_publisher<geometry_msgs::msg::WrenchStamped>("control/wrench_request",rclcpp::QoS(1));
  }
  bool manual_mode=false,zero_input=false;
  std::array<double,6> manual_levels{1,0,0,0,0,0};
  unsigned expected_mask=0xf0;
  std::uint64_t tx_frames=0,nonzero_runs=0,stop_runs=0;
  unsigned last_mask=0;
  std::array<int,8> last_rpm{};
  ui::GatewayStatus gateway; ui::AllocationStatus allocation;
  bool gateway_seen=false,allocation_seen=false;
private:
  std::uint64_t milliseconds() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now().time_since_epoch()).count();
  }
  // 按 AA58 帧格式构造网关心跳：帧头 AA58、小端序号、微秒时间戳、CRC16 与帧尾 58AA。
  wire::Bytes heartbeat() {
    wire::Bytes b{0xaa,0x58,1,1,0,0};
    const auto put32=[&](std::uint32_t n){for(int i=0;i<4;++i)b.push_back((n>>(8*i))&255);};
    put32(heartbeat_seq_++); b.push_back(5); b.push_back(0); put32(milliseconds()*1000);
    b.insert(b.end(),5,0); auto c=wire::crc16(b.data()+1,b.size()-1);
    b.push_back(c&255); b.push_back(c>>8); b.push_back(0x58); b.push_back(0xaa); return b;
  }
  Fixture & fixture_; wire::Parser parser_;
  bool tx_seen_=false; std::uint32_t heartbeat_seq_=0;
  Clock::time_point last_heartbeat_{},last_wrench_{};
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr conflict_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_;
  rclcpp::Subscription<ui::GatewayStatus>::SharedPtr gateway_sub_;
  rclcpp::Subscription<ui::AllocationStatus>::SharedPtr allocation_sub_;
};
// 用例主体：按阶段顺序驱动夹具与探针；任一 require 失败立即进入 catch。
int main(int argc,char ** argv) {
  try {
    Fixture fixture; rclcpp::init(argc,argv); auto node=std::make_shared<Probe>(fixture);
    node->wait_ready();
    // 阶段 1：未使能时只发布预览，任何电机输出都必须为零。
    node->run_for(1.5,true);
    require(node->allocation_seen && node->allocation.solver_ok,"allocation preview absent");
    require(node->nonzero_runs==0,"unarmed command moved a motor");
    // 阶段 2：使能后 Fz=500N 应到达串口，只有后垂直电机反向转。
    node->enable(); node->run_for(2,true);
    require(node->nonzero_runs>10 && node->last_mask==0xf0,"enabled Fz did not reach serial"); // 0xf0 = 仅后四个垂直节点
    for(int i=0;i<4;++i) require(node->last_rpm[i]==0,"horizontal RPM must be zero");
    for(int i=4;i<6;++i) require(node->last_rpm[i]>=1000,"front vertical positive RPM missing"); // 前垂直取正转
    for(int i=6;i<8;++i) require(node->last_rpm[i]<=-1000,"rear vertical negative RPM missing"); // 后垂直取反转
    std::cout<<"Motor RPM:"; for(auto rpm:node->last_rpm) std::cout<<" "<<rpm; std::cout<<"\n";
    node->run_for(1.2,false);
    // 阶段 3：停止输入 1.2 秒（超过输入超时）后必须停桨并解除武装。
    require(node->last_mask==0 && !node->allocation.armed,"input timeout did not stop and disarm");
    auto prior=node->nonzero_runs; node->run_for(1,true);
    // 阶段 4：超时失效的旧指令不得自行恢复，必须显式重新使能。
    require(node->nonzero_runs==prior,"stale authority resumed without rearm");
    node->enable(); node->run_for(1,true); require(node->nonzero_runs>prior,"explicit rearm failed");
    const auto connection=node->gateway.connection_id;
    fixture.disconnect(); node->run_for(0.6,true); fixture.reconnect(); node->reset_rx(); // 断开 0.6 秒后重建 pty
    prior=node->nonzero_runs; node->run_for(3.5,true);
    require(node->gateway.connected && node->gateway.connection_id!=connection,"serial reconnect failed"); // connection_id 变化证明是新会话
    require(node->nonzero_runs==prior && node->last_mask==0 && !node->allocation.armed,"reconnect resumed an old command");
    // 阶段 5：手动模式自身的预览不应改变电机输出。
    node->use_manual();node->run_for(1.5,true);
    require(node->allocation.solver_ok && node->nonzero_runs==prior,"manual preview changed motor output");
    // 阶段 6：手动 +Fx 启动水平节点，极性与推进器实际布置一致。
    node->enable();node->run_for(1.5,true);
    require(node->last_mask==0x0f,"first manual step did not start horizontal nodes"); // 0x0f = 四个水平节点
    for(int i=0;i<4;++i)require(std::abs(node->last_rpm[i])>=1000 && std::abs(node->last_rpm[i])<=6000,"manual startup RPM wrong");
    for(int i=4;i<8;++i)require(node->last_rpm[i]==0,"horizontal translation needs no vertical compensation at the centroid");
    require(node->last_rpm[0]>0 && node->last_rpm[1]<0 && node->last_rpm[2]>0 && node->last_rpm[3]<0,
      "manual +Fx positive-RPM polarity does not match actual thrusters");
    require(node->allocation.requested_wrench[0]<0 && node->allocation.modeled_rpm_wrench[0]<0,
      "manual +Fx did not reverse the effective NED request and model output");
    require(std::abs(node->allocation.modeled_rpm_wrench[4])<0.1,"manual Fx startup produced unintended pitch");
    std::cout<<"Manual Fx level 1 RPM:";for(auto rpm:node->last_rpm)std::cout<<" "<<rpm;std::cout<<"\n";
    // 阶段 7：逐轴逐符号核对电机符号表，以及质心单轴首步的标称启动转速。
    const std::array<int,6> axis_signs{-1,1,1,-1,1,-1};
    const std::array<std::array<int,8>,6> motor_signs{{
      {1,-1,1,-1,0,0,0,0},{-1,-1,1,1,0,0,0,0},{0,0,0,0,1,1,-1,-1},
      {0,0,0,0,1,-1,-1,1},{0,0,0,0,-1,-1,-1,-1},{1,1,1,1,0,0,0,0}}};
    for(int axis=0;axis<6;++axis)for(int sign:{-1,1}) {
      node->set_manual_axis(axis,sign);node->run_for(0.5,true);
      require(node->last_mask==node->expected_mask,"single-axis startup lost required nodes");
      require(node->allocation.solver_ok && !node->allocation.rpm_deadband_applied,"single-axis startup lost compensation to deadband");
      require(axis_signs[axis]*sign*node->allocation.modeled_rpm_wrench[axis]>0,"manual axis direction wrong");
      for(int j=0;j<6;++j)if(j!=axis)
        require(std::abs(node->allocation.modeled_rpm_wrench[j])<(j<3?0.5:0.1),"manual startup produced unintended force or torque"); // 力 0.5N / 力矩 0.1N·m 以内视为无串扰
      for(int i=0;i<8;++i)if(motor_signs[axis][i])
        require(sign*motor_signs[axis][i]*node->last_rpm[i]>0,"manual motor sign disagrees with operator table");
      for(int i=0;i<8;++i) {
        if(motor_signs[axis][i])require(std::abs(node->last_rpm[i])==1200,"centroid single-axis first step must reach nominal startup RPM"); // 单轴首步标称 1200 RPM
        else require(node->last_rpm[i]==0,"single-axis startup enabled an unrelated motor");
      }
      std::cout<<"Serial manual axis="<<axis<<" sign="<<sign<<" RPM:";
      for(auto rpm:node->last_rpm)std::cout<<" "<<rpm;std::cout<<"\n";
    }
    // 阶段 8：手动归零、超时解除武装，以及 manual 与 wrench 两路输入互斥。
    node->zero_input=true;node->run_for(0.5,true);
    require(node->last_mask==0,"manual zero did not stop");
    node->zero_input=false;node->run_for(0.5,true);node->run_for(1.2,false);
    require(node->last_mask==0&&!node->allocation.armed,"manual timeout did not disarm");
    node->enable();node->run_for(0.8,true);node->add_conflicting_source();node->run_for(0.8,true);
    require(node->last_mask==0&&!node->allocation.armed,"manual and wrench sources were not mutually exclusive");
    std::cout<<"PASS serial control integration: tx_frames="<<node->tx_frames
      <<" stop_runs="<<node->stop_runs<<" nonzero_run_frames="<<node->nonzero_runs
      <<"; preview, enable, timeout, rearm, reconnect, manual startup/zero/timeout/source exclusion\n";
    node.reset(); rclcpp::shutdown(); return 0; // 退出码 0 = 全部通过
  } catch(const std::exception & e) {
    std::cerr<<"FAIL serial control integration: "<<e.what()<<"\n";
    if(rclcpp::ok()) rclcpp::shutdown(); return 1; // 退出码 1 = 断言或初始化失败
  }
}
