// 验证内容：跨包接口兼容性契约——imu_py/depth_py 与 sensors_cpp 之间的
//   underwater_interfaces ↔ msg_pkg 字段映射、原始与标定 IMU 双路数据、
//   CSV 记录落盘、以及 manual_control 的按键/预览/退出行为。
// 运行方式：ros2 run underwater_bringup compat_probe（需先 source install/setup.bash）。
// 通过标准：退出码 0，且末行输出 result=PASS；末尾同时打印各子进程 exit 与证据目录。
// 是否需要真实硬件：不需要。用本进程合成的 ROS 话题 + 伪终端（forkpty）模拟键盘输入，
//   子进程全部指向 /compat_fixture 命名空间，不接触真机串口与网络。
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <underwater_interfaces/msg/imu_state.hpp>
#include <underwater_interfaces/msg/depth_state.hpp>
#include <underwater_interfaces/msg/allocation_status.hpp>
#include <msg_pkg/msg/imu_data.hpp>
#include <msg_pkg/msg/depth_sensor.hpp>
#include <sys/wait.h>
#include <pty.h>
#include <signal.h>
#include <fcntl.h>
#include <unistd.h>
#include <chrono>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <thread>
#include <vector>
using namespace std::chrono_literals;
using Clock=std::chrono::steady_clock;
struct Child{pid_t pid;int terminal;std::string label;};
// 拉起一个被测节点：auto_start:=false 让节点等外部输入，tty=true 时用 forkpty 以便模拟按键。
Child launch(const std::string &pkg,const std::string &name,const std::vector<std::string> &params,bool tty=false) {
  std::vector<std::string> args{"ros2","run",pkg,name,"--ros-args","-p","auto_start:=false","-p","robot_namespace:=/compat_fixture"};
  for (const auto &p : params) {
    args.push_back("-p");
    args.push_back(p);
  }
  std::vector<char*> argv;
  for (auto &s : args) {
    argv.push_back(s.data());
  }
  argv.push_back(nullptr);
  int terminal=-1;pid_t pid=tty?forkpty(&terminal,nullptr,nullptr,nullptr):fork();
  if (pid < 0) {
    throw std::runtime_error("fork failed");
  }
  if(pid==0){
    if(!tty){int fd=open("/dev/null",O_WRONLY);dup2(fd,1);dup2(fd,2);close(fd);}
    execvp("ros2",argv.data());_exit(127);
  }
  if (terminal >= 0) {
    fcntl(terminal, F_SETFL, O_NONBLOCK);
  }
  return {pid,terminal,name};
}
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  auto n=std::make_shared<rclcpp::Node>("compat_contract_probe");
  // 以下布尔量分别记录各项契约断言是否成立；enable 计数用于区分按键触发与自行启动。
  bool publish=true,armed=false,expired=false,converted=false,depth_ok=false;
  bool calibrated_selected=false;
  int enables=0,disables=0,wrenches=0;
  std::array<double,6> last{};
  // 合成 IMU/深度输入：group 0x80 为原始量，0x81 为姿态量；连接 ID 固定 42。
  auto ip=n->create_publisher<underwater_interfaces::msg::ImuState>("/compat_fixture/sensors/imu/state",rclcpp::SensorDataQoS());
  auto cp=n->create_publisher<underwater_interfaces::msg::ImuState>("/compat_fixture/sensors/imu/calibrated",rclcpp::SensorDataQoS());
  auto dp=n->create_publisher<underwater_interfaces::msg::DepthState>("/compat_fixture/sensors/depth/state",rclcpp::SensorDataQoS());
  auto ap=n->create_publisher<underwater_interfaces::msg::AllocationStatus>("/compat_fixture/control/allocation",10);
  // /imu_data 契约：合成 yaw=90°、gyro_z=180 应被换算为 msg_pkg 字段，随后由失效帧置为 NaN。
  auto is=n->create_subscription<msg_pkg::msg::ImuData>("/imu_data",10,[&](msg_pkg::msg::ImuData::ConstSharedPtr m){
    if(m->raw_valid&&m->attitude_valid&&std::abs(m->angle_z-90)<1e-6&&std::abs(m->gyro_z-180)<1e-6)converted=true;
    if(converted&&!m->raw_valid&&!m->attitude_valid&&std::isnan(m->angle_z)&&std::isnan(m->acc_x))expired=true;
    if(m->raw_valid&&m->attitude_valid&&std::abs(m->angle_z-30)<1e-6&&std::abs(m->gyro_z-90)<1e-6)calibrated_selected=true; // imu_source:=calibrated 时应改用标定量（90-60=30）
  });
  auto ds=n->create_subscription<msg_pkg::msg::DepthSensor>("/depth_temp",10,[&](msg_pkg::msg::DepthSensor::ConstSharedPtr m){
    if(m->depth_valid&&std::abs(m->depth-0.75)<1e-9&&m->temperature_valid)depth_ok=true; // 合成 filtered_depth_m=0.75 应原样转发
  });
  auto ws=n->create_subscription<geometry_msgs::msg::WrenchStamped>("/compat_fixture/control/wrench_request",10,
    [&](geometry_msgs::msg::WrenchStamped::ConstSharedPtr m){++wrenches;last={m->wrench.force.x,m->wrench.force.y,m->wrench.force.z,m->wrench.torque.x,m->wrench.torque.y,m->wrench.torque.z};});
  auto ms=n->create_subscription<geometry_msgs::msg::WrenchStamped>("/compat_fixture/control/manual_request",10,
    [&](geometry_msgs::msg::WrenchStamped::ConstSharedPtr m){++wrenches;last={m->wrench.force.x,m->wrench.force.y,m->wrench.force.z,m->wrench.torque.x,m->wrench.torque.y,m->wrench.torque.z};});
  auto service=n->create_service<std_srvs::srv::SetBool>("/compat_fixture/control/enable",
    [&](std_srvs::srv::SetBool::Request::SharedPtr q,std_srvs::srv::SetBool::Response::SharedPtr r){
      armed=q->data;if(armed)++enables;else ++disables;r->success=true;r->message="isolated fake service";
    });
  // 以 50ms 周期发布合成帧；publish=false 时停发，用于验证各通道的失效（stale）行为。
  auto timer=n->create_wall_timer(50ms,[&]{
    if(publish){
      underwater_interfaces::msg::ImuState m;m.header.stamp=n->now();m.connection_id=42;m.fresh=true;m.group=0x80;m.raw_valid=true;
      m.acceleration_m_s2={1,2,3};m.angular_velocity_rad_s={0,0,3.14159265358979323846};ip->publish(m);
      m.group=0x81;m.raw_valid=false;m.euler_valid=true;m.euler_rpy_rad={0,0,1.57079632679489661923};ip->publish(m);
      m.group=0x80;m.raw_valid=true;m.euler_valid=false;m.angular_velocity_rad_s={0,0,1.57079632679489661923};cp->publish(m);
      m.group=0x81;m.raw_valid=false;m.euler_valid=true;m.euler_rpy_rad={0,0,0.52359877559829887308};cp->publish(m);
      underwater_interfaces::msg::DepthState d;d.header.stamp=n->now();d.connection_id=42;d.fresh=d.depth_valid=d.temperature_valid=d.pressure_valid=true;
      d.filtered_depth_m=0.75;d.temperature_c=25;d.pressure_pa=108000;dp->publish(d);
    }
    underwater_interfaces::msg::AllocationStatus a;a.header.stamp=n->now();a.armed=armed;ap->publish(a);
  });
  std::vector<Child> children;
  // spin：推进本节点回调，并抽空子进程 pty 输出，防止缓冲区写满而阻塞被测进程。
  auto spin=[&](double seconds){
    auto end=Clock::now()+std::chrono::duration<double>(seconds);
    while(Clock::now()<end){rclcpp::spin_some(n);
      for(const auto &c:children)if(c.terminal>=0){char buf[4096];while(read(c.terminal,buf,sizeof(buf))>0){}}
      std::this_thread::sleep_for(5ms);
    }
  };
  // finish：最多等 5 秒（100 × spin(.05)）收集子进程退出码；超时则 SIGKILL 并判失败。
  auto finish=[&](Child &c){
    int s=0;for(int i=0;i<100;++i){if(waitpid(c.pid,&s,WNOHANG)==c.pid){std::cout<<c.label<<" exit="<<(WIFEXITED(s)?WEXITSTATUS(s):-WTERMSIG(s))<<"\n";if(c.terminal>=0)close(c.terminal);c.pid=-1;c.terminal=-1;return WIFEXITED(s)&&WEXITSTATUS(s)==0;}spin(.05);}
    kill(c.pid,SIGKILL);waitpid(c.pid,&s,0);if(c.terminal>=0)close(c.terminal);c.pid=-1;c.terminal=-1;return false;
  };
  const auto out=std::filesystem::path("/tmp")/("underwater-compat-probe-"+std::to_string(getpid()));
  std::filesystem::create_directory(out);
  // 第一轮：四条传感器链路同时在线 3 秒，再停发 3 秒验证失效语义。
  children.push_back(launch("imu_py","imu_publisher",{"duration_s:=5.0"}));
  children.push_back(launch("depth_py","depth_publisher",{"duration_s:=5.0"}));
  children.push_back(launch("sensors_cpp","sensors_sub",{"duration_s:=5.0"}));
  children.push_back(launch("sensors_cpp","sensors_sub_jilushuju",{"duration_s:=5.0","output_dir:="+out.string()}));
  spin(3);publish=false;spin(3);
  bool ok=converted&&expired&&depth_ok;
  for(auto &c:children)ok=finish(c)&&ok;
  // CSV 记录契约：输出目录下应有行数 > 10 的记录文件，证明订阅端确实在写盘。
  bool recorded=false;
  for(const auto &p:std::filesystem::directory_iterator(out))if(p.path().extension()==".csv"){
    std::ifstream in(p.path());std::string line;int lines=0;while(std::getline(in,line))++lines;recorded|=lines>10;
  }
  ok&=recorded;
  // 第二轮：imu_source:=calibrated，记录文件应含标记列 calibrated。
  publish=true;
  const auto corrected_out=out/"calibrated";
  children.push_back(launch("imu_py","imu_publisher",{"duration_s:=2.0","imu_source:=calibrated"}));
  children.push_back(launch("sensors_cpp","sensors_sub_jilushuju",{"duration_s:=2.0","imu_source:=calibrated","output_dir:="+corrected_out.string()}));
  spin(3);
  ok=finish(children[children.size()-2])&&ok;ok=finish(children.back())&&ok;
  bool calibrated_recorded=false;
  if(std::filesystem::exists(corrected_out))for(const auto &p:std::filesystem::directory_iterator(corrected_out)){
    std::ifstream in(p.path());std::string line;
    while(std::getline(in,line))if(line.find(",calibrated,")!=std::string::npos)calibrated_recorded=true;
  }
  ok=calibrated_selected&&calibrated_recorded&&ok;publish=false;
  // 第三轮：manual_control 在 pty 中运行，验证启动不自行使能、按键 WATI 生效、Q 退出并停桨。
  children.push_back(launch("manual_control","manual_control",{"duration_s:=10.0"},true));
  auto &manual=children.back();spin(2);const bool no_start_enable=enables==0;
  ok=(write(manual.terminal,"WATI",4)==4)&&ok;spin(1.5);
  const bool keys=enables==1&&wrenches>0&&last[0]==1&&last[1]==1&&last[2]==1&&last[5]==1; // WATI 应对应 Fx/Fy/Fz/Tz 各 1.0
  ok=(write(manual.terminal,"Q",1)==1)&&ok;spin(.5);ok=finish(manual)&&ok;
  const bool stopped=!armed&&disables==1;ok&=no_start_enable&&keys&&stopped;
  spin(.5);
  // 第四轮：preview:=true 时单键输入与 Ctrl+C，随后 forces_pub 波形发布不得触发使能。
  children.push_back(launch("manual_control","manual_control",{"preview:=true","duration_s:=10.0"},true));
  auto &preview=children.back();spin(2);ok=(write(preview.terminal,"W",1)==1)&&ok;spin(.5);
  // Generate the terminal Ctrl+C event (foreground process group), not a
  // signal sent only to the ros2 Python launcher.
  ok=(write(preview.terminal,"\003",1)==1)&&ok;spin(.5);ok=finish(preview)&&ok;ok&=enables==1;
  spin(1);const int before_wave=wrenches;
  children.push_back(launch("manual_control","forces_pub",{"preview:=true","duration_s:=1.0"}));
  spin(3);ok=finish(children.back())&&ok;
  const bool waveform=wrenches>before_wave&&enables==1;ok&=waveform;
  // 汇总各契约布尔量；返回 0 表示 PASS，1 表示 FAIL。
  std::cout<<"converted="<<converted<<" expired="<<expired<<" depth="<<depth_ok<<" recorded="<<recorded
           <<" calibrated_selected="<<calibrated_selected<<" calibrated_recorded="<<calibrated_recorded
           <<" no_start_enable="<<no_start_enable<<" keys="<<keys<<" stopped="<<stopped<<" waveform="<<waveform<<" enables="<<enables<<" result="<<(ok?"PASS":"FAIL")<<" output="<<out<<"\n";
  rclcpp::shutdown();return ok?0:1;
}
