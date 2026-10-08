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
Child launch(const std::string &pkg,const std::string &name,const std::vector<std::string> &params,bool tty=false) {
  std::vector<std::string> args{"ros2","run",pkg,name,"--ros-args","-p","auto_start:=false","-p","robot_namespace:=/compat_fixture"};
  for(const auto &p:params){args.push_back("-p");args.push_back(p);}
  std::vector<char*> argv;for(auto &s:args)argv.push_back(s.data());argv.push_back(nullptr);
  int terminal=-1;pid_t pid=tty?forkpty(&terminal,nullptr,nullptr,nullptr):fork();
  if(pid<0)throw std::runtime_error("fork failed");
  if(pid==0){
    if(!tty){int fd=open("/dev/null",O_WRONLY);dup2(fd,1);dup2(fd,2);close(fd);}
    execvp("ros2",argv.data());_exit(127);
  }
  if(terminal>=0)fcntl(terminal,F_SETFL,O_NONBLOCK);
  return {pid,terminal,name};
}
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  auto n=std::make_shared<rclcpp::Node>("compat_contract_probe");
  bool publish=true,armed=false,expired=false,converted=false,depth_ok=false;
  int enables=0,disables=0,wrenches=0;
  std::array<double,6> last{};
  auto ip=n->create_publisher<underwater_interfaces::msg::ImuState>("/compat_fixture/sensors/imu/state",rclcpp::SensorDataQoS());
  auto dp=n->create_publisher<underwater_interfaces::msg::DepthState>("/compat_fixture/sensors/depth/state",rclcpp::SensorDataQoS());
  auto ap=n->create_publisher<underwater_interfaces::msg::AllocationStatus>("/compat_fixture/control/allocation",10);
  auto is=n->create_subscription<msg_pkg::msg::ImuData>("/imu_data",10,[&](msg_pkg::msg::ImuData::ConstSharedPtr m){
    if(m->raw_valid&&m->attitude_valid&&std::abs(m->angle_z-90)<1e-6&&std::abs(m->gyro_z-180)<1e-6)converted=true;
    if(converted&&!m->raw_valid&&!m->attitude_valid&&std::isnan(m->angle_z)&&std::isnan(m->acc_x))expired=true;
  });
  auto ds=n->create_subscription<msg_pkg::msg::DepthSensor>("/depth_temp",10,[&](msg_pkg::msg::DepthSensor::ConstSharedPtr m){
    if(m->depth_valid&&std::abs(m->depth-0.75)<1e-9&&m->temperature_valid)depth_ok=true;
  });
  auto ws=n->create_subscription<geometry_msgs::msg::WrenchStamped>("/compat_fixture/control/wrench_request",10,
    [&](geometry_msgs::msg::WrenchStamped::ConstSharedPtr m){++wrenches;last={m->wrench.force.x,m->wrench.force.y,m->wrench.force.z,m->wrench.torque.x,m->wrench.torque.y,m->wrench.torque.z};});
  auto ms=n->create_subscription<geometry_msgs::msg::WrenchStamped>("/compat_fixture/control/manual_request",10,
    [&](geometry_msgs::msg::WrenchStamped::ConstSharedPtr m){++wrenches;last={m->wrench.force.x,m->wrench.force.y,m->wrench.force.z,m->wrench.torque.x,m->wrench.torque.y,m->wrench.torque.z};});
  auto service=n->create_service<std_srvs::srv::SetBool>("/compat_fixture/control/enable",
    [&](std_srvs::srv::SetBool::Request::SharedPtr q,std_srvs::srv::SetBool::Response::SharedPtr r){
      armed=q->data;if(armed)++enables;else ++disables;r->success=true;r->message="isolated fake service";
    });
  auto timer=n->create_wall_timer(50ms,[&]{
    if(publish){
      underwater_interfaces::msg::ImuState m;m.header.stamp=n->now();m.connection_id=42;m.fresh=true;m.group=0x80;m.raw_valid=true;
      m.acceleration_m_s2={1,2,3};m.angular_velocity_rad_s={0,0,3.14159265358979323846};ip->publish(m);
      m.group=0x81;m.raw_valid=false;m.euler_valid=true;m.euler_rpy_rad={0,0,1.57079632679489661923};ip->publish(m);
      underwater_interfaces::msg::DepthState d;d.header.stamp=n->now();d.connection_id=42;d.fresh=d.depth_valid=d.temperature_valid=d.pressure_valid=true;
      d.filtered_depth_m=0.75;d.temperature_c=25;d.pressure_pa=108000;dp->publish(d);
    }
    underwater_interfaces::msg::AllocationStatus a;a.header.stamp=n->now();a.armed=armed;ap->publish(a);
  });
  std::vector<Child> children;
  auto spin=[&](double seconds){
    auto end=Clock::now()+std::chrono::duration<double>(seconds);
    while(Clock::now()<end){rclcpp::spin_some(n);
      for(const auto &c:children)if(c.terminal>=0){char buf[4096];while(read(c.terminal,buf,sizeof(buf))>0){}}
      std::this_thread::sleep_for(5ms);
    }
  };
  auto finish=[&](Child &c){
    int s=0;for(int i=0;i<100;++i){if(waitpid(c.pid,&s,WNOHANG)==c.pid){std::cout<<c.label<<" exit="<<(WIFEXITED(s)?WEXITSTATUS(s):-WTERMSIG(s))<<"\n";if(c.terminal>=0)close(c.terminal);c.pid=-1;c.terminal=-1;return WIFEXITED(s)&&WEXITSTATUS(s)==0;}spin(.05);}
    kill(c.pid,SIGKILL);waitpid(c.pid,&s,0);if(c.terminal>=0)close(c.terminal);c.pid=-1;c.terminal=-1;return false;
  };
  const auto out=std::filesystem::path("/tmp")/("underwater-compat-probe-"+std::to_string(getpid()));
  std::filesystem::create_directory(out);
  children.push_back(launch("imu_py","imu_publisher",{"duration_s:=5.0"}));
  children.push_back(launch("depth_py","depth_publisher",{"duration_s:=5.0"}));
  children.push_back(launch("sensors_cpp","sensors_sub",{"duration_s:=5.0"}));
  children.push_back(launch("sensors_cpp","sensors_sub_jilushuju",{"duration_s:=5.0","output_dir:="+out.string()}));
  spin(3);publish=false;spin(3);
  bool ok=converted&&expired&&depth_ok;
  for(auto &c:children)ok=finish(c)&&ok;
  bool recorded=false;
  for(const auto &p:std::filesystem::directory_iterator(out))if(p.path().extension()==".csv"){
    std::ifstream in(p.path());std::string line;int lines=0;while(std::getline(in,line))++lines;recorded|=lines>10;
  }
  ok&=recorded;
  children.push_back(launch("manual_control","manual_control",{"duration_s:=10.0"},true));
  auto &manual=children.back();spin(2);const bool no_start_enable=enables==0;
  ok=(write(manual.terminal,"WATI",4)==4)&&ok;spin(1.5);
  const bool keys=enables==1&&wrenches>0&&last[0]==1&&last[1]==1&&last[2]==1&&last[5]==1;
  ok=(write(manual.terminal,"Q",1)==1)&&ok;spin(.5);ok=finish(manual)&&ok;
  const bool stopped=!armed&&disables==1;ok&=no_start_enable&&keys&&stopped;
  spin(.5);
  children.push_back(launch("manual_control","manual_control",{"preview:=true","duration_s:=10.0"},true));
  auto &preview=children.back();spin(2);ok=(write(preview.terminal,"W",1)==1)&&ok;spin(.5);
  // Generate the terminal Ctrl+C event (foreground process group), not a
  // signal sent only to the ros2 Python launcher.
  ok=(write(preview.terminal,"\003",1)==1)&&ok;spin(.5);ok=finish(preview)&&ok;ok&=enables==1;
  spin(1);const int before_wave=wrenches;
  children.push_back(launch("manual_control","forces_pub",{"preview:=true","duration_s:=1.0"}));
  spin(3);ok=finish(children.back())&&ok;
  const bool waveform=wrenches>before_wave&&enables==1;ok&=waveform;
  std::cout<<"converted="<<converted<<" expired="<<expired<<" depth="<<depth_ok<<" recorded="<<recorded
           <<" no_start_enable="<<no_start_enable<<" keys="<<keys<<" stopped="<<stopped<<" waveform="<<waveform<<" enables="<<enables<<" result="<<(ok?"PASS":"FAIL")<<" output="<<out<<"\n";
  rclcpp::shutdown();return ok?0:1;
}
