// Synthetic frames only: launches devices_node, never a serial gateway or controller.
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <underwater_interfaces/msg/gateway_frame.hpp>
#include <underwater_interfaces/msg/imu_state.hpp>
#include <underwater_interfaces/msg/imu_calibration_status.hpp>
#include <underwater_interfaces/srv/calibrate_imu.hpp>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <thread>
#include <fcntl.h>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

namespace ui=underwater_interfaces::msg;
using Service=underwater_interfaces::srv::CalibrateImu;
using Clock=std::chrono::steady_clock;
using namespace std::chrono_literals;
namespace fs=std::filesystem;
constexpr double rad=3.14159265358979323846/180;
void require(bool ok,const std::string &why) { if(!ok) throw std::runtime_error(why); }
struct Device {
  pid_t pid=-1;
  void stop() {
    if(pid<=0) return;
    kill(-pid,SIGINT);
    for(int i=0;i<60;++i) { if(waitpid(pid,nullptr,WNOHANG)==pid){pid=-1;return;} std::this_thread::sleep_for(50ms); }
    kill(-pid,SIGKILL);waitpid(pid,nullptr,0);pid=-1;
  }
  void start(const std::string &ns,const std::string &file,const std::string &log) {
    stop();pid=fork();require(pid>=0,"fork failed");
    if(pid==0) {
      setpgid(0,0);int fd=open(log.c_str(),O_WRONLY|O_CREAT|O_APPEND,0600);
      if(fd>=0){dup2(fd,1);dup2(fd,2);close(fd);}
      const std::string remap="__ns:="+ns,param="imu_calibration_file:="+file;
      execlp("ros2","ros2","run","underwater_devices","devices_node","--ros-args","-r",remap.c_str(),
        "-p",param.c_str(),static_cast<char *>(nullptr));_exit(127);
    }
    setpgid(pid,pid);
  }
  ~Device(){stop();}
};
void put(std::vector<std::uint8_t> &bytes,std::size_t offset,float value) {
  std::uint32_t bits;std::memcpy(&bits,&value,4);
  for(int i=0;i<4;++i) bytes[offset+i]=(bits>>(8*i))&255;
}
int main(int argc,char **argv) {
  // This explicit test domain cannot accidentally reuse the real robot namespace.
  setenv("ROS_DOMAIN_ID","184",1);setenv("ROS_LOCALHOST_ONLY","1",1);
  rclcpp::init(argc,argv);
  const std::string ns="/imu_reference_fixture_"+std::to_string(getpid());
  const fs::path directory=fs::path("/tmp")/("underwater-imu-probe-"+std::to_string(getpid()));
  fs::create_directories(directory);const std::string file=(directory/"reference.txt").string();
  int result=1;
  try {
    auto node=std::make_shared<rclcpp::Node>("imu_reference_probe",ns);
    auto publisher=node->create_publisher<ui::GatewayFrame>("gateway/frames",rclcpp::SensorDataQoS().keep_last(100));
    bool send=true;double yaw=179*rad,gyro=0;std::uint32_t seq=0;
    unsigned raw_count=0,calibrated_count=0,status_count=0;
    ui::ImuState raw,calibrated;ui::ImuCalibrationStatus status;
    auto a=node->create_subscription<ui::ImuState>("sensors/imu/state",rclcpp::SensorDataQoS(),
      [&](ui::ImuState::ConstSharedPtr m){if(m->group==0x81){raw=*m;++raw_count;}});
    auto b=node->create_subscription<ui::ImuState>("sensors/imu/calibrated",rclcpp::SensorDataQoS(),
      [&](ui::ImuState::ConstSharedPtr m){if(m->group==0x81){calibrated=*m;++calibrated_count;}
        else if(m->group==0x80 && m->raw_valid)require(std::abs(m->acceleration_m_s2[2]-9.80665)<1e-6,"raw acceleration changed");});
    auto s=node->create_subscription<ui::ImuCalibrationStatus>("sensors/imu/calibration_status",rclcpp::QoS(1).transient_local(),
      [&](ui::ImuCalibrationStatus::ConstSharedPtr m){status=*m;++status_count;});
    auto timer=node->create_wall_timer(20ms,[&] {
      if(!send)return;
      ui::GatewayFrame f;f.header.stamp=node->now();f.family=0x5b;f.flags=8;f.target=1;f.connection_id=42;
      f.sequence=++seq;f.command=0x80;f.payload.assign(40,0);f.payload[0]=3;
      put(f.payload,12,1);put(f.payload,24,static_cast<float>(gyro));publisher->publish(f);
      f.sequence=++seq;f.command=0x81;f.payload.assign(32,0);f.payload[0]=8;
      put(f.payload,28,static_cast<float>(yaw));publisher->publish(f);
    });
    auto client=node->create_client<Service>("sensors/imu/calibrate");
    auto clear=node->create_client<std_srvs::srv::Trigger>("sensors/imu/clear_calibration");
    auto until=[&](const std::function<bool()> &condition,double timeout) {
      const auto end=Clock::now()+std::chrono::duration<double>(timeout);
      while(Clock::now()<end){rclcpp::spin_some(node);if(condition())return true;std::this_thread::sleep_for(5ms);}return false;
    };
    Device device;device.start(ns,file,(directory/"devices.log").string());
    require(until([&]{return client->service_is_ready()&&raw_count>5&&status_count>0;},10),"device startup");
    require(!status.calibrated && !calibrated.euler_valid,"uncalibrated attitude must not be labelled valid");
    auto request=std::make_shared<Service::Request>();request->samples=10;request->timeout_s=5;
    auto response=client->async_send_request(request);
    require(until([&]{return response.wait_for(0ms)==std::future_status::ready;},8),"calibration response timeout");
    require(response.get()->success && fs::is_regular_file(file),"capture saved successfully");
    require(until([&]{return status.calibrated&&calibrated.euler_valid&&std::abs(calibrated.euler_rpy_rad[2])<1e-5;},3),"reference pose becomes zero");
    require(raw.euler_valid&&std::abs(raw.euler_rpy_rad[2]-179*rad)<1e-5,"raw attitude untouched");
    gyro=.2;response=client->async_send_request(request);
    require(until([&]{return response.wait_for(0ms)==std::future_status::ready;},7),"motion response");
    require(!response.get()->success && status.calibrated,"moving capture preserves existing reference");gyro=0;
    yaw=-179*rad;
    require(until([&]{return calibrated.euler_valid&&std::abs(calibrated.euler_rpy_rad[2]-2*rad)<1e-5;},3),"yaw wraps continuously");
    send=false;
    require(until([&]{return !calibrated.fresh&&!calibrated.euler_valid&&std::isnan(calibrated.euler_rpy_rad[2]);},3),"stale correction invalidates");
    send=true;
    device.stop();status_count=raw_count=calibrated_count=0;status.calibrated=false;calibrated.euler_valid=false;
    device.start(ns,file,(directory/"devices.log").string());
    require(until([&]{return status_count>0&&status.calibrated&&calibrated_count>5&&calibrated.euler_valid;},10),"restart loads reference");
    require(std::abs(calibrated.euler_rpy_rad[2]-2*rad)<1e-5,"reloaded correction matches");
    auto cleared=clear->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    require(until([&]{return cleared.wait_for(0ms)==std::future_status::ready;},5),"clear response");
    require(cleared.get()->success&&!fs::exists(file),"persistent reference removed");
    require(until([&]{return !status.calibrated&&!calibrated.euler_valid&&std::isnan(calibrated.euler_rpy_rad[2]);},3),"cleared attitude invalidates");
    require(raw.euler_valid,"raw stream survives clear");
    device.stop();status_count=0;
    // A directory cannot be replaced by a calibration file. Failure must not activate a candidate.
    device.start(ns,directory.string(),(directory/"devices.log").string());
    require(until([&]{return status_count>0&&client->service_is_ready();},10),"invalid-file startup remains usable");
    response=client->async_send_request(request);
    require(until([&]{return response.wait_for(0ms)==std::future_status::ready;},8),"save failure response");
    require(!response.get()->success&&!status.calibrated&&fs::is_directory(directory),"failed save does not activate or damage path");
    device.stop();result=0;
    std::cout<<"PASS IMU calibration integration: capture, raw preservation, motion rejection, yaw wrap, expiry, restart/load, clear, save failure; evidence="<<directory<<'\n';
  } catch(const std::exception &e) {std::cerr<<"FAIL IMU calibration integration: "<<e.what()<<"; evidence="<<directory<<'\n';}
  rclcpp::shutdown();return result;
}
