// Synthetic frames only: launches devices_node, never a serial gateway or controller.
// 验证内容：IMU 参考姿态标定全流程——采集存盘、原始姿态不受影响、运动中拒绝标定、
//   偏航角连续回绕、数据过期后校正失效、重启加载参考值、清除标定，以及保存失败不得激活候选值。
// 运行方式：bash tools/validate.sh imu-calibration（等价于 ros2 run underwater_bringup imu_calibration_probe）。
// 通过标准：退出码 0 且打印 "PASS IMU calibration integration"；证据（参考文件、devices.log）留在 /tmp 目录。
// 是否需要真实硬件：不需要。只启动 underwater_devices 的 devices_node，用合成网关帧驱动，不接触串口与控制器。
//
// ── 初学者导读 ────────────────────────────────────────────────────────────
// 这是 IMU 姿态基准标定的端到端集成测试：用合成网关帧驱动真的 devices_node，验证
// “采集参考姿态 → 校正生效 → 运动中拒绝 → 偏航回绕 → 过期失效 → 重启加载 → 清除标定”。
// 关键手法：
//   - fork/execvp + setpgid：在独立进程组里启停 devices_node；
//   - create_publisher<GatewayFrame>：伪造 MCU 帧；-r __ns:=... 重映射命名空间；
//   - create_client + async_send_request：调用标定/清除服务；
//   - transient_local QoS：晚加入的订阅者也能收到最近一次标定状态。
// C++ 概念：std::function、lambda [&] 捕获、std::filesystem、memcpy 按小端写字节。
// ─────────────────────────────────────────────────────────────────────────
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
void require(bool ok, const std::string &why) {
  if (!ok) {
    throw std::runtime_error(why);
  }
} // 断言失败抛异常，main 中统一置 result=1
// Device：以独立进程组启停 devices_node，stop() 先 SIGINT，最多等 3 秒（60 × 50ms）再 SIGKILL。
struct Device {
  pid_t pid=-1;
  void stop() {
    if(pid<=0) return;
    kill(-pid,SIGINT);
    for (int i = 0; i < 60; ++i) {
      if (waitpid(pid, nullptr, WNOHANG) == pid) {
        pid = -1;
        return;
      }
      std::this_thread::sleep_for(50ms);
    }
    kill(-pid, SIGKILL);
    waitpid(pid, nullptr, 0);
    pid = -1;
  }
  void start(const std::string &ns,const std::string &file,const std::string &log) {
    stop();pid=fork();require(pid>=0,"fork failed");
    if(pid==0) {
      setpgid(0,0);int fd=open(log.c_str(),O_WRONLY|O_CREAT|O_APPEND,0600); // 子进程日志追加写入，便于失败后取证
      if(fd>=0){dup2(fd,1);dup2(fd,2);close(fd);}
      const std::string remap="__ns:="+ns,param="imu_calibration_file:="+file; // 重映射命名空间并指定参考文件路径
      execlp("ros2","ros2","run","underwater_devices","devices_node","--ros-args","-r",remap.c_str(),
        "-p",param.c_str(),static_cast<char *>(nullptr));_exit(127); // exec 失败返回码 127
    }
    setpgid(pid,pid);
  }
  ~Device(){stop();}
};
// 以小端序把 float 写入帧载荷的指定偏移，模拟设备的字节布局。
void put(std::vector<std::uint8_t> &bytes,std::size_t offset,float value) {
  std::uint32_t bits;std::memcpy(&bits,&value,4);
  for (int i = 0; i < 4; ++i) {
    bytes[offset + i] = static_cast<std::uint8_t>((bits >> (8 * i)) & 0xff);
  }
}
// 测试入口：设定独立 ROS 域，启动 devices_node，逐个用例断言标定行为。
int main(int argc,char **argv) {
  // This explicit test domain cannot accidentally reuse the real robot namespace.
  setenv("ROS_DOMAIN_ID","184",1);setenv("ROS_LOCALHOST_ONLY","1",1);
  rclcpp::init(argc,argv);
  const std::string ns="/imu_reference_fixture_"+std::to_string(getpid());
  const fs::path directory=fs::path("/tmp")/("underwater-imu-probe-"+std::to_string(getpid()));
  fs::create_directories(directory);const std::string file=(directory/"reference.txt").string();
  int result=1; // 默认失败；仅在全部用例通过后置 0
  try {
    auto node=std::make_shared<rclcpp::Node>("imu_reference_probe",ns);
    auto publisher=node->create_publisher<ui::GatewayFrame>("gateway/frames",rclcpp::SensorDataQoS().keep_last(100));
    bool send=true;double yaw=179*rad,gyro=0;std::uint32_t seq=0; // 起始 yaw=179°，gyro=0 表示静止
    unsigned raw_count=0,calibrated_count=0,status_count=0;
    ui::ImuState raw,calibrated;ui::ImuCalibrationStatus status;
    // 组 0x81 为姿态帧：/sensors/imu/state 是未校正原始值，/sensors/imu/calibrated 是校正后结果。
    auto a=node->create_subscription<ui::ImuState>("sensors/imu/state",rclcpp::SensorDataQoS(),
      [&](ui::ImuState::ConstSharedPtr m){if(m->group==0x81){raw=*m;++raw_count;}});
    auto b=node->create_subscription<ui::ImuState>("sensors/imu/calibrated",rclcpp::SensorDataQoS(),
      [&](ui::ImuState::ConstSharedPtr m){if(m->group==0x81){calibrated=*m;++calibrated_count;}
        else if(m->group==0x80 && m->raw_valid)require(std::abs(m->acceleration_m_s2[2]-9.80665)<1e-6,"raw acceleration changed");});
    // 标定状态用 transient_local 发布，晚加入的订阅者也能拿到最近一次状态。
    auto s=node->create_subscription<ui::ImuCalibrationStatus>("sensors/imu/calibration_status",rclcpp::QoS(1).transient_local(),
      [&](ui::ImuCalibrationStatus::ConstSharedPtr m){status=*m;++status_count;});
    // 20ms 周期发送合成帧：0x80 为原始量（此处 az=1g、gyro 可变），0x81 为姿态量（偏移 28 处放 yaw）。
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
    // until：在给定超时内反复 spin，条件满足即返回 true。
    // until：在给定超时内反复 spin，条件满足即返回 true，否则超时返回 false。
    auto until=[&](const std::function<bool()> &condition,double timeout) {
      const auto end=Clock::now()+std::chrono::duration<double>(timeout);
      while(Clock::now()<end){rclcpp::spin_some(node);if(condition())return true;std::this_thread::sleep_for(5ms);}return false;
    };
    // 用例 1：启动后用 179° 静止姿态采集参考，期望参考姿态被归零、原始流保持不变。
    Device device;device.start(ns,file,(directory/"devices.log").string());
    require(until([&]{return client->service_is_ready()&&raw_count>5&&status_count>0;},10),"device startup");
    require(!status.calibrated && !calibrated.euler_valid,"uncalibrated attitude must not be labelled valid");
    auto request=std::make_shared<Service::Request>();request->samples=10;request->timeout_s=5; // 采 10 个样本，最多等 5 秒
    auto response=client->async_send_request(request);
    require(until([&]{return response.wait_for(0ms)==std::future_status::ready;},8),"calibration response timeout");
    require(response.get()->success && fs::is_regular_file(file),"capture saved successfully");
    require(until([&]{return status.calibrated&&calibrated.euler_valid&&std::abs(calibrated.euler_rpy_rad[2])<1e-5;},3),"reference pose becomes zero");
    require(raw.euler_valid&&std::abs(raw.euler_rpy_rad[2]-179*rad)<1e-5,"raw attitude untouched"); // 原始话题仍应是 179°
    // 用例 2：给 0.2rad/s 的角速度模拟运动，标定应被拒绝且保留既有参考值。
    gyro=.2;response=client->async_send_request(request);
    require(until([&]{return response.wait_for(0ms)==std::future_status::ready;},7),"motion response");
    require(!response.get()->success && status.calibrated,"moving capture preserves existing reference");gyro=0;
    // 用例 3：yaw 由 179° 跳到 -179°，校正后应连续回绕为 +2°。
    yaw=-179*rad;
    require(until([&]{return calibrated.euler_valid&&std::abs(calibrated.euler_rpy_rad[2]-2*rad)<1e-5;},3),"yaw wraps continuously");
    send=false;
    // 用例 4：停止发帧后校正结果必须失效（fresh=false、角度 NaN）。
    require(until([&]{return !calibrated.fresh&&!calibrated.euler_valid&&std::isnan(calibrated.euler_rpy_rad[2]);},3),"stale correction invalidates");
    send=true;
    // 用例 5：重启 devices_node 后应从磁盘加载参考值，校正结果与重启前一致。
    device.stop();status_count=raw_count=calibrated_count=0;status.calibrated=false;calibrated.euler_valid=false;
    device.start(ns,file,(directory/"devices.log").string());
    require(until([&]{return status_count>0&&status.calibrated&&calibrated_count>5&&calibrated.euler_valid;},10),"restart loads reference");
    require(std::abs(calibrated.euler_rpy_rad[2]-2*rad)<1e-5,"reloaded correction matches");
    // 用例 6：清除标定后参考文件应删除，校正话题立即失效，原始流不受影响。
    auto cleared=clear->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    require(until([&]{return cleared.wait_for(0ms)==std::future_status::ready;},5),"clear response");
    require(cleared.get()->success&&!fs::exists(file),"persistent reference removed");
    require(until([&]{return !status.calibrated&&!calibrated.euler_valid&&std::isnan(calibrated.euler_rpy_rad[2]);},3),"cleared attitude invalidates");
    require(raw.euler_valid,"raw stream survives clear");
    device.stop();status_count=0;
    // 用例 7：把参考文件路径指向一个目录，保存必然失败；此时不得激活候选值，也不能损坏该路径。
    // A directory cannot be replaced by a calibration file. Failure must not activate a candidate.
    device.start(ns,directory.string(),(directory/"devices.log").string());
    require(until([&]{return status_count>0&&client->service_is_ready();},10),"invalid-file startup remains usable");
    response=client->async_send_request(request);
    require(until([&]{return response.wait_for(0ms)==std::future_status::ready;},8),"save failure response");
    require(!response.get()->success&&!status.calibrated&&fs::is_directory(directory),"failed save does not activate or damage path");
    device.stop();result=0; // 全部用例通过，退出码置 0
    std::cout<<"PASS IMU calibration integration: capture, raw preservation, motion rejection, yaw wrap, expiry, restart/load, clear, save failure; evidence="<<directory<<'\n';
  } catch(const std::exception &e) {std::cerr<<"FAIL IMU calibration integration: "<<e.what()<<"; evidence="<<directory<<'\n';} // 退出码沿用 result（仍为 1）
  rclcpp::shutdown();return result;
}
