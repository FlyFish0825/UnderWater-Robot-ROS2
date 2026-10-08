#include "sensors_cpp/entry.hpp"
#include <rclcpp/rclcpp.hpp>
#include <underwater_interfaces/msg/imu_state.hpp>
#include <underwater_interfaces/msg/depth_state.hpp>
#include <msg_pkg/msg/imu_data.hpp>
#include <msg_pkg/msg/depth_sensor.hpp>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <thread>
#include <unistd.h>
namespace compatibility {
using Clock=std::chrono::steady_clock;
using namespace std::chrono_literals;
namespace ui=underwater_interfaces::msg;
static constexpr double degrees=57.295779513082320876;
int telemetry(const std::string &entry,const std::shared_ptr<rclcpp::Node> &node) {
  const auto ns=backend_namespace(node);
  const bool imu_entry=entry=="imu_publisher";
  const bool depth_entry=entry=="depth_publisher"||entry=="depth";
  const bool record=entry=="sensors_sub_jilushuju";
  const double nan=std::numeric_limits<double>::quiet_NaN();
  std::array<double,3> acc{nan,nan,nan},gyro=acc,angle=acc;
  double depth=nan,temp=nan,pressure=nan;
  bool raw_valid=false,attitude_valid=false,depth_valid=false,temp_valid=false,depth_fresh=false;
  Clock::time_point raw_at{},attitude_at{},depth_at{};
  std::uint64_t connection=0;
  std_msgs::msg::Header imu_header,depth_header;
  rclcpp::Publisher<msg_pkg::msg::ImuData>::SharedPtr imu_pub;
  rclcpp::Publisher<msg_pkg::msg::DepthSensor>::SharedPtr depth_pub;
  if(imu_entry) {
    if(node->count_publishers("/imu_data")) throw std::runtime_error("/imu_data already has a publisher; reuse the running imu_publisher");
    imu_pub=node->create_publisher<msg_pkg::msg::ImuData>("/imu_data",10);
  }
  if(depth_entry) {
    if(node->count_publishers("/depth_temp")) throw std::runtime_error("/depth_temp already has a publisher; depth and depth_publisher are alternative entries");
    depth_pub=node->create_publisher<msg_pkg::msg::DepthSensor>("/depth_temp",10);
  }
  auto reset=[&](std::uint64_t id){
    if(connection==id)return;
    connection=id; acc.fill(nan);gyro.fill(nan);angle.fill(nan);
    depth=temp=pressure=nan;raw_valid=attitude_valid=depth_valid=temp_valid=depth_fresh=false;
  };
  auto imu_sub=node->create_subscription<ui::ImuState>(ns+"/sensors/imu/state",rclcpp::SensorDataQoS(),
    [&](ui::ImuState::ConstSharedPtr m){
      reset(m->connection_id);
      if(m->group==0x80) {
        raw_valid=m->fresh&&m->raw_valid; raw_at=Clock::now(); imu_header=m->header;
        for(int i=0;i<3;++i){acc[i]=raw_valid?m->acceleration_m_s2[i]:nan;gyro[i]=raw_valid?m->angular_velocity_rad_s[i]*degrees:nan;}
      } else if(m->group==0x81) {
        attitude_valid=m->fresh&&m->euler_valid;attitude_at=Clock::now();imu_header=m->header;
        for(int i=0;i<3;++i)angle[i]=attitude_valid?m->euler_rpy_rad[i]*degrees:nan;
      }
    });
  auto depth_sub=node->create_subscription<ui::DepthState>(ns+"/sensors/depth/state",rclcpp::SensorDataQoS(),
    [&](ui::DepthState::ConstSharedPtr m){
      reset(m->connection_id);depth_at=Clock::now();depth_header=m->header;
      depth_fresh=m->fresh;depth_valid=m->fresh&&m->depth_valid;temp_valid=m->fresh&&m->temperature_valid;
      depth=depth_valid?m->filtered_depth_m:nan;temp=temp_valid?m->temperature_c:nan;
      pressure=(m->fresh&&m->pressure_valid)?m->pressure_pa:nan;
    });
  std::ofstream file;
  if(record) {
    const auto out=node->declare_parameter<std::string>("output_dir",DEFAULT_RECORD_DIR);
    std::filesystem::create_directories(out);
    const auto stamp=std::chrono::system_clock::now().time_since_epoch();
    const auto filename=std::filesystem::path(out)/("sensors_"+std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(stamp).count())+"_"+std::to_string(getpid())+".csv");
    file.open(filename); if(!file)throw std::runtime_error("Cannot create recording file");
    file<<"receipt_time_s,depth_m,temp_c,pressure_pa,acc_x,acc_y,acc_z,gyro_x_deg_s,gyro_y_deg_s,gyro_z_deg_s,roll_deg,pitch_deg,yaw_deg,raw_valid,attitude_valid,depth_valid\n";
    RCLCPP_INFO(node->get_logger(),"Recording CSV at 10 Hz: %s",filename.c_str());
  }
  const auto start=Clock::now();
  const double duration=node->get_parameter("duration_s").as_double();
  if(!std::isfinite(duration)||duration<0)throw std::invalid_argument("duration_s must be nonnegative");
  auto next=Clock::now(); unsigned rows=0;
  while(running()&&(duration==0||std::chrono::duration<double>(Clock::now()-start).count()<duration)) {
    rclcpp::spin_some(node);
    if(Clock::now()<next){std::this_thread::sleep_for(5ms);continue;}
    next=Clock::now()+100ms;
    if(Clock::now()-raw_at>500ms){raw_valid=false;acc.fill(nan);gyro.fill(nan);}
    if(Clock::now()-attitude_at>500ms){attitude_valid=false;angle.fill(nan);}
    if(Clock::now()-depth_at>500ms){depth_valid=temp_valid=depth_fresh=false;depth=temp=pressure=nan;}
    if(imu_pub) {
      msg_pkg::msg::ImuData m;m.header=imu_header;
      m.acc_x=acc[0];m.acc_y=acc[1];m.acc_z=acc[2];m.gyro_x=gyro[0];m.gyro_y=gyro[1];m.gyro_z=gyro[2];
      m.angle_x=angle[0];m.angle_y=angle[1];m.angle_z=angle[2];m.raw_valid=raw_valid;m.attitude_valid=attitude_valid;imu_pub->publish(m);
    }
    if(depth_pub) {
      msg_pkg::msg::DepthSensor m;m.header=depth_header;m.depth=depth;m.temp=temp;m.pressure_pa=pressure;
      m.fresh=depth_fresh;m.depth_valid=depth_valid;m.temperature_valid=temp_valid;depth_pub->publish(m);
    }
    if(file.is_open()) {
      file<<std::setprecision(15)<<node->now().seconds()<<','<<depth<<','<<temp<<','<<pressure;
      for(const auto &a:{acc,gyro,angle})for(double v:a)file<<','<<v;
      file<<','<<raw_valid<<','<<attitude_valid<<','<<depth_valid<<'\n';
      if(++rows%10==0)file.flush();
      if(!file)throw std::runtime_error("Recording write failed");
    }
    if(imu_entry) RCLCPP_INFO_THROTTLE(node->get_logger(),*node->get_clock(),500,"angle(deg): %.3f %.3f %.3f | acc(m/s2): %.3f %.3f %.3f | raw=%d attitude=%d",angle[0],angle[1],angle[2],acc[0],acc[1],acc[2],raw_valid,attitude_valid);
    else if(depth_entry) RCLCPP_INFO_THROTTLE(node->get_logger(),*node->get_clock(),500,"depth=%.4f m temp=%.2f C pressure=%.1f Pa valid=%d",depth,temp,pressure,depth_valid);
    else RCLCPP_INFO_THROTTLE(node->get_logger(),*node->get_clock(),500,"depth=%.4f m | roll/pitch/yaw=%.3f %.3f %.3f deg | raw=%d attitude=%d depth=%d",depth,angle[0],angle[1],angle[2],raw_valid,attitude_valid,depth_valid);
  }
  if(file.is_open()){file.flush();RCLCPP_INFO(node->get_logger(),"Recording closed, %u rows",rows);}
  return 0;
}
}
