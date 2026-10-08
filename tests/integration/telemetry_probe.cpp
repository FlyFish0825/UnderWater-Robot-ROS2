// Bounded ROS integration check. Synthetic mode publishes ONLY internal ROS frames.
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <underwater_interfaces/msg/gateway_frame.hpp>
#include <underwater_interfaces/msg/gateway_status.hpp>
#include <underwater_interfaces/msg/imu_state.hpp>
#include <underwater_interfaces/msg/depth_state.hpp>
#include <underwater_interfaces/msg/motor_state.hpp>
namespace ui=underwater_interfaces::msg;
using Clock=std::chrono::steady_clock;
class Probe final : public rclcpp::Node {
public:
  Probe():Node("telemetry_probe") {
    synthetic=declare_parameter<bool>("synthetic",false);
    auto q=rclcpp::SensorDataQoS().keep_last(100);
    imu_sub=create_subscription<ui::ImuState>("sensors/imu/state",q,[this](ui::ImuState::ConstSharedPtr m){
      if(m->fresh && m->raw_valid) { ++imu_raw; if(synthetic && std::abs(m->acceleration_m_s2[0]-9.80665)>1e-6) ++errors; }
      if(m->fresh && (m->quaternion_valid||m->euler_valid)) ++imu_attitude;
      if(m->fresh && m->group==0x85 && m->baro_pressure_valid) ++imu_baro;
      if(!m->fresh && m->group==0x85) {
        ++baro_expired;
        if(m->baro_pressure_valid || m->baro_temperature_valid || m->baro_height_valid ||
           m->baro_reference_valid || !std::isnan(m->baro_pressure_pa)) ++errors;
      }
      if(!m->fresh) { ++imu_expired; if(m->raw_valid||m->quaternion_valid||m->euler_valid) ++errors; }
    });
    baro_pressure_sub=create_subscription<sensor_msgs::msg::FluidPressure>("sensors/imu/fluid_pressure",q,
      [this](sensor_msgs::msg::FluidPressure::ConstSharedPtr m){ ++baro_pressure;
        if(synthetic && (m->fluid_pressure!=101325 || m->header.frame_id!="imu_sensor")) ++errors; });
    baro_temperature_sub=create_subscription<sensor_msgs::msg::Temperature>("sensors/imu/temperature",q,
      [this](sensor_msgs::msg::Temperature::ConstSharedPtr m){ ++baro_temperature;
        if(synthetic && m->temperature!=22.5) ++errors; });
    raw_sub=create_subscription<sensor_msgs::msg::Imu>("sensors/imu/data_raw",q,[this](sensor_msgs::msg::Imu::ConstSharedPtr m){
      ++standard_imu; if(m->orientation_covariance[0]!=-1) ++errors;
    });
    depth_sub=create_subscription<ui::DepthState>("sensors/depth/state",q,[this](ui::DepthState::ConstSharedPtr m){
      if(m->fresh) { ++depth_count; last_depth=*m; }
      else ++depth_expired;
      if(!m->pressure_valid && !std::isnan(m->pressure_pa)) ++errors;
      if(!m->depth_valid && (!std::isnan(m->raw_depth_m)||!std::isnan(m->filtered_depth_m))) ++errors;
      if(!m->fresh && (m->raw_valid||m->pressure_valid||m->depth_valid)) ++errors;
    });
    motor_sub=create_subscription<ui::MotorState>("actuators/thrusters/state",q,[this](ui::MotorState::ConstSharedPtr m){
      if(m->feedback_valid) { ++motor_count; if(synthetic && m->speed_rpm!=-100) ++errors; }
      if(!m->feedback_valid) { ++motor_expired; if(!std::isnan(m->speed_rpm)) ++errors; }
    });
    gateway_sub=create_subscription<ui::GatewayStatus>("gateway/status",10,[this](ui::GatewayStatus::ConstSharedPtr m){
      last_gateway=*m; ++gateway_count; if(m->tx_enabled) ++errors;
    });
    publisher=create_publisher<ui::GatewayFrame>("gateway/frames",q);
  }
  void inject() {
    ui::GatewayFrame f; f.header.stamp=now(); f.connection_id=99999; f.family=0x5b;
    f.flags=8; f.target=1; f.sequence=1; f.command=0x80; f.payload.resize(40); f.payload[0]=3;
    put_float(f.payload,4,1.0f); publisher->publish(f);
    f.command=0x85; f.sequence=2; f.payload.assign(20,0); f.payload[0]=0x30;
    put_float(f.payload,4,12.5f); put_float(f.payload,8,22.5f);
    put_float(f.payload,12,101325.0f); put_float(f.payload,16,101500.0f); publisher->publish(f);
    f.target=2; f.command=0x82; f.payload.assign(32,0); f.payload[0]=3; f.payload[1]=5;
    publisher->publish(f);
    f.family=0x55; f.flags=2; f.can_id=0x201; f.payload.assign(12,0);
    f.payload[0]=0x9c; f.payload[1]=0xff; publisher->publish(f);
  }
  void refresh_raw() {
    ui::GatewayFrame f; f.header.stamp=now(); f.connection_id=99999; f.family=0x5b;
    f.flags=8; f.target=1; f.sequence=++raw_sequence; f.command=0x80; f.payload.resize(40); f.payload[0]=3;
    put_float(f.payload,4,1.0f); publisher->publish(f);
  }
  int result() {
    std::cout<<"imu_raw="<<imu_raw<<" attitude="<<imu_attitude<<" standard_imu="<<standard_imu
      <<" depth="<<depth_count<<" motor_feedback="<<motor_count<<" gateway_status="<<gateway_count
      <<" invalid_value_errors="<<errors<<"\n";
    std::cout<<"imu_barometer="<<imu_baro<<" pressure_msgs="<<baro_pressure
      <<" temperature_msgs="<<baro_temperature<<" barometer_expired="<<baro_expired<<"\n";
    std::cout<<"depth status=0x"<<std::hex<<last_depth.device_status<<std::dec
      <<" pressure_valid="<<last_depth.pressure_valid<<" depth_valid="<<last_depth.depth_valid
      <<" D1="<<last_depth.d1<<" D2="<<last_depth.d2<<"\n";
    bool ok=errors==0 && imu_raw>0 && standard_imu>0 && depth_count>0;
    if(synthetic) {
      std::cout<<"expired: imu="<<imu_expired<<" depth="<<depth_expired<<" motor="<<motor_expired<<"\n";
      ok=ok && motor_count>0 && imu_expired>0 && depth_expired>0 && motor_expired>0;
      ok=ok && imu_raw>1 && imu_baro==1 && baro_pressure==1 && baro_temperature==1 && baro_expired>0;
    } else {
      std::cout<<"gateway connected="<<last_gateway.connected<<" rx="<<last_gateway.rx_bytes
        <<" crc_errors="<<last_gateway.crc_errors<<" shape_errors="<<last_gateway.shape_errors
        <<" startup_or_stale_dropped="<<last_gateway.stale_frames
        <<" heartbeat_age_ms="<<last_gateway.heartbeat_age_ms<<"\n";
      ok=ok && imu_attitude>0 && gateway_count>0 && last_gateway.connected &&
        last_gateway.heartbeat_age_ms>=0 && last_gateway.heartbeat_age_ms<2500;
    }
    std::cout<<(ok?"PASS":"FAIL")<<(synthetic?" synthetic integration":" live receive-only integration")<<"\n";
    return ok?0:1;
  }
  bool synthetic=false;
private:
  static void put_float(std::vector<std::uint8_t> & b,std::size_t offset,float v) {
    std::uint32_t raw; std::memcpy(&raw,&v,4);
    for(int i=0;i<4;++i) b[offset+i]=(raw>>(8*i))&255;
  }
  unsigned imu_raw=0,imu_attitude=0,standard_imu=0,depth_count=0,motor_count=0,gateway_count=0;
  unsigned imu_expired=0,depth_expired=0,motor_expired=0,errors=0;
  unsigned imu_baro=0,baro_pressure=0,baro_temperature=0,baro_expired=0;
  std::uint32_t raw_sequence=2;
  ui::DepthState last_depth; ui::GatewayStatus last_gateway;
  rclcpp::Publisher<ui::GatewayFrame>::SharedPtr publisher;
  rclcpp::Subscription<ui::ImuState>::SharedPtr imu_sub;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr raw_sub;
  rclcpp::Subscription<ui::DepthState>::SharedPtr depth_sub;
  rclcpp::Subscription<ui::MotorState>::SharedPtr motor_sub;
  rclcpp::Subscription<ui::GatewayStatus>::SharedPtr gateway_sub;
  rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr baro_pressure_sub;
  rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr baro_temperature_sub;
};
int main(int argc,char ** argv) {
  rclcpp::init(argc,argv); auto node=std::make_shared<Probe>();
  const auto start=Clock::now(); bool sent=false;
  auto next_raw=start+std::chrono::milliseconds(2200);
  while(rclcpp::ok() && Clock::now()-start<std::chrono::seconds(10)) {
    rclcpp::spin_some(node);
    if(node->synthetic && !sent && Clock::now()-start>std::chrono::seconds(2)) { node->inject(); sent=true; }
    if(node->synthetic && sent && Clock::now()>=next_raw) {
      node->refresh_raw(); next_raw=Clock::now()+std::chrono::milliseconds(200);
    }
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }
  int result=node->result(); node.reset(); rclcpp::shutdown(); return result;
}
