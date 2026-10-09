#include "underwater_control/allocator.hpp"
#include <chrono>
#include <algorithm>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <underwater_interfaces/msg/allocation_status.hpp>
#include <underwater_interfaces/msg/gateway_status.hpp>
#include <underwater_interfaces/msg/motor_setpoint.hpp>
namespace uc=underwater::control;
namespace ui=underwater_interfaces::msg;
using Clock=std::chrono::steady_clock;
using namespace std::chrono_literals;
class ControlNode final:public rclcpp::Node {
public:
  ControlNode():Node("thrust_allocator") {
    rcl_interfaces::msg::ParameterDescriptor d; d.read_only=true;
    uc::Config cfg;
    auto numbers=[&](const char * name,const auto & initial) {
      return declare_parameter<std::vector<double>>(name,std::vector<double>(initial.begin(),initial.end()),d);
    };
    std::vector<double> positions,axes;
    for(std::size_t i=0;i<8;++i) {
      positions.insert(positions.end(),cfg.positions[i].begin(),cfg.positions[i].end());
      axes.insert(axes.end(),cfg.axes[i].begin(),cfg.axes[i].end());
    }
    positions=declare_parameter<std::vector<double>>("positions_m",positions,d);
    axes=declare_parameter<std::vector<double>>("axes",axes,d);
    auto force=numbers("nominal_max_thrust_n",cfg.nominal_max_thrust_n);
    auto weights=numbers("wrench_weights",cfg.weights);
    auto ids=declare_parameter<std::vector<std::int64_t>>("node_ids",{1,2,3,4,5,6,7,8},d);
    auto dirs=declare_parameter<std::vector<std::int64_t>>("motor_direction",
      std::vector<std::int64_t>(cfg.motor_direction.begin(),cfg.motor_direction.end()),d);
    if(positions.size()!=24 || axes.size()!=24 || force.size()!=8 || weights.size()!=6 || ids.size()!=8 || dirs.size()!=8)
      throw std::invalid_argument("allocation configuration array size mismatch");
    for(std::size_t i=0;i<8;++i) {
      if(ids[i]<1 || ids[i]>8 || (dirs[i]!=1 && dirs[i]!=-1)) throw std::invalid_argument("invalid node map");
      cfg.node_ids[i]=ids[i]; cfg.motor_direction[i]=dirs[i]; cfg.nominal_max_thrust_n[i]=force[i];
      for(int j=0;j<3;++j) { cfg.positions[i][j]=positions[3*i+j]; cfg.axes[i][j]=axes[3*i+j]; }
    }
    std::copy(weights.begin(),weights.end(),cfg.weights.begin());
    cfg.min_rpm=declare_parameter<int>("min_rpm",1000,d);
    cfg.max_rpm=declare_parameter<int>("max_rpm",6000,d);
    cfg.manual_start_rpm=declare_parameter<int>("manual_start_rpm",1200,d);
    const auto manual_signs=declare_parameter<std::vector<std::int64_t>>("manual_axis_signs",
      std::vector<std::int64_t>(cfg.manual_axis_signs.begin(),cfg.manual_axis_signs.end()),d);
    if(manual_signs.size()!=6) throw std::invalid_argument("manual_axis_signs must contain six signs");
    for(std::size_t j=0;j<6;++j) {
      if(manual_signs[j]!=1 && manual_signs[j]!=-1) throw std::invalid_argument("manual_axis_signs must be +/-1");
      cfg.manual_axis_signs[j]=manual_signs[j];
    }
    cfg.effort_weight=declare_parameter<double>("effort_weight",1e-6,d);
    timeout_=std::chrono::milliseconds(declare_parameter<int>("input_timeout_ms",500,d));
    if(timeout_.count()<100 || timeout_.count()>1000) throw std::invalid_argument("input timeout must be 100..1000 ms");
    frame_id_=declare_parameter<std::string>("wrench_frame_id","thruster_center",d);
    allocator_=std::make_unique<uc::Allocator>(cfg);
    output_=create_publisher<ui::MotorSetpoint>("actuators/thrusters/setpoint",rclcpp::QoS(1));
    status_=create_publisher<ui::AllocationStatus>("control/allocation",10);
    input_=create_subscription<geometry_msgs::msg::WrenchStamped>("control/wrench_request",rclcpp::QoS(1),
      [this](geometry_msgs::msg::WrenchStamped::ConstSharedPtr w){ wrench(*w,false); });
    manual_input_=create_subscription<geometry_msgs::msg::WrenchStamped>("control/manual_request",rclcpp::QoS(1),
      [this](geometry_msgs::msg::WrenchStamped::ConstSharedPtr w){ wrench(*w,true); });
    gateway_sub_=create_subscription<ui::GatewayStatus>("gateway/status",10,
      [this](ui::GatewayStatus::ConstSharedPtr s){
        if(gateway_ && gateway_->connection_id!=s->connection_id) disable("serial connection changed");
        gateway_=*s; gateway_at_=Clock::now();
        if(armed_ && (!s->connected || !s->tx_enabled)) disable("gateway unavailable");
        if(armed_ && s->motor_session_id==session_) {
          if(s->motor_rearm_required && acknowledged_) disable("gateway requires rearm");
          else if(!s->motor_rearm_required) acknowledged_=true;
        }
      });
    enable_=create_service<std_srvs::srv::SetBool>("control/enable",
      [this](std_srvs::srv::SetBool::Request::SharedPtr req,std_srvs::srv::SetBool::Response::SharedPtr res){
        if(!req->data) { disable("operator disabled"); publish_command(); res->success=true; res->message="Disabled; stop requested"; return; }
        if(!gateway_ready()) { res->success=false; res->message="Gateway must be connected, settled, heartbeating and enable_motor_tx=true"; return; }
        if(command_sources()>1) { res->success=false; res->message="More than one manual/wrench publisher"; return; }
        session_=std::max(session_+1,static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count()));
        sequence_=0; armed_=true; acknowledged_=false; received_after_enable_=false;
        last_stamp_.reset(); input_valid_=false; enabled_at_=Clock::now(); enabled_stamp_=now();
        detail_="enabled; zero handshake and fresh wrench required"; publish_command();
        res->success=true; res->message=detail_;
      });
    timer_=create_wall_timer(20ms,[this]{ tick(); });
    RCLCPP_INFO(get_logger(),"Allocator ready: actual URDF geometry in NED, confirmed motor polarity, provisional linear RPM curve, disarmed");
    RCLCPP_INFO(get_logger(),"Manual axes use configured signs and balanced startup; physical wrench remains NED");
    const auto & center=allocator_->wrench_origin();
    RCLCPP_INFO(get_logger(),"Wrench origin: thruster centroid [%.6f %.6f %.6f] m in base_link",center[0],center[1],center[2]);
  }
  ~ControlNode() override {
    try { disable("shutdown"); if(rclcpp::ok()) publish_command(); } catch(...) {}
  }
private:
  std::size_t command_sources() const {
    return count_publishers("control/wrench_request")+count_publishers("control/manual_request");
  }
  bool gateway_ready() const {
    return gateway_ && Clock::now()-gateway_at_<2500ms && gateway_->connected &&
      !gateway_->settling && gateway_->tx_enabled && gateway_->heartbeat_age_ms>=0 && gateway_->heartbeat_age_ms<2500;
  }
  bool fresh() const { return input_valid_ && Clock::now()-input_at_<timeout_; }
  void disable(const std::string & reason) {
    armed_=false; acknowledged_=false; received_after_enable_=false; detail_=reason;
  }
  void wrench(const geometry_msgs::msg::WrenchStamped & w,bool manual) {
    const rclcpp::Time stamp(w.header.stamp);
    const auto age=(now()-stamp).nanoseconds();
    if(w.header.frame_id!=frame_id_ || age< -100000000LL || age>250000000LL ||
       (last_stamp_ && stamp<=*last_stamp_) || command_sources()>1) {
      input_valid_=false; disable("invalid frame, stale/duplicate timestamp or multiple publishers"); publish_command(); return;
    }
    last_stamp_=stamp;
    desired_={w.wrench.force.x,w.wrench.force.y,w.wrench.force.z,
      w.wrench.torque.x,w.wrench.torque.y,w.wrench.torque.z};
    manual_mode_=manual;
    solver_ok_=manual?allocator_->solve_manual(desired_,result_):allocator_->solve(desired_,result_);
    if(solver_ok_)desired_=result_.effective_request;
    input_valid_=solver_ok_;
    input_at_=Clock::now()-std::chrono::nanoseconds(std::max<std::int64_t>(0,age));
    if(!solver_ok_) {
      disable(manual?"invalid manual request or balanced startup infeasible within RPM bounds":"invalid wrench or QP solve failed");
      publish_command(); return;
    }
    if(armed_ && stamp>=enabled_stamp_) received_after_enable_=true;
    if(!armed_) detail_=manual?"manual preview; output is stopped":"wrench preview; output is stopped";
  }
  void publish_command() {
    if(!gateway_) return;
    ui::MotorSetpoint m; m.header.stamp=now(); m.header.frame_id=frame_id_;
    m.connection_id=gateway_->connection_id; m.session_id=session_; m.sequence=++sequence_;
    m.node_mask=0xff; m.armed=armed_;
    if(armed_ && acknowledged_ && received_after_enable_ && fresh() && gateway_ready()) {
      m.rpm=result_.rpm;
      for(std::size_t i=0;i<8;++i) if(m.rpm[i]!=0) m.run_mask|=1u<<i;
      detail_=m.run_mask?(manual_mode_?"streaming manual throttle RPM":"streaming allocated RPM"):"all outputs inside deadband or zero";
    }
    output_->publish(m);
  }
  void tick() {
    if(armed_) {
      if(!gateway_ready()) disable("gateway heartbeat/connection expired");
      else if(!acknowledged_ && Clock::now()-enabled_at_>2500ms) disable("zero handshake timed out");
      else if(received_after_enable_ && !fresh()) disable("wrench input timed out; enable again");
      else if(command_sources()>1) disable("multiple command publishers");
    }
    publish_command();
    if(++ticks_%5) return;
    ui::AllocationStatus s; s.header.stamp=now(); s.header.frame_id=frame_id_;
    s.armed=armed_; s.input_fresh=fresh(); s.gateway_ready=gateway_ready(); s.solver_ok=solver_ok_;
    s.session_id=session_; s.requested_wrench=desired_; s.thruster_force_n=result_.thrust;
    s.allocated_wrench=result_.allocated; s.modeled_rpm_wrench=result_.modeled_rpm;
    s.proposed_motor_rpm=result_.rpm; s.rpm_deadband_applied=result_.deadband;
    s.rpm_saturated=result_.saturated; s.detail=detail_; status_->publish(s);
  }
  std::unique_ptr<uc::Allocator> allocator_;
  uc::Result result_; uc::Wrench desired_{};
  std::string frame_id_,detail_="disarmed";
  std::chrono::milliseconds timeout_{500};
  std::optional<ui::GatewayStatus> gateway_;
  std::optional<rclcpp::Time> last_stamp_;
  rclcpp::Time enabled_stamp_{0,0,RCL_ROS_TIME};
  Clock::time_point gateway_at_{},input_at_{},enabled_at_{};
  std::uint64_t session_=std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count(),sequence_=0;
  bool armed_=false,acknowledged_=false,received_after_enable_=false,input_valid_=false,solver_ok_=false;
  bool manual_mode_=false;
  unsigned ticks_=0;
  rclcpp::Publisher<ui::MotorSetpoint>::SharedPtr output_;
  rclcpp::Publisher<ui::AllocationStatus>::SharedPtr status_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr input_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr manual_input_;
  rclcpp::Subscription<ui::GatewayStatus>::SharedPtr gateway_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char ** argv) {
  rclcpp::init(argc,argv);
  try { rclcpp::spin(std::make_shared<ControlNode>()); }
  catch(const std::exception & e) { RCLCPP_ERROR(rclcpp::get_logger("thrust_allocator"),"%s",e.what()); rclcpp::shutdown(); return 1; }
  rclcpp::shutdown(); return 0;
}
