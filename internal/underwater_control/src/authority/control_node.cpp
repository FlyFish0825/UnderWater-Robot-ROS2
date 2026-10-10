// 推力分配节点：把 geometry_msgs/WrenchStamped 请求（物理 wrench 或手动档位）
// 经 Allocator 求解后发布 MotorSetpoint，并向网关确认会话健康后才允许输出。
// 安全门限：必须显式 control/enable、网关已连接且 tx_enabled、零点握手完成、
// wrench 输入新鲜且只有一个发布者；任一条件失效立即解除武装并发送停机指令。
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
    positions=declare_parameter<std::vector<double>>("positions_m",positions,d);  // 24 个数：8 桨 × (x,y,z)，单位 m
    axes=declare_parameter<std::vector<double>>("axes",axes,d);  // 24 个数：8 桨 × 单位方向
    auto force=numbers("nominal_max_thrust_n",cfg.nominal_max_thrust_n);  // 8 个额定推力 (N)
    auto weights=numbers("wrench_weights",cfg.weights);  // 6 个 wrench 权重
    auto ids=declare_parameter<std::vector<std::int64_t>>("node_ids",{1,2,3,4,5,6,7,8},d);
    auto dirs=declare_parameter<std::vector<std::int64_t>>("motor_direction",
      std::vector<std::int64_t>(cfg.motor_direction.begin(),cfg.motor_direction.end()),d);
    if(positions.size()!=24 || axes.size()!=24 || force.size()!=8 || weights.size()!=6 || ids.size()!=8 || dirs.size()!=8)
      throw std::invalid_argument("allocation configuration array size mismatch");  // 尺寸不符即拒绝启动，避免半配置运行
    for(std::size_t i=0;i<8;++i) {
      if(ids[i]<1 || ids[i]>8 || (dirs[i]!=1 && dirs[i]!=-1)) throw std::invalid_argument("invalid node map");
      cfg.node_ids[i]=ids[i]; cfg.motor_direction[i]=dirs[i]; cfg.nominal_max_thrust_n[i]=force[i];
      for(int j=0;j<3;++j) { cfg.positions[i][j]=positions[3*i+j]; cfg.axes[i][j]=axes[3*i+j]; }  // 扁平数组按行还原
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
    timeout_=std::chrono::milliseconds(declare_parameter<int>("input_timeout_ms",500,d));  // wrench 输入看门狗
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
        if(gateway_ && gateway_->connection_id!=s->connection_id) disable("serial connection changed");  // 串口重连即解除武装
        gateway_=*s; gateway_at_=Clock::now();
        if(armed_ && (!s->connected || !s->tx_enabled)) disable("gateway unavailable");
        if(armed_ && s->motor_session_id==session_) {
          if(s->motor_rearm_required && acknowledged_) disable("gateway requires rearm");  // 网关要求重新武装
          else if(!s->motor_rearm_required) acknowledged_=true;  // 零点握手完成
        }
      });
    enable_=create_service<std_srvs::srv::SetBool>("control/enable",
      [this](std_srvs::srv::SetBool::Request::SharedPtr req,std_srvs::srv::SetBool::Response::SharedPtr res){
        if(!req->data) { disable("operator disabled"); publish_command(); res->success=true; res->message="Disabled; stop requested"; return; }
        if(!gateway_ready()) { res->success=false; res->message="Gateway must be connected, settled, heartbeating and enable_motor_tx=true"; return; }
        if(command_sources()>1) { res->success=false; res->message="More than one manual/wrench publisher"; return; }  // 防止多源争抢
        session_=std::max(session_+1,static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count()));  // 会话号单调且唯一
        sequence_=0; armed_=true; acknowledged_=false; received_after_enable_=false;
        last_stamp_.reset(); input_valid_=false; enabled_at_=Clock::now(); enabled_stamp_=now();
        detail_="enabled; zero handshake and fresh wrench required"; publish_command();
        res->success=true; res->message=detail_;
      });
    timer_=create_wall_timer(20ms,[this]{ tick(); });  // 20 ms 看门狗与指令心跳
    RCLCPP_INFO(get_logger(),"Allocator ready: actual URDF geometry in NED, confirmed motor polarity, provisional linear RPM curve, disarmed");
    RCLCPP_INFO(get_logger(),"Manual axes use configured signs and balanced startup; physical wrench remains NED");
    const auto & center=allocator_->wrench_origin();
    RCLCPP_INFO(get_logger(),"Wrench origin: thruster centroid [%.6f %.6f %.6f] m in base_link",center[0],center[1],center[2]);
  }
  ~ControlNode() override {
    try { disable("shutdown"); if(rclcpp::ok()) publish_command(); } catch(...) {}  // 退出前尽力发一次停机
  }
private:
  std::size_t command_sources() const {
    return count_publishers("control/wrench_request")+count_publishers("control/manual_request");  // 两个入口合计发布者数
  }
  bool gateway_ready() const {
    return gateway_ && Clock::now()-gateway_at_<2500ms && gateway_->connected &&  // 状态自身 2.5 s 内新鲜
      !gateway_->settling && gateway_->tx_enabled && gateway_->heartbeat_age_ms>=0 && gateway_->heartbeat_age_ms<2500;  // 心跳 2.5 s 内
  }
  bool fresh() const { return input_valid_ && Clock::now()-input_at_<timeout_; }  // wrench 输入是否在超时窗口内
  void disable(const std::string & reason) {
    armed_=false; acknowledged_=false; received_after_enable_=false; detail_=reason;
  }
  void wrench(const geometry_msgs::msg::WrenchStamped & w,bool manual) {
    const rclcpp::Time stamp(w.header.stamp);
    const auto age=(now()-stamp).nanoseconds();
    // 校验坐标系、时钟容差(未来 100 ms/过去 250 ms)、时间戳严格递增以及单一发布者。
    if(w.header.frame_id!=frame_id_ || age< -100000000LL || age>250000000LL ||
       (last_stamp_ && stamp<=*last_stamp_) || command_sources()>1) {
      input_valid_=false; disable("invalid frame, stale/duplicate timestamp or multiple publishers"); publish_command(); return;
    }
    last_stamp_=stamp;
    desired_={w.wrench.force.x,w.wrench.force.y,w.wrench.force.z,
      w.wrench.torque.x,w.wrench.torque.y,w.wrench.torque.z};  // 顺序 Fx Fy Fz Mx My Mz
    manual_mode_=manual;
    solver_ok_=manual?allocator_->solve_manual(desired_,result_):allocator_->solve(desired_,result_);
    if(solver_ok_)desired_=result_.effective_request;  // 记录求解器实际使用的请求
    input_valid_=solver_ok_;
    input_at_=Clock::now()-std::chrono::nanoseconds(std::max<std::int64_t>(0,age));  // 以输入时间戳推算超时起点
    if(!solver_ok_) {
      disable(manual?"invalid manual request or balanced startup infeasible within RPM bounds":"invalid wrench or QP solve failed");
      publish_command(); return;
    }
    if(armed_ && stamp>=enabled_stamp_) received_after_enable_=true;  // 只认武装之后的时间戳
    if(!armed_) detail_=manual?"manual preview; output is stopped":"wrench preview; output is stopped";
  }
  void publish_command() {
    if(!gateway_) return;  // 未收到网关状态前不发布
    ui::MotorSetpoint m; m.header.stamp=now(); m.header.frame_id=frame_id_;
    m.connection_id=gateway_->connection_id; m.session_id=session_; m.sequence=++sequence_;  // 每条指令序号自增
    m.node_mask=0xff; m.armed=armed_;  // 八个节点全部纳入寻址
    if(armed_ && acknowledged_ && received_after_enable_ && fresh() && gateway_ready()) {  // 全部安全条件满足才下发 RPM
      m.rpm=result_.rpm;
      for(std::size_t i=0;i<8;++i) if(m.rpm[i]!=0) m.run_mask|=1u<<i;  // run_mask 标记非零转速通道
      detail_=m.run_mask?(manual_mode_?"streaming manual throttle RPM":"streaming allocated RPM"):"all outputs inside deadband or zero";
    }
    output_->publish(m);
  }
  void tick() {
    if(armed_) {
      if(!gateway_ready()) disable("gateway heartbeat/connection expired");
      else if(!acknowledged_ && Clock::now()-enabled_at_>2500ms) disable("zero handshake timed out");  // 2.5 s 未完成握手即放弃
      else if(received_after_enable_ && !fresh()) disable("wrench input timed out; enable again");
      else if(command_sources()>1) disable("multiple command publishers");
    }
    publish_command();
    if(++ticks_%5) return;  // 每 5 个周期（100 ms）发布一次状态
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
  std::optional<ui::GatewayStatus> gateway_;  // 最近一次网关状态
  std::optional<rclcpp::Time> last_stamp_;  // 上一条 wrench 时间戳，用于去重
  rclcpp::Time enabled_stamp_{0,0,RCL_ROS_TIME};
  Clock::time_point gateway_at_{},input_at_{},enabled_at_{};
  std::uint64_t session_=std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count(),sequence_=0;  // 会话号初值取启动时刻
  bool armed_=false,acknowledged_=false,received_after_enable_=false,input_valid_=false,solver_ok_=false;
  bool manual_mode_=false;  // 当前请求来自手动档位话题
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
