// 推力分配接口：Config 描述八个推进器的安装几何与极性映射，Allocator 在
// 六维力/力矩(NED)与各 CAN 节点转速(RPM)之间求解，并给出死区/饱和标志。
// 约定：位置单位 m、力单位 N、力矩参考点为八个安装点的算术质心；
// 物理 wrench 接口固定为 NED(+X 北/+Y 东/+Z 下)，手动档位接口另用 Fx,Fy,Fz,Mx,My,Mz 符号。
#pragma once
#include <ThrustAllocationSolver.hpp>
#include <array>
#include <cstdint>
namespace underwater::control {
using Vec3=std::array<double,3>;
using Wrench=std::array<double,6>;  // [Fx Fy Fz Mx My Mz]，N 与 N·m
// 推进器几何与极性配置；位置/坐标单位 m，推力单位 N。
struct Config {
  // Actual ROV_Product_FV.SLDASM URDF; NED: +X north, +Y east, +Z down.
  std::array<Vec3,8> positions{{
    {0.23016,-0.18359,-0.040532},{0.23016,0.18359,-0.040532},
    {-0.25591,-0.18359,-0.040532},{-0.25591,0.18359,-0.040532},
    {0.062129,-0.125,-0.040532},{0.062129,0.125,-0.040532},
    {-0.087871,-0.125,-0.040532},{-0.087871,0.125,-0.040532}}};
  // Model-positive force axes, independent of positive motor RPM.
  std::array<Vec3,8> axes{{  // 模型正推力方向，与电机正 RPM 无必然关系
    {-0.70711,-0.70711,0},{-0.70711,0.70711,0},
    {0.70711,-0.70711,0},{0.70711,0.70711,0},
    {0,0,1},{0,0,1},{0,0,1},{0,0,1}}};
  std::array<double,8> nominal_max_thrust_n{500,500,500,500,500,500,500,500};  // 单桨额定最大推力 (N)
  std::array<int,8> node_ids{1,2,3,4,5,6,7,8};  // 各几何行对应的 CAN 节点号(1..8)
  // Positive RPM: SW, NW, SE, NE, down, down, up, up.
  std::array<int,8> motor_direction{1,-1,-1,1,1,1,-1,-1};  // 模型正推力→正 RPM 的符号映射
  Wrench weights{1,1,1,1,1,1};  // 六维力/力矩的求解权重
  double effort_weight=1e-6;  // 推力最小化权重，抑制零空间抖动
  int min_rpm=1000,max_rpm=6000;  // 有效转速区间 (RPM)，低于 min_rpm 视为死区
  int manual_start_rpm=1200;  // 手动模式起始转速，保证越过死区
  // Operator directions: Fx, Fy, Fz, Mx, My, Mz. Physical-wrench API remains NED.
  std::array<int,6> manual_axis_signs{-1,1,1,-1,1,-1};  // 操作者直觉方向到 NED 的符号
};
// 分配结果；thrust 为模型正方向推力(N)，rpm 已按 CAN 节点号排序。
struct Result {
  std::array<double,8> thrust{};
  std::array<std::int16_t,8> rpm{}; // CAN node order.
  Wrench allocated{},modeled_rpm{},effective_request{};  // 实际合力/等效 RPM 合力/归一化后的请求
  bool deadband=false,saturated=false;  // 是否触发 RPM 死区/饱和
};
// 线性推力-RPM 曲线：返回带符号 RPM，置死区与饱和标志。
std::int16_t linear_rpm(double thrust_n,double nominal_max_n,int min_rpm,int max_rpm,
  bool & deadband,bool & saturated);
// 推力分配器：构造时校验几何与极性并初始化 QP 求解器。
class Allocator {
public:
  explicit Allocator(Config config);
  bool solve(const Wrench &,Result &);  // 物理 wrench 接口(NED)
  // Keyboard levels in [-100,100], separate from the physical-wrench API.
  bool solve_manual(const Wrench & levels,Result &);  // 手动档位接口，-100..100
  const Config & config() const { return config_; }
  const Vec3 & wrench_origin() const { return wrench_origin_; }
private:
  Wrench wrench_of(const std::array<double,8> &) const;  // 由八桨模型推力合成六维 wrench
  void finish(Result &) const;  // 生成 RPM、映射节点并回算等效 wrench
  Config config_;
  Vec3 wrench_origin_{}; // Arithmetic centroid of all eight installation points in base_link.
  ThrustAllocationSolver solver_;
};
}
