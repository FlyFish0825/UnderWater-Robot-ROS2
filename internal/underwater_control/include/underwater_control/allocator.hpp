// ==============================================================================
// internal/underwater_control/include/underwater_control/allocator.hpp
// ==============================================================================
// 推力分配接口：Config 描述八个推进器的安装几何与极性映射，Allocator 在
// 六维力/力矩(NED)与各 CAN 节点转速(RPM)之间求解，并给出死区/饱和标志。
// 约定：位置单位 m、力单位 N、力矩参考点为八个安装点的算术质心；
// 物理 wrench 接口固定为 NED(+X 北/+Y 东/+Z 下)，手动档位接口另用 Fx,Fy,Fz,Mx,My,Mz 符号。
// ==============================================================================
#pragma once

#include <ThrustAllocationSolver.hpp>
#include <array>
#include <cstdint>

namespace underwater::control {

using Vec3 = std::array<double, 3>;
using Wrench = std::array<double, 6>;  // [Fx, Fy, Fz, Mx, My, Mz]，单位分别为 N 与 N·m

// 推进器几何与极性配置；位置/坐标单位 m，推力单位 N。
struct Config {
  // Actual ROV_Product_FV.SLDASM URDF; NED: +X north, +Y east, +Z down.
  // 八个推进器在机体坐标系(base_link)下的三维物理安装坐标 (x, y, z)，单位：米 (m)
  std::array<Vec3, 8> positions{{
    { 0.23016, -0.18359, -0.040532},
    { 0.23016,  0.18359, -0.040532},
    {-0.25591, -0.18359, -0.040532},
    {-0.25591,  0.18359, -0.040532},
    { 0.062129, -0.125,   -0.040532},
    { 0.062129,  0.125,   -0.040532},
    {-0.087871, -0.125,   -0.040532},
    {-0.087871,  0.125,   -0.040532}
  }};

  // Model-positive force axes, independent of positive motor RPM.
  // 各推进器在模型坐标系中的正推力方向单位向量（与电机实际旋转方向解耦）
  std::array<Vec3, 8> axes{{
    {-0.70711, -0.70711, 0.0},
    {-0.70711,  0.70711, 0.0},
    { 0.70711, -0.70711, 0.0},
    { 0.70711,  0.70711, 0.0},
    { 0.0,      0.0,      1.0},
    { 0.0,      0.0,      1.0},
    { 0.0,      0.0,      1.0},
    { 0.0,      0.0,      1.0}
  }};

  // 单桨额定最大推力 (N)
  std::array<double, 8> nominal_max_thrust_n{500, 500, 500, 500, 500, 500, 500, 500};

  // 各几何行对应的 CAN 节点编号 (必须是 1..8 的全排列)
  std::array<int, 8> node_ids{1, 2, 3, 4, 5, 6, 7, 8};

  // Positive RPM: SW, NW, SE, NE, down, down, up, up.
  // 模型正推力 -> 电机正向转速(RPM)的极性符号映射 (+1 或 -1)
  std::array<int, 8> motor_direction{1, -1, -1, 1, 1, 1, -1, -1};

  // 六维力/力矩二次型优化的权重因子 [Fx, Fy, Fz, Mx, My, Mz]
  Wrench weights{1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

  // 控制能耗最小化权重，用于抑制推力分配零空间内的不必要震荡
  double effort_weight = 1e-6;

  // 电机转速安全工作区间 (RPM)，低于 min_rpm 视为死区停转，高于 max_rpm 进行钳位
  int min_rpm = 1000;
  int max_rpm = 6000;

  // 手动操作模式起步转速，确保档位动作越过电机机械死区
  int manual_start_rpm = 1200;

  // Operator directions: Fx, Fy, Fz, Mx, My, Mz. Physical-wrench API remains NED.
  // 操作手直觉输入坐标轴到物理 NED 坐标系的符号映射
  std::array<int, 6> manual_axis_signs{-1, 1, 1, -1, 1, -1};
};

// 分配结果结构体；thrust 为模型正方向推力(N)，rpm 已按 CAN 节点号顺序排列。
struct Result {
  std::array<double, 8> thrust{};                          // 求解器计算出的 8 个推进器推力 (N)
  std::array<std::int16_t, 8> rpm{};                       // 转换后的 8 节点电机目标转速 (RPM, 按 CAN ID 1..8 升序排列)
  Wrench allocated{};                                      // 由分配推力实际合成的六维物理力/力矩
  Wrench modeled_rpm{};                                    // 经转速离散化及死区/饱和截断后反算出的等效力/力矩
  Wrench effective_request{};                              // 经归一化或缩放后的实际有效期望力/力矩
  bool deadband = false;                                   // 标记是否存在落入机械死区而被置零的通道
  bool saturated = false;                                  // 标记是否存在超出额定转速上限而饱和截断的通道
};

// 线性推力-RPM 曲线映射函数：返回带符号的目标转速，并维护死区与饱和标志
std::int16_t linear_rpm(
    double thrust_n,
    double nominal_max_n,
    int min_rpm,
    int max_rpm,
    bool &deadband,
    bool &saturated);

// 推力分配器类：负责初始化推进器几何模型并调用 QP 二次规划求解器
class Allocator {
public:
  explicit Allocator(Config config);

  // 物理 wrench 求解接口 (NED 坐标系：+X 北/+Y 东/+Z 下)
  bool solve(const Wrench &desired, Result &result);

  // 手动档位求解接口 (输入各轴档位范围 [-100, 100]，保持六轴比例协调缩放)
  bool solve_manual(const Wrench &levels, Result &result);

  // 获取配置引用与安装中心几何参考点
  const Config &config() const { return config_; }
  const Vec3 &wrench_origin() const { return wrench_origin_; }

private:
  // 由八桨模型推力合成六维机体合力与合力矩
  Wrench wrench_of(const std::array<double, 8> &force) const;

  // 将推力映射为电机 RPM、重排 CAN 节点并回算等效力矩
  void finish(Result &result) const;

  Config config_;
  Vec3 wrench_origin_{};  // 八个推进器物理安装位置在 base_link 坐标系下的算术质心
  ThrustAllocationSolver solver_;
};

}  // namespace underwater::control
