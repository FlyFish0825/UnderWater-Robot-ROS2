// ==============================================================================
// internal/underwater_control/include/underwater_control/allocator.hpp
// ==============================================================================
// 推力分配接口：Config 描述八个推进器的安装几何与极性映射，Allocator 在
// 六维力/力矩(NED)与各 CAN 节点转速(RPM)之间求解，并给出死区/饱和标志。
// 约定：位置单位 m、力单位 N、力矩参考点为八个安装点的算术质心；
// 物理 wrench 接口固定为 NED(+X 北/+Y 东/+Z 下)，手动档位接口另用 Fx,Fy,Fz,Mx,My,Mz 符号。
// ==============================================================================
// ------------------------------------------------------------------------------
// 【初学者导读】本文件属于 internal/underwater_control（水下控制内部库）的公共头文件，
// 对外暴露“推力分配”这一步：把操作手/上层算法想要的六维力与力矩（wrench，单位 N 与 N·m）
// 换算成 8 个推进器各自的目标转速（RPM）。
// 交互关系：由 control_node.cpp 里的 ControlNode 包含并调用；本文件只做纯计算，
// 不直接收发 ROS 2 话题，因此这里没有 Node/Publisher/Subscription。
// 出现的 ROS 2 概念：本文件本身不涉及节点，但它产出的数值最终会由 control_node.cpp
// 打包成 MotorSetpoint 消息发给网关与电机。
// 出现的 C++ 概念：
//   namespace（命名空间）＝给名字加前缀，避免不同库重名；
//   std::array<T,N>＝长度固定的数组，比 C 数组更安全；
//   struct＝把一组相关字段打包成一个类型；
//   class＝带访问控制（public/private）的结构体；
//   explicit＝禁止隐式类型转换的构造函数修饰。
// ------------------------------------------------------------------------------
#pragma once

// 引入第三方二次规划（QP）求解器接口，以及定长数组和整型头文件。
#include <ThrustAllocationSolver.hpp>
#include <array>
#include <cstdint>

// 本项目所有控制代码都放在 underwater::control 命名空间内。
namespace underwater::control {

// 类型别名：Vec3=三维向量 (x,y,z)，Wrench=六维力/力矩 [Fx,Fy,Fz,Mx,My,Mz]。
using Vec3 = std::array<double, 3>;
using Wrench = std::array<double, 6>;  // [Fx, Fy, Fz, Mx, My, Mz]，单位分别为 N 与 N·m

// 推进器几何与极性配置；位置/坐标单位 m，推力单位 N。
// Config：描述 8 个推进器的“装在哪、朝哪个方向、对应哪个 CAN 节点”，
// 是分配算法唯一的几何/参数来源；改动它就等于改动整台机器的物理模型。
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
// Result：一次分配求解的完整输出。既给出电机转速，也给出“理论/实际”两类 wrench，
// 便于上层做安全判断（例如是否撞到死区或饱和上限）。
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
// linear_rpm：把单桨推力线性折算成带符号的 RPM，并用引用参数回传死区/饱和标志。
// 输入：thrust_n 推力(N)、nominal_max_n 单桨额定推力、min/max 转速边界；
// 输出：目标 RPM 整数；坑位：它会修改 deadband/saturated 两个 bool，调用前需先初始化。
std::int16_t linear_rpm(
    double thrust_n,
    double nominal_max_n,
    int min_rpm,
    int max_rpm,
    bool &deadband,
    bool &saturated);

// 推力分配器类：负责初始化推进器几何模型并调用 QP 二次规划求解器
// Allocator：整个推力分配的核心类。构造时校验并注册几何，solve() 做一次求解。
class Allocator {
public:
  explicit Allocator(Config config);

// solve()：物理 wrench 模式入口；返回 false 表示输入非法或求解失败。
  // 物理 wrench 求解接口 (NED 坐标系：+X 北/+Y 东/+Z 下)
  bool solve(const Wrench &desired, Result &result);

// solve_manual()：手动档位模式入口；输入各轴档位 [-100,100]，内部做等比缩放。
  // 手动档位求解接口 (输入各轴档位范围 [-100, 100]，保持六轴比例协调缩放)
  bool solve_manual(const Wrench &levels, Result &result);

  // 获取配置引用与安装中心几何参考点
// 只读访问器：供节点读取配置和力矩参考点，不会修改对象。
  const Config &config() const { return config_; }
  const Vec3 &wrench_origin() const { return wrench_origin_; }

private:
  // 由八桨模型推力合成六维机体合力与合力矩
// 由 8 个推力值正算回六维 wrench（用于校验与状态上报），纯函数、不修改成员变量。
  Wrench wrench_of(const std::array<double, 8> &force) const;

  // 将推力映射为电机 RPM、重排 CAN 节点并回算等效力矩
// finish()：求解后的收尾——推力→RPM→按 CAN 节点重排→反算等效 wrench。
  void finish(Result &result) const;

// 成员变量：配置、八桨安装点的算术质心（力矩参考点）、底层 QP 求解器。
// 注意 solver_ 不是智能指针而是值成员，随 Allocator 一起构造与析构。
  Config config_;
  Vec3 wrench_origin_{};  // 八个推进器物理安装位置在 base_link 坐标系下的算术质心
  ThrustAllocationSolver solver_;
};

}  // namespace underwater::control
