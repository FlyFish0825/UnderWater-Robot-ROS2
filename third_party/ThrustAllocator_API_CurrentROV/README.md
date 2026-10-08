# ThrustAllocator API - Current ROV

这是当前 8 推进器 ROV 的离线可构建交付包。

包含：
- C API：`include/thrust_allocator_c.h`
- C++ API：`include/ThrustAllocationSolver.hpp`
- 当前机器人几何预设：`include/current_robot_8thruster_config.hpp`
- 核心实现：`src/`
- Eigen / OSQP / QDLDL / OsqpEigen 本地源码：`Middlewares/Third_Party/`
- Windows / Linux 构建脚本
- 独立 consumer 示例：`example/consumer/`

## 当前机器人配置核对

几何来源：`ROV_Product_FV.SLDASM` 对应 URDF。
URDF 的推进器 joint 轴 `0 0 -1` 按 `origin rpy` 变换到 `base_link` 后：
- 1~4 号为水平 45° 布置；
- 5~8 号均为 `+Z`；
- 8 个位置见 `config/current_robot_thrusters.csv`。

原 overlay 示例中 1~4 号位置/方向正确，5~8 号方向正确，
但 5~8 号位置不是该 URDF 的几何；本交付包已改成 URDF 数值。
修正后的 6x8 几何分配矩阵 rank=6，可覆盖 6 自由度。

## URDF 没有给出的参数

URDF 只能确认安装位置和轴方向，不能确认：
- 单推进器实际最大推力 `max_thrust_N`
- 正反向推力是否对称
- 螺旋桨反扭矩系数/方向
- 推力随转速、电压、流速的非线性

`example/example.cpp` 中 `500 N` 仅用于构建与 API 演示，不能当真实标定值。

## Windows 一键构建

```bat
scripts\build_windows.bat
```

若存在 `D:\w64devkit\bin\gcc.exe`，脚本优先使用该 MinGW；
否则使用当前 CMake/Ninja 能找到的编译器。

输出：
```text
dist/windows-x64-static/
  include/
  lib/libthrust_allocator.a
  lib/cmake/ThrustAllocator/
```

运行自带示例：
```bat
build\windows-static\bin\thrust_allocator_example.exe
```

## 在其他 CMake 工程中使用

```cmake
find_package(ThrustAllocator CONFIG REQUIRED)
target_link_libraries(your_target PRIVATE ThrustAllocator::thrust_allocator)
```

配置 consumer 时指定：
```bat
-DThrustAllocator_DIR=<SDK>/lib/cmake/ThrustAllocator
```

`example/consumer/` 是完整独立消费端工程。

## 求解语义

每个推进器变量为归一化命令 `k in [-1,1]`。
`force=max_thrust*axis`，`torque=position x force + reaction torque`。
QP 为软跟踪形式，目标超出能力时饱和到最接近的可行解。

第三方源码及各自 LICENSE 文件随包保留；再分发前请按项目要求审阅许可证。
