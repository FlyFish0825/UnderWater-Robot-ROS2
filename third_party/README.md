# ThrustAllocator 依赖

`ThrustAllocator_API_CurrentROV` 来自用户指定的 `F:/code/ThrustAllocator_API_CurrentROV`。
使用源码在 Nano ARM64 上编译，不链接包内 Windows x64 二进制。保留原库及第三方许可证。

实际布局使用用户指定的 `F:/file/BaiduSyncdisk/Project/OSQP/main.cpp`，不使用库自带的
`current_robot_8thruster_config.hpp` 默认布局。两者的 5–8 号推进器位置不同。
原 `main.cpp` SHA-256：`8d8fb714fd86f1e6c075742fc15e054282dd0bdbef06beab69b15c3df08eb45e`。

`500 N` 是临时比例和求解边界，不是实测最大推力。后续通过控制配置和推力–转速模型替换。
