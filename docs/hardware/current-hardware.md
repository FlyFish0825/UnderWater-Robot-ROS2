# 当前实机与待确认信息

- 机器人计算机：NVIDIA Jetson Orin NX / Seeed reComputer，Ubuntu 22.04.5，ROS 2 Humble。
- Nano 项目：/home/lamost/UnderWater-Robot-ROS2。
- MCU USB CDC：0483:5740；当前稳定路径：
  /dev/serial/by-id/usb-STMicroelectronics_Lamost_UnderWater_Robot_ComPort_3588346E3134-if00
- 对应 ttyACM0；lamost 已属于 dialout。启动前确认没有 GUI 或其他进程占用同一串口。
- IMU 和深度计接在 MCU 后端，ROS 节点不另开 IMU 的原生串口或深度计的 I²C。
- 初次部署检查时没有 /dev/video*；相机与照明尚未接入，后续事项见 [README 的后续规划](../../README.md#后续规划)，当前不保留空驱动目录。
- 推进器布局自 2026-10-09 起采用实际 `ROV_Product_FV.SLDASM.urdf`，5～8 位于内侧。
  用户确认模型为北东地（+X 北、+Y 东、+Z 地），fan1..8 对应节点 1..8。
  正转速推力为 1 西南、2 西北、3 东南、4 东北、5/6 竖直向下、7/8 竖直向上；
  `motor_direction=[1,-1,-1,1,1,1,-1,-1]`，详见 [动力分配配置](../deployment/thrust-allocation.md)。
- 力矩基准为八台推进器安装点的算术平均中心：相对实际 URDF base_link 为
  `(-0.012873,0,-0.040532)` m，请求和分配状态使用 thruster_center。
- 未确认：实际最大推力与 RPM 曲线、传感器安装外参、IMU 世界坐标约定、相机内外参。
- 深度计此前实测状态 0x503：在线、RAW/PROM 有效，型号/压力/温度/水深尚未有效。
  这是当时状态，不代表每次启动都如此；以实时 DepthState 为准。

不要将示例设备坐标系名称当作已测量外参。ROS 简化 URDF 定义 base_link 和实际安装点推导的 thruster_center；
分配几何取自用户提供的实际 CAD URDF，不能由此推定 IMU 外参或世界坐标转换已完成。
