# UnderWater-Robot-ROS2 网页版说明文档 (Web Documentation)

本项目在 `doc/` 目录下构建了一套现代、响应式、交互式且**完全支持离线本地打开**的网页说明文档系统。

## 如何打开与查看

### 方法 1：直接本地双击打开（最简单推荐）
1. 在 Windows 资源管理器或 Linux 文件管理器中打开项目根目录下的 `doc/` 文件夹；
2. 双击打开 `index.html` 即可在默认浏览器（Chrome、Edge、Firefox、Safari 等）中直接加载查看。
3. 文档采用纯原生静态网页（HTML5 + CSS3 + 纯原生 Vanilla JS），**零外部网络 CDN 依赖**，离线状态下所有交互、图表与样式均 100% 正常工作。

### 方法 2：使用简易本地 HTTP 服务器
如果您在局域网内或希望通过 Web 服务浏览：
```bash
# 在 doc 目录下启动 Python 内置 HTTP 服务器
cd doc
python -m http.server 8080
```
然后在浏览器中访问：`http://127.0.0.1:8080` 即可。

---

## 网页系统涵盖内容索引

1. **项目概览与硬件平台**：主控计算平台 NVIDIA Jetson Orin NX 16GB Super、STM32H750 MCU、MS5837-02BA 深度计、IMU、双目相机与 8 台水下推进器的完整规格与拓扑。
2. **系统分层架构与数据流**：清晰拆解从物理硬件到网关驱动、核心设备、分配控制算法、再到顶层上位机的四层架构流向图。
3. **源码目录与 13 包全景矩阵**：拆解 `src/` 与 `internal/` 下全部 13 个 ROS 2 源码包的定位、语言与职责。
4. **核心节点详解 (Nodes)**：深入解析 `mcu_gateway_node`、`devices_node`、`control_node`、`global_cam_pub` 等常驻后台节点的运行机制与参数。
5. **话题与服务总线 (Topics & Services)**：汇总 `/underwater_robot` 命名空间下全部话题、QoS、发布频率、自定义 IDL 消息字段（如 `ImuState`, `DepthState`, `MotorState` 等）与服务接口（`depth/zero`, `control/enable`, `control_ownership`）。
6. **核心算法原理解析**：
   - 八推进器二次规划（OSQP）最优推力分配模型、URDF 几何中心 `thruster_center` 消除原点力矩偏移原理、1000~6000 RPM 死区与极性映射；
   - 底层串口 AA55 / AA58 / AA5B 帧族有界增量流式解包状态机与单调重同步机制；
   - IMU 姿态相对基准校准与流体静力学深度水面零点采样。
7. **初学者实战教程 (Tutorials)**：
   - 教程 1：环境搭建与全量 `colcon build` 编译；
   - 教程 2：日常开机连接与传感器实时监视；
   - 教程 3：双目相机图像采集与 TCP 9001 网络推流；
   - 教程 4：键盘安全控制与 preview 隔离预览；
   - 教程 5：上位机 Underwater_GUI 与 Orin NX 独占控制权切换（TCP 9000/9002）；
   - 教程 6：10Hz 高速全量传感器 CSV 数据落盘与离线分析。
8. **实机排障速查 (FAQ & Troubleshooting)**：针对串口占用、WCH-Link 干扰 IMU、电机死区保护等常见坑提供实机避坑指南。
9. **交互式小工具**：
   - 交互式推进器转速与推力实时模拟器（拖动滑块直观查看 8 台电机转速响应与死区变化）；
   - 代码块一键复制、实时全局搜索过滤、暗黑/亮色主题切换。
