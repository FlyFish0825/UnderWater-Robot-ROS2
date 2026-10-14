# UnderWater-Robot-ROS2 多页面网页版说明文档 (Web Documentation)

本项目在 `doc/` 目录下构建了一套现代、响应式、交互式且**完全支持离线本地直接双击打开**的**多页面网页说明文档系统**。

每个核心章节独立成页，左侧侧边栏、正文导读卡片与底部“上一页/下一页”无缝联动，支持随点随跳。

---

## 页面清单与核心内容

| 文件名 | 页面定位 | 核心涵盖内容 |
|---|---|---|
| [index.html](index.html) | **1. 项目概览与硬件平台** | 硬件规格（Orin NX 16GB Super + STM32H750 + MS5837 + 8 推进器）、系统拓扑、初学者核心闭环理解与各篇导读地图。 |
| [architecture.html](architecture.html) | **2. 系统分层架构与 13 包** | 现代 5 层系统分层视觉流向图（从硬件到上位机）、通信通道与 13 个 ROS 2 源码包全景矩阵与职责。 |
| [nodes.html](nodes.html) | **3. 核心节点详解 (Nodes)** | 10 大核心节点（网关、设备抽象、动力分配、双目视觉、键盘交互等）专属深度卡片、接口矩阵、核心参数与底层机制。 |
| [interfaces.html](interfaces.html) | **4. 话题、服务与消息总线** | `/underwater_robot` 命名空间下全部话题速查矩阵、核心服务接口（使能、归零、校准、控制权）及自定义 IDL 消息字段深度拆解。 |
| [algorithms.html](algorithms.html) | **5. 算法原理与底层协议** | 八推进器二次规划（OSQP）动力分配算法、`thruster_center` 几何中心力矩消除、底层 AA55/58/5B 串口增量流式解包状态机。 |
| [tutorials.html](tutorials.html) | **6. 初学者分步实操教程** | 教程 1~6：环境编译、日常开机监视、双目推流、键盘安全操控（含 preview 隔离）、上位机独占控制权握手、10Hz CSV 记录落盘。 |
| [troubleshooting.html](troubleshooting.html) | **7. 实机排障与推力模拟器** | 常见故障避坑排查手册（串口权限、WCH-Link 干扰、电机死区）以及交互式八推进器推力与 RPM 实时模拟换算器。 |

---

## 如何在本地打开与查看

### 方法 1：直接本地双击打开（最简单推荐）
1. 在 Windows 资源管理器或 Linux 文件管理器中打开项目根目录下的 `doc/` 文件夹；
2. 双击打开 [index.html](index.html) 即可在默认浏览器（Chrome、Edge、Firefox、Safari 等）中直接加载查看；
3. 点击左侧侧边栏或页面底部的翻页卡片，即可在各个章节页面之间**秒级跳转**；
4. 文档采用纯原生静态网页（HTML5 + CSS3 + 纯原生 Vanilla JS），**零外部网络 CDN 依赖**，离线状态下所有交互、图表与样式均 100% 正常工作。

### 方法 2：使用简易本地 HTTP 服务器
如果您在局域网内或希望通过 Web 服务浏览：
```bash
# 在 doc 目录下启动 Python 内置 HTTP 服务器
cd doc
python -m http.server 8080
```
然后在浏览器中访问：`http://127.0.0.1:8080` 即可。
