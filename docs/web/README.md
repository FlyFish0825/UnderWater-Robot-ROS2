# UnderWater-Robot-ROS2 多页面网页版说明文档 (Web Documentation)

本目录（`docs/web/`）构建了一套现代、响应式、交互式且**完全支持离线本地直接双击打开**的**多页面网页说明文档系统**。

> ℹ️ 该网页文档原位于仓库根目录的 `doc/`，现已迁入 `docs/` 文档体系下的 `docs/web/`，与 `docs/` 中的 Markdown 文档（architecture / interfaces / usage / protocols / hardware / experiments）统一管理。

每个核心章节独立成页，左侧侧边栏、正文导读卡片与底部“上一页/下一页”无缝联动，支持随点随跳。

---

## 页面清单与核心内容

| 文件名 | 页面定位 | 核心涵盖内容 |
|---|---|---|
| [index.html](index.html) | **1. 项目概览与硬件平台** | 硬件规格（Orin NX 16GB Super + STM32H750 + MS5837 + 8 推进器）、系统拓扑、初学者核心闭环理解与各篇导读地图。 |
| [signal-flow.html](signal-flow.html) | **2. 系统信号流图与项目大纲** | 项目大纲结构树（`src/` 6 包 + `internal/` 7 包 + 支撑目录）、端到端信号流图（感知 → MCU → 串口网关 → 设备语义 → 控制分配 → 推进器，含 TCP 图像/遥测旁路）、12 条关键话题信号链、控制权与安全信号。 |
| [architecture.html](architecture.html) | **3. 系统分层架构与 13 包** | 现代 5 层系统分层视觉流向图（从硬件到上位机）、通信通道与 13 个 ROS 2 源码包全景矩阵与职责。 |
| [nodes.html](nodes.html) | **4. 核心节点详解 (Nodes)** | 10 大核心节点（网关、设备抽象、动力分配、双目视觉、键盘交互等）专属深度卡片、接口矩阵、核心参数与底层机制。 |
| [interfaces.html](interfaces.html) | **5. 话题、服务与消息总线** | `/underwater_robot` 命名空间下全部话题速查矩阵、核心服务接口（使能、归零、校准、控制权）及自定义 IDL 消息字段深度拆解。 |
| [algorithms.html](algorithms.html) | **6. 算法原理与底层协议** | 八推进器二次规划（OSQP）动力分配算法、`thruster_center` 几何中心力矩消除、底层 AA55/58/5B 串口增量流式解包状态机。 |
| [tutorials.html](tutorials.html) | **7. 初学者分步实操教程** | 教程 1~6：环境编译、日常开机监视、双目推流、键盘安全操控（含 preview 隔离）、上位机独占控制权握手、10Hz CSV 记录落盘。 |
| [troubleshooting.html](troubleshooting.html) | **8. 实机排障与推力模拟器** | 常见故障避坑排查手册（串口权限、WCH-Link 干扰、电机死区）以及交互式八推进器推力与 RPM 实时模拟换算器。 |
| [viewer.html](viewer.html) | **仓库 Markdown 在线阅读器** | 将仓库内全部 `.md` 文档渲染为网页，供侧边栏“仓库 README / 更新发布日志”等入口使用。 |

---

## 仓库 Markdown 在线阅读（viewer.html）

侧边栏“快捷外部资源”里的 **仓库 README**、**更新发布日志** 不再是原始 `.md` 文本，而是经由本站在线渲染：

- 入口形如 `viewer.html?doc=README.md`，`doc` 为**仓库相对路径**（如 `docs/usage/01-installation.md`）；
- 文档内的其它 `.md` 链接会被自动改写到同一个阅读器，图片、YAML 等其它相对资源则按仓库根目录解析；
- 渲染器 [md-viewer.js](md-viewer.js) 为零依赖纯前端实现，**不发起任何网络请求**，因此在 `file://` 双击打开时同样可用（浏览器禁止 `file://` 下 fetch 本地文件，故不能采用“运行时读取 md”的方案）；
- 文档内容由 [md-data.js](md-data.js) 内联提供，该文件是**构建产物**，来源是仓库内所有 Markdown。

新增或修改 Markdown 后，请在仓库根目录重新生成数据：

```bash
node tools/generate-web-md-data.mjs
```

---

## 如何在本地打开与查看

### 方法 1：直接本地双击打开（最简单推荐）
1. 在 Windows 资源管理器或 Linux 文件管理器中打开仓库的 `docs/web/` 文件夹；
2. 双击打开 [index.html](index.html) 即可在默认浏览器（Chrome、Edge、Firefox、Safari 等）中直接加载查看；
3. 点击左侧侧边栏或页面底部的翻页卡片，即可在各个章节页面之间**秒级跳转**；
4. 文档采用纯原生静态网页（HTML5 + CSS3 + 纯原生 Vanilla JS），**零外部网络 CDN 依赖**，离线状态下所有交互、图表与样式均 100% 正常工作。

### 方法 2：使用简易本地 HTTP 服务器
如果您在局域网内或希望通过 Web 服务浏览：
```bash
# 在 docs/web 目录下启动 Python 内置 HTTP 服务器
cd docs/web
python -m http.server 8080
```
然后在浏览器中访问：`http://127.0.0.1:8080` 即可。

---

## 目录结构

```text
docs/
├── architecture/     Markdown 架构与部署文档
├── experiments/      实验记录
├── hardware/         硬件清单
├── interfaces/       话题与接口说明
├── protocols/        底层协议
├── usage/            使用与维护手册
└── web/              本网页版说明文档
    ├── index.html               1. 项目概览与硬件平台
    ├── signal-flow.html         2. 系统信号流图与项目大纲
    ├── architecture.html        3. 系统分层架构与 13 包
    ├── nodes.html               4. 核心节点（Nodes）详解
    ├── interfaces.html          5. 话题、服务与消息总线
    ├── algorithms.html          6. 算法原理与底层协议
    ├── tutorials.html           7. 初学者分步实操教程
    ├── troubleshooting.html     8. 实机排障与推力模拟器
    ├── viewer.html              仓库 Markdown 在线阅读器
    ├── md-viewer.js             Markdown 解析与渲染（零依赖）
    ├── md-data.js               全部 Markdown 内容快照（构建产物）
    ├── style.css                全站主题、图表与阅读视图样式
    ├── app.js                   主题切换 / 导航高亮 / 搜索 / 推力模拟器
    └── README.md                本说明文件
```

---

## 维护约定

- 新增页面时，请同步更新**全部页面**的左侧导航（知识体系导航）与底部翻页器，并在此表补充条目；
- 页码按侧边栏顺序连续编号，避免插入页面后编号与导航不一致；
- “快捷外部资源”统一指向 `viewer.html?doc=README.md` 与 `viewer.html?doc=CHANGELOG.md`；
- 修改任意 Markdown 后运行 `node tools/generate-web-md-data.mjs` 刷新 `md-data.js`；
- 保持零外部依赖，确保离线双击打开时样式、脚本、图表与 Markdown 渲染均正常工作。
