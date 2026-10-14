// ==============================================================================
// src/sensors_cpp/include/sensors_cpp/entry.hpp
// ==============================================================================
// sensors_cpp 兼容入口包的公共接口声明：
// 所有旧入口命令 (depth / sensors_sub / sensors_sub_jilushuju / manual_control 等)
// 统一通过 main 调用 compatibility::run()，并复用共享后台租约与遥测/手动实现。
// ==============================================================================
// ------------------------------------------------------------------------------
// 【初学者导读】
// 所属：sensors_cpp 包（C++ 兼容入口库）。本文件是它的公共头文件，只做声明，不含实现。
// 角色：把很多“旧命令名”（depth、sensors_sub、manual_control 等）统一到一个 run() 入口，
//       具体实现分布在 entry.cpp / backend.cpp / manual.cpp / telemetry.cpp。
// 交互：发布 <ns>/control/manual_request、<ns>/control/wrench_request、/forces；
//       订阅 <ns>/gateway/status、<ns>/sensors/imu/state、<ns>/sensors/depth/state 等；
//       调用服务 <ns>/control/enable（std_srvs/SetBool）申请或释放电机控制权。
// ROS 2 概念白话：
//   Node（节点）＝一个能收发话题、调用服务的独立小程序；这里用 shared_ptr<Node> 传递。
//   Publisher / Subscription（发布者/订阅者）＝向话题发消息 / 从话题收消息。
//   Service（服务）＝请求-应答式调用，像函数调用但跨进程。
//   Parameter（参数）＝节点配置项，命令行用 -p 名:=值 设置。
//   QoS（服务质量）＝话题的传输策略，如队列深度、是否保留最后一条。
//   Executor / spin＝驱动节点处理回调的循环，spin_some 只处理当前积压的一批就返回。
//   命名空间（namespace）＝话题名前缀，用来隔离多台机器人，这里是 robot_namespace。
// C++ 概念白话：
//   std::shared_ptr<T>＝可共享所有权的智能指针，引用计数归零时自动释放对象。
//   std::string / std::set＝标准字符串 / 有序集合。
//   RAII＝对象构造时获取资源、析构时自动释放；BackendLease 正是靠它自动交还后台。
//   提前声明 namespace rclcpp { class Node; }＝只告诉编译器有这个名字，避免包含整个大文件。
// ------------------------------------------------------------------------------
#pragma once

#include <memory>
#include <string>

// C++ 小知识：命名空间（namespace）＝给名字加前缀的“文件夹”，避免不同库重名冲突。
// 这里只“提前声明” Node 类，让下面函数签名能引用它，而不必在此包含庞大的 rclcpp 头文件。
namespace rclcpp {
class Node;
}

// compatibility 命名空间：本包所有兼容入口的公共接口都放在这里，避免全局重名。
namespace compatibility {

// running()：给各业务主循环判断“是否该继续跑”。
// 返回 true 的条件：ROS 2 未被关闭（rclcpp::ok()）且没收到 Ctrl+C / SIGTERM。
// 谁在用：manual() 与 telemetry() 的 while 循环。
// 进程是否继续运行：rclcpp 正常且未收到 SIGINT/SIGTERM 信号
bool running();

// run()：所有兼容入口命令的总调度，各 main() 只负责调用它。
// 输入：entry 是命令名（由 CMake 按目标名注入），argc/argv 是命令行参数。
// 返回：0 正常；69 表示命令名保留但功能未实现；1 表示参数或后端异常。
// 入口总调度：
// - 处理 --help / --check；
// - 初始化 rclcpp 并声明通用参数；
// - 装配共享后台 BackendLease 后分派到对应业务逻辑；
// - 返回码：0 正常，69 表示该入口名保留但功能未实现，1 为参数/后端异常。
int run(const std::string &entry, int argc, char **argv);

// telemetry()：订阅传感器话题并打印 / 记录 CSV。
// 覆盖命令：depth, depth_publisher, imu_publisher, sensors_sub, sensors_sub_jilushuju。
// 传感器查看与 CSV 记录实现：覆盖 depth, depth_publisher, imu_publisher, sensors_sub, sensors_sub_jilushuju
int telemetry(const std::string &entry, const std::shared_ptr<rclcpp::Node> &node);

// manual()：键盘手动控制或预设物理力曲线激励。
// 覆盖命令：manual_control（档位）与 forces_pub（波形）。
// 手动控制与实验激励实现：覆盖 manual_control (键盘档位) 与 forces_pub (预设物理力曲线激励)
int manual(const std::string &entry, const std::shared_ptr<rclcpp::Node> &node);

// backend_namespace()：从 ROS 参数 robot_namespace 读出命名空间并校验。
// 要求以 '/' 开头且不能就是 "/"；返回去掉末尾多余 '/' 后的规范形式。
// 读取并校验 robot_namespace 参数 (必须绝对路径且不为 "/")，返回规整后的命名空间
std::string backend_namespace(const std::shared_ptr<rclcpp::Node> &node);

// BackendLease：共享后台的 RAII“租约”。
// 构造时确保共享后台（underwater_bringup）在运行；析构时自动释放自己持有的共享锁。
// 好处：即使中途抛异常，锁也会在栈展开时释放，不会把后台永久卡住。
// 共享后台 RAII 租约：
// 构造时检测/复用/启动共享后端，析构时释放本入口持有的客户端锁。
// 通过 clients.lock 的共享锁计数活跃入口，最后一个入口退出后由宿主自动关闭后台。
class BackendLease {
public:
// 构造函数：定位运行时目录 -> 以共享锁加入客户端集合 -> 按需复用或拉起共享后台。
  explicit BackendLease(const std::shared_ptr<rclcpp::Node> &node);
// 析构函数：若仍持有共享锁 fd，就解锁并关闭，从而实现“最后一个入口退出后后台自动关闭”。
  ~BackendLease();

  BackendLease(const BackendLease &) = delete;
  BackendLease &operator=(const BackendLease &) = delete;

private:
  int clients_ = -1;  // 保存持有的共享锁 fd；-1 表示未持有 (auto_start=false 或已移交)
};

// backend_host()：后台宿主的真正主函数，负责启动/监控/关闭 ros2 launch 子进程。
// 一般由 BackendLease 自动 fork 拉起，用户无需手工运行。
// 后台宿主主函数入口：负责管理 ROS 2 bringup 子进程生命周期
int backend_host(const std::string &directory, const std::string &ns, const std::string &port);

}  // namespace compatibility
