// sensors_cpp 兼容入口包的公共接口声明。
// 所有入口（ros2 run sensors_cpp depth / sensors_sub / sensors_sub_jilushuju / manual_control 等）
// 均由同一个 main 调用 compatibility::run()，并复用此处的后台租约与遥测/手动实现。
#pragma once
#include <memory>
#include <string>
namespace rclcpp { class Node; }
namespace compatibility {
// 进程是否继续运行：rclcpp 正常且未收到 SIGINT/SIGTERM（信号只置标志位）。
bool running();
// 入口总调度：处理 --help/--check，初始化 rclcpp，声明参数，装配共享后台后分派。
// 返回进程退出码：0 正常，69 表示该入口名保留但功能未实现，1 为参数/后端异常。
int run(const std::string &entry, int argc, char **argv);
// 传感器查看/记录实现，覆盖 depth、depth_publisher、imu_publisher、sensors_sub、sensors_sub_jilushuju。
int telemetry(const std::string &entry, const std::shared_ptr<rclcpp::Node> &node);
// 手动控制/实验激励实现，覆盖 manual_control（键盘档位）与 forces_pub（物理力激励）。
int manual(const std::string &entry, const std::shared_ptr<rclcpp::Node> &node);
// 读取并校验 robot_namespace 参数（必须绝对路径且不为 "/"），返回规整后的命名空间。
std::string backend_namespace(const std::shared_ptr<rclcpp::Node> &node);
// 共享后台 RAII 租约：构造时检测/复用/启动共享后端，析构时释放本入口持有的客户端锁。
// 通过 clients.lock 的共享锁计数活跃入口，最后一个入口退出后由宿主关闭后台。
class BackendLease {
public:
  explicit BackendLease(const std::shared_ptr<rclcpp::Node> &node);
  ~BackendLease();
  BackendLease(const BackendLease &) = delete;
  BackendLease &operator=(const BackendLease &) = delete;
private:
  int clients_ = -1;
};
// clients_ 保存持有的共享锁 fd；-1 表示未持有（auto_start=false 或已移交）。
int backend_host(const std::string &directory, const std::string &ns, const std::string &port);
}
