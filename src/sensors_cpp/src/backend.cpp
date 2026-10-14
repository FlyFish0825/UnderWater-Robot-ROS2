// ==============================================================================
// src/sensors_cpp/src/backend.cpp
// ==============================================================================
// 共享后台的生命周期管理：
// - 检测、复用或启动 underwater_bringup；
// - 统计活跃入口客户端锁 (clients.lock)，最后一个入口退出后安全关闭后台；
// - 运行时目录由 ROS_DOMAIN_ID 与 robot_namespace 散列决定，防多域冲突。
// ==============================================================================
// ------------------------------------------------------------------------------
// 【初学者导读】
// 所属：sensors_cpp 包，共享后台生命周期管理（backend.cpp）。
// 角色：多个兼容入口共用同一个硬件后台（underwater_bringup）。谁需要就在全局私有目录里
//       抢/占一把锁：第一个进入的负责 fork 出 backend_host，其余入口复用现有后台。
// 交互：
//   - 订阅 <ns>/gateway/status（underwater_interfaces/GatewayStatus）判断后台是否健康；
//   - 由 backend_host 内部执行 ros2 launch 启动整套节点；
//   - 借助文件锁（flock）与 /tmp 下私有目录跨进程协调。
// ROS 2 概念白话：
//   launch＝一次性启动多个节点的脚本；命名空间/参数通过命令行传给 launch。
//   话题名 <ns>/gateway/status 里的 <ns> 就是 robot_namespace，用来隔离多台机器人。
//   QoS(1)＝只保留最近 1 条消息的队列深度。
// C++ 概念白话：
//   struct Fd / RAII＝把文件描述符包成对象，析构自动 close，异常安全。
//   std::hash<std::string>＝把字符串映射成整数，这里用于生成稳定的目录名。
//   fork / execl / setsid＝创建子进程、替换成另一个程序、脱离终端会话。
//   flock＝文件锁，多个进程靠它协商“谁在用后台”。
//   std::this_thread::sleep_for＝让当前线程睡眠指定时长。
// ------------------------------------------------------------------------------
#include "sensors_cpp/entry.hpp"

#include <chrono>
#include <cerrno>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <thread>

#include <fcntl.h>
#include <signal.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

#include <ament_index_cpp/get_package_prefix.hpp>
#include <rclcpp/rclcpp.hpp>
#include <underwater_interfaces/msg/gateway_status.hpp>

namespace compatibility {

// 字面量后缀：让我们能写 20ms、100ms 这类直观的时间常量。
using namespace std::chrono_literals;

// C++ 小知识：RAII＝把资源（此处是文件描述符）绑在对象生命周期上，析构时自动 close。
// 禁止拷贝，避免两个对象各关一次同一个 fd。
// 极简 RAII 文件描述符：析构自动 close，禁止拷贝
struct Fd {
  int value = -1;
  explicit Fd(int v = -1) : value(v) {}
  ~Fd() {
    if (value >= 0) {
      ::close(value);
    }
  }
  Fd(const Fd &) = delete;
  Fd &operator=(const Fd &) = delete;
};

// open_private()：安全地打开锁文件。
// O_NOFOLLOW 防止有人用符号链接把我们引到别处；0600 保证只本用户可读写。
// 再校验属主与文件类型，避免锁文件被他人掉包。
// 以 0600 且 O_NOFOLLOW 打开锁文件，并校验属主为本用户、类型为常规文件，防止符号链接攻击
static int open_private(const std::string &path) {
  const int fd = ::open(path.c_str(), O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW, 0600);
  if (fd < 0) {
    throw std::runtime_error("Cannot open runtime lock: " + path);
  }
  struct stat st{};
  if (fstat(fd, &st) != 0 || st.st_uid != getuid() || !S_ISREG(st.st_mode)) {
    close(fd);
    throw std::runtime_error("Invalid runtime lock");
  }
  return fd;
}

// backend_namespace()：读参数 robot_namespace 并规范化校验（绝对路径、非根、去尾斜杠）。
std::string backend_namespace(const std::shared_ptr<rclcpp::Node> &node) {
  auto ns = node->get_parameter("robot_namespace").as_string();
  if (ns.empty() || ns.front() != '/') {
    throw std::invalid_argument("robot_namespace must be absolute");
  }
  while (ns.size() > 1 && ns.back() == '/') {
    ns.pop_back();
  }
  if (ns == "/") {
    throw std::invalid_argument("robot_namespace must name a robot");
  }
  return ns;
}

// 构造函数：先算出私有运行时目录（按 UID + ROS 域 + 命名空间散列，避免多实例冲突），
// 再打开 owner/clients/request 三个锁文件，并以共享锁加入客户端集合。
// 构造租约：定位运行时目录并以共享锁占用 clients.lock，然后按需复用或启动共享后台
BackendLease::BackendLease(const std::shared_ptr<rclcpp::Node> &node) {
  const auto ns = backend_namespace(node);
  const bool need_tx = node->has_parameter("preview") && !node->get_parameter("preview").as_bool();
  const char *domain = std::getenv("ROS_DOMAIN_ID");
  const std::string key = std::string(domain ? domain : "0") + ":" + ns;
  const std::string dir = "/tmp/underwater-entry-" + std::to_string(getuid()) + "-" +
                          std::to_string(std::hash<std::string>{}(key));

// 创建 0700 私有目录；若已存在（EEXIST）也算成功。
  if (mkdir(dir.c_str(), 0700) != 0 && errno != EEXIST) {
    throw std::runtime_error("Cannot create runtime directory");
  }

  // 打开各同步锁文件
  const Fd owner(open_private(dir + "/owner.lock"));
  Fd clients(open_private(dir + "/clients.lock"));
  const Fd request(open_private(dir + "/control.request"));

  // 以共享锁加入客户端集合
  if (flock(clients.value, LOCK_SH) != 0) {
    throw std::runtime_error("Cannot register entry client");
  }

  // 校验当前已有后台的健康状态
// 局部 lambda：探测现有后台是否健康——订阅 gateway/status，看是否 connected（需要时还要求 tx_enabled）。
// C++ 小知识：lambda＝匿名函数，[&] 表示按引用捕获外部变量，常作为回调传入。
  auto ready = [&]() {
    bool found = false;
// create_subscription<消息类型>(话题, QoS, 回调)：每来一条消息执行一次回调。
    auto sub = node->create_subscription<underwater_interfaces::msg::GatewayStatus>(
        ns + "/gateway/status",
        rclcpp::QoS(1),
        [&](underwater_interfaces::msg::GatewayStatus::ConstSharedPtr m) {
          if (m->connected && (!need_tx || m->tx_enabled)) {
            found = true;
          }
        });
// 最多 spin 5 轮、每轮 20ms（共约 100ms），给后台一点时间上报状态。
    for (int i = 0; i < 5 && !found; ++i) {
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(20ms);
    }
    return found;
  };

// 后台已就绪：把共享锁 fd 交给成员保存，并把局部 value 置 -1，防止 Fd 析构时误关它。
  if (ready()) {
    clients_ = clients.value;
    clients.value = -1;
    return;
  }

// auto_start=false 时不允许自动拉起后台，直接报错让上层退出。
  if (!node->get_parameter("auto_start").as_bool()) {
    throw std::runtime_error("Shared backend is not ready and auto_start is false");
  }

// 需要电机发送时，往 control.request 写入 '1'，通知后续宿主以 tx 模式启动。
  if (need_tx) {
    if (pwrite(request.value, "1", 1, 0) != 1) {
      throw std::runtime_error("Cannot request control backend");
    }
  }

// 用非阻塞独占锁选出唯一负责人：谁抢到 owner.lock 谁负责 fork 后台宿主，避免重复启动。
  // 尝试获取 owner 独占锁，判定是否由当前进程负责启动 backend_host
  if (flock(owner.value, LOCK_EX | LOCK_NB) == 0) {
    flock(owner.value, LOCK_UN);
    const auto prefix = ament_index_cpp::get_package_prefix("sensors_cpp");
    const std::string helper = prefix + "/lib/sensors_cpp/backend_host";
    const auto port = node->get_parameter("serial_port").as_string();

// fork()＝复制当前进程。父进程继续往下；子进程稍后 exec 成 backend_host。
    const pid_t pid = fork();
    if (pid < 0) {
      throw std::runtime_error("Cannot fork backend host");
    }
// 子进程：脱离终端会话（setsid），把标准输入/输出/错误重定向到 /dev/null，再 exec helper。
    if (pid == 0) {
      // 子进程分离并拉起 helper
      setsid();
      const int devnull = open("/dev/null", O_RDWR);
      if (devnull >= 0) {
        dup2(devnull, STDIN_FILENO);
        dup2(devnull, STDOUT_FILENO);
        dup2(devnull, STDERR_FILENO);
        close(devnull);
      }
      execl(helper.c_str(), helper.c_str(), dir.c_str(), ns.c_str(), port.c_str(), nullptr);
      _exit(127);
    }
  }

// 父进程在此等待后台就绪：最多 50 轮 × 100ms ≈ 5 秒。
  // 等待后台就绪 (最多等待 5 秒)
  for (int i = 0; i < 50 && running() && !ready(); ++i) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(100ms);
  }

  if (!ready()) {
    throw std::runtime_error("Shared backend did not become ready within 5s");
  }

  clients_ = clients.value;
  clients.value = -1;
}

// 析构：释放共享锁并关闭 fd。所有入口都退出后，宿主检测到无人占用便会关停后台。
BackendLease::~BackendLease() {
  if (clients_ >= 0) {
    flock(clients_, LOCK_UN);
    close(clients_);
  }
}

// 宿主主循环：靠“能否独占 clients.lock”判断还有没有客户端，空闲一段时间后优雅关停。
// 宿主主循环：监控 client 共享锁，在所有客户端退出后等待 2s 空闲并优雅关停后台
// 宿主的停止标志，由下面的信号处理函数在收到 SIGINT/SIGTERM 时置 1。
static volatile std::sig_atomic_t host_stop = 0;

static void on_host_stop(int) {
  host_stop = 1;
}

// backend_host()：宿主主函数。先确保自己是唯一宿主，再启动并监管 ros2 launch 子进程。
int backend_host(const std::string &dir, const std::string &ns, const std::string &port) {
  const Fd owner(open_private(dir + "/owner.lock"));
  if (flock(owner.value, LOCK_EX | LOCK_NB) != 0) {
    return 0;  // 另一个宿主已经在运行
  }

  const Fd clients(open_private(dir + "/clients.lock"));
  const Fd request(open_private(dir + "/control.request"));

  std::signal(SIGINT, on_host_stop);
  std::signal(SIGTERM, on_host_stop);

// requested()：读 control.request 首字节是否为 '1'，判断是否有人请求 tx 模式。
  auto requested = [&]() {
    char c = 0;
    return pread(request.value, &c, 1, 0) == 1 && c == '1';
  };

  bool tx = requested();

// launch()：fork 出子进程并 exec ros2 launch，把命名空间/串口/是否发送电机指令作为 launch 参数。
// setpgid 让子进程单独成组，方便之后用 kill(-pid) 一次结束整棵进程树。
  auto launch = [&]() -> pid_t {
    const auto bringup = ament_index_cpp::get_package_prefix("underwater_bringup");
    const std::string xml = bringup + "/share/underwater_bringup/launch/bringup.launch.xml";
    const pid_t child = fork();
    if (child < 0) {
      throw std::runtime_error("Cannot fork bringup");
    }
    if (child == 0) {
      const std::string a_ns = "robot_namespace:=" + ns;
      const std::string a_port = "serial_port:=" + port;
      const std::string a_tx = std::string("enable_motor_tx:=") + (tx ? "true" : "false");
      execlp("ros2", "ros2", "launch", xml.c_str(), a_ns.c_str(), a_port.c_str(), a_tx.c_str(), nullptr);
      _exit(127);
    }
    setpgid(child, child);  // 单独进程组
    return child;
  };

// stop_child()：优雅关停子进程——先 SIGINT 等 5 秒，再 SIGTERM 等 2 秒，最后 SIGKILL 兜底。
  auto stop_child = [&](pid_t child) {
    int status = 0;
    kill(-child, SIGINT);
    for (int i = 0; i < 50; ++i) {
      if (waitpid(child, &status, WNOHANG) == child) {
        return;
      }
      std::this_thread::sleep_for(100ms);
    }
    kill(-child, SIGTERM);
    for (int i = 0; i < 20; ++i) {
      if (waitpid(child, &status, WNOHANG) == child) {
        return;
      }
      std::this_thread::sleep_for(100ms);
    }
    kill(-child, SIGKILL);
    waitpid(child, &status, 0);
  };

  pid_t child = launch();
  unsigned idle = 0;
  int status = 0;

// 主监控循环：每 200ms 检查子进程是否退出、是否所有客户端都已离开、以及是否有人请求切换 tx。
  while (!host_stop) {
    const auto done = waitpid(child, &status, WNOHANG);
    if (done == child) {
      kill(-child, SIGINT);
      return WIFEXITED(status) ? WEXITSTATUS(status) : 1;
    }
    if (done < 0 && errno != EINTR) {
      return 1;
    }

// 用非阻塞独占锁探测：能拿到说明已无客户端持有共享锁，空闲计数 +1。
    if (flock(clients.value, LOCK_EX | LOCK_NB) == 0) {
      flock(clients.value, LOCK_UN);
      ++idle;
    } else {
      idle = 0;
    }

// 连续 10 次（约 2 秒）空闲才真正关停，避免入口短暂重启导致后台频繁启停。
    if (idle >= 10) {
      if (flock(clients.value, LOCK_EX | LOCK_NB) == 0) {
        break;
      }
      idle = 0;
    }

// 运行期有人请求 tx：重启一次 bringup，使能电机发送。
    if (!tx && requested()) {
      stop_child(child);
      tx = true;
      child = launch();
    }
    std::this_thread::sleep_for(200ms);
  }

  stop_child(child);
  flock(owner.value, LOCK_UN);
  flock(clients.value, LOCK_UN);
  return 0;
}

}  // namespace compatibility
