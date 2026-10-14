// ==============================================================================
// src/sensors_cpp/src/backend.cpp
// ==============================================================================
// 共享后台的生命周期管理：
// - 检测、复用或启动 underwater_bringup；
// - 统计活跃入口客户端锁 (clients.lock)，最后一个入口退出后安全关闭后台；
// - 运行时目录由 ROS_DOMAIN_ID 与 robot_namespace 散列决定，防多域冲突。
// ==============================================================================
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

using namespace std::chrono_literals;

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

// 构造租约：定位运行时目录并以共享锁占用 clients.lock，然后按需复用或启动共享后台
BackendLease::BackendLease(const std::shared_ptr<rclcpp::Node> &node) {
  const auto ns = backend_namespace(node);
  const bool need_tx = node->has_parameter("preview") && !node->get_parameter("preview").as_bool();
  const char *domain = std::getenv("ROS_DOMAIN_ID");
  const std::string key = std::string(domain ? domain : "0") + ":" + ns;
  const std::string dir = "/tmp/underwater-entry-" + std::to_string(getuid()) + "-" +
                          std::to_string(std::hash<std::string>{}(key));

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
  auto ready = [&]() {
    bool found = false;
    auto sub = node->create_subscription<underwater_interfaces::msg::GatewayStatus>(
        ns + "/gateway/status",
        rclcpp::QoS(1),
        [&](underwater_interfaces::msg::GatewayStatus::ConstSharedPtr m) {
          if (m->connected && (!need_tx || m->tx_enabled)) {
            found = true;
          }
        });
    for (int i = 0; i < 5 && !found; ++i) {
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(20ms);
    }
    return found;
  };

  if (ready()) {
    clients_ = clients.value;
    clients.value = -1;
    return;
  }

  if (!node->get_parameter("auto_start").as_bool()) {
    throw std::runtime_error("Shared backend is not ready and auto_start is false");
  }

  if (need_tx) {
    if (pwrite(request.value, "1", 1, 0) != 1) {
      throw std::runtime_error("Cannot request control backend");
    }
  }

  // 尝试获取 owner 独占锁，判定是否由当前进程负责启动 backend_host
  if (flock(owner.value, LOCK_EX | LOCK_NB) == 0) {
    flock(owner.value, LOCK_UN);
    const auto prefix = ament_index_cpp::get_package_prefix("sensors_cpp");
    const std::string helper = prefix + "/lib/sensors_cpp/backend_host";
    const auto port = node->get_parameter("serial_port").as_string();

    const pid_t pid = fork();
    if (pid < 0) {
      throw std::runtime_error("Cannot fork backend host");
    }
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

BackendLease::~BackendLease() {
  if (clients_ >= 0) {
    flock(clients_, LOCK_UN);
    close(clients_);
  }
}

// 宿主主循环：监控 client 共享锁，在所有客户端退出后等待 2s 空闲并优雅关停后台
static volatile std::sig_atomic_t host_stop = 0;

static void on_host_stop(int) {
  host_stop = 1;
}

int backend_host(const std::string &dir, const std::string &ns, const std::string &port) {
  const Fd owner(open_private(dir + "/owner.lock"));
  if (flock(owner.value, LOCK_EX | LOCK_NB) != 0) {
    return 0;  // 另一个宿主已经在运行
  }

  const Fd clients(open_private(dir + "/clients.lock"));
  const Fd request(open_private(dir + "/control.request"));

  std::signal(SIGINT, on_host_stop);
  std::signal(SIGTERM, on_host_stop);

  auto requested = [&]() {
    char c = 0;
    return pread(request.value, &c, 1, 0) == 1 && c == '1';
  };

  bool tx = requested();

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

  while (!host_stop) {
    const auto done = waitpid(child, &status, WNOHANG);
    if (done == child) {
      kill(-child, SIGINT);
      return WIFEXITED(status) ? WEXITSTATUS(status) : 1;
    }
    if (done < 0 && errno != EINTR) {
      return 1;
    }

    if (flock(clients.value, LOCK_EX | LOCK_NB) == 0) {
      flock(clients.value, LOCK_UN);
      ++idle;
    } else {
      idle = 0;
    }

    if (idle >= 10) {
      if (flock(clients.value, LOCK_EX | LOCK_NB) == 0) {
        break;
      }
      idle = 0;
    }

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
