// 共享后台的生命周期管理：检测/复用/启动 underwater_bringup，以及最后一个入口退出后关闭。
// 供全部 sensors_cpp 入口使用（BackendLease）；宿主逻辑位于 backend_host（同文件下半部分）。
// 约定：运行时目录由 ROS_DOMAIN_ID + robot_namespace 决定，位于 /tmp/underwater-entry-<uid>-<hash>。
#include "sensors_cpp/entry.hpp"
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <underwater_interfaces/msg/gateway_status.hpp>
#include <sys/file.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <chrono>
#include <cerrno>
#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <thread>
namespace compatibility {
using namespace std::chrono_literals;
// 极简 RAII 文件描述符：析构自动 close，禁止拷贝，用于各种锁文件。
struct Fd {
  int value=-1;
  explicit Fd(int v=-1):value(v) {}
  ~Fd(){if(value>=0) ::close(value);}
  Fd(const Fd &)=delete;
  Fd &operator=(const Fd &)=delete;
};
// 以 0600 且 O_NOFOLLOW 打开锁文件，并校验属主为本用户、类型为常规文件，防止符号链接替换。
static int open_private(const std::string &path) {
  const int fd=::open(path.c_str(),O_CREAT|O_RDWR|O_CLOEXEC|O_NOFOLLOW,0600);
  if(fd<0) throw std::runtime_error("Cannot open runtime lock: "+path);
  struct stat st{};
  if(fstat(fd,&st)!=0 || st.st_uid!=getuid() || !S_ISREG(st.st_mode)) { close(fd); throw std::runtime_error("Invalid runtime lock"); }
  return fd;
}
std::string backend_namespace(const std::shared_ptr<rclcpp::Node> &node) {
  auto ns=node->get_parameter("robot_namespace").as_string();
  if(ns.empty()||ns.front()!='/') throw std::invalid_argument("robot_namespace must be absolute");
  while(ns.size()>1&&ns.back()=='/') ns.pop_back();
  if(ns=="/") throw std::invalid_argument("robot_namespace must name a robot");
  return ns;
}
// 构造租约：定位运行时目录并以共享锁占用 clients.lock，然后按需复用或启动共享后台。
BackendLease::BackendLease(const std::shared_ptr<rclcpp::Node> &node) {
  const auto ns=backend_namespace(node);
  const bool need_tx=node->has_parameter("preview")&&!node->get_parameter("preview").as_bool(); // preview 为 false 才需要控制后端；无 preview 参数的入口视为需要
  const char *domain=std::getenv("ROS_DOMAIN_ID");
  const std::string key=std::string(domain?domain:"0")+":"+ns; // 用 ROS_DOMAIN_ID 隔离不同 DDS 域的后台
  const std::string dir="/tmp/underwater-entry-"+std::to_string(getuid())+"-"+std::to_string(std::hash<std::string>{}(key)); // 每用户/每命名空间的私有运行时目录
  if(mkdir(dir.c_str(),0700)!=0&&errno!=EEXIST) throw std::runtime_error("Cannot create runtime directory");
  struct stat st{};
  if(lstat(dir.c_str(),&st)!=0||!S_ISDIR(st.st_mode)||st.st_uid!=getuid()||(st.st_mode&0077))
    throw std::runtime_error("Runtime directory is not private");
  Fd clients(open_private(dir+"/clients.lock"));
  if(flock(clients.value,LOCK_SH)!=0) throw std::runtime_error("Cannot acquire runtime lease"); // 共享锁即“本入口存活”标记
  if(!node->get_parameter("auto_start").as_bool()) { clients_=clients.value; clients.value=-1; return; } // auto_start=false 只持锁不准备后台
  bool tx_ready=false;
  auto status=node->create_subscription<underwater_interfaces::msg::GatewayStatus>(ns+"/gateway/status",10,
    [&](underwater_interfaces::msg::GatewayStatus::ConstSharedPtr m){tx_ready=m->tx_enabled;}); // 用 tx_enabled 判断控制通道是否已解锁
  const auto ready=[&]{return node->count_publishers(ns+"/gateway/status")>0;}; // 有 gateway/status 发布者即认定后台已在运行
  // Nano 上新进程的 DDS 发现可能超过 1.5 秒；先给已有 launch 足够的发现时间。
  for(int i=0;i<50&&running()&&!ready();++i) {rclcpp::spin_some(node); std::this_thread::sleep_for(100ms);}
  if(ready()) { // 复用已有后台
    Fd owner(open_private(dir+"/owner.lock"));
    const bool owned=flock(owner.value,LOCK_EX|LOCK_NB)!=0; // 取不到独占锁说明后台宿主持有 owner.lock
    if(owned&&need_tx) { // 后台已在跑但需要控制：写 request_tx 请求切换 motor_tx
      Fd request(open_private(dir+"/request_tx"));
      if(pwrite(request.value,"1",1,0)!=1)throw std::runtime_error("Cannot request control backend");
      for(int i=0;i<150&&running()&&!tx_ready;++i){rclcpp::spin_some(node);std::this_thread::sleep_for(100ms);} // 最多等 15s 完成切换
      if(running()&&!tx_ready)throw std::runtime_error("Control backend transition timed out; see "+dir+"/backend.log");
    }
    RCLCPP_INFO(node->get_logger(),"Reusing existing robot backend at %s",ns.c_str());
    clients_=clients.value; clients.value=-1; return;
  }
  Fd start(open_private(dir+"/start.lock"));
  if(flock(start.value,LOCK_EX)!=0) throw std::runtime_error("Cannot serialize backend startup"); // 串行化并发入口的启动动作
  if(ready()&&need_tx) {
    Fd owner(open_private(dir+"/owner.lock"));
    if(flock(owner.value,LOCK_EX|LOCK_NB)!=0) {
      Fd request(open_private(dir+"/request_tx"));
      if(pwrite(request.value,"1",1,0)!=1)throw std::runtime_error("Cannot request control backend");
      for(int i=0;i<150&&running()&&!tx_ready;++i){rclcpp::spin_some(node);std::this_thread::sleep_for(100ms);}
      if(running()&&!tx_ready)throw std::runtime_error("Control backend transition timed out");
    }
  }
  if(!ready()&&running()) { // 无人提供后台：由本入口负责拉起
    Fd owner(open_private(dir+"/owner.lock"));
    if(flock(owner.value,LOCK_EX|LOCK_NB)==0) { // 拿到独占锁说明由本进程启动宿主
      // The helper acquires this lock itself. start.lock serializes entry processes.
      flock(owner.value,LOCK_UN);
      Fd request(open_private(dir+"/request_tx"));
      if(pwrite(request.value,need_tx?"1":"0",1,0)!=1)throw std::runtime_error("Cannot initialize backend mode"); // 预写后台启动模式
      const std::string host=ament_index_cpp::get_package_prefix("sensors_cpp")+"/lib/sensors_cpp/backend_host";
      const auto port=node->get_parameter("serial_port").as_string();
      Fd logfile(open_private(dir+"/backend.log"));
      lseek(logfile.value,0,SEEK_END);
      const pid_t pid=fork(); // fork 出脱离终端的助手进程，入口退出后后台仍存活
      if(pid<0) throw std::runtime_error("Cannot fork backend helper");
      if(pid==0) {
        setsid(); // 新建会话，避免继承入口的控制终端
        dup2(logfile.value,STDOUT_FILENO); dup2(logfile.value,STDERR_FILENO);
        const int nullfd=::open("/dev/null",O_RDONLY);
        if(nullfd>=0){dup2(nullfd,STDIN_FILENO);close(nullfd);}
        execl(host.c_str(),host.c_str(),dir.c_str(),ns.c_str(),port.c_str(),static_cast<char *>(nullptr));
        _exit(127);
      }
      RCLCPP_INFO(node->get_logger(),"Starting shared backend (%s); allocator remains disarmed. Log: %s/backend.log",need_tx?"control requested":"monitor only",dir.c_str());
    } else if(need_tx) { // 已有其它入口在启动：只补写控制请求
      Fd request(open_private(dir+"/request_tx"));
      if(pwrite(request.value,"1",1,0)!=1)throw std::runtime_error("Cannot request control backend");
    }
    for(int i=0;i<150&&running()&&(!ready()||(need_tx&&!tx_ready));++i) { rclcpp::spin_some(node); std::this_thread::sleep_for(100ms); } // 最多等 15s 至后台就绪
    if(running()&&(!ready()||(need_tx&&!tx_ready))) throw std::runtime_error("Backend did not become ready; see "+dir+"/backend.log");
  }
  clients_=clients.value; clients.value=-1;
}
BackendLease::~BackendLease(){if(clients_>=0)close(clients_);} // 释放共享锁，宿主据此判定入口是否已全部退出
static volatile sig_atomic_t host_stop=0;
static void stop_host(int){host_stop=1;}
// 共享后台宿主：独占 owner.lock 表示本进程拥有后台，代跑 ros2 launch underwater_bringup manual.launch.xml，
// 并在最后一个客户端退出后关闭它启动的 launch 进程组。返回 launch 的退出码或 0/1。
int backend_host(const std::string &dir,const std::string &ns,const std::string &port) {
  Fd owner(open_private(dir+"/owner.lock"));
  if(flock(owner.value,LOCK_EX|LOCK_NB)!=0) return 0; // 已有宿主在运行：单例，静默退出
  Fd clients(open_private(dir+"/clients.lock"));
  Fd request(open_private(dir+"/request_tx"));
  signal(SIGINT,stop_host); signal(SIGTERM,stop_host);
  const std::string argns="namespace:="+ns.substr(1),argport="serial_port:="+port; // launch 参数：命名空间去掉前导 "/"
  auto requested=[&]{char value='0';return pread(request.value,&value,1,0)==1&&value=='1';}; // request_tx=="1" 表示需要控制
  bool tx=requested();
  auto launch=[&]() { // 启动 launch：motor_tx 由 request_tx 决定；false 只监控，不驱动电机
  pid_t child=fork();
  if(child<0) throw std::runtime_error("Cannot start ROS launch");
  if(child==0) {
    setpgid(0,0);
    execlp("ros2","ros2","launch","underwater_bringup","manual.launch.xml",
      tx?"motor_tx:=true":"motor_tx:=false",argns.c_str(),argport.c_str(),static_cast<char *>(nullptr));
    _exit(127);
  }
  setpgid(child,child); // 单独进程组，便于整组发送信号
  return child;
  };
  auto stop_child=[&](pid_t child) { // 优雅关停：SIGINT(5s) → SIGTERM(2s) → SIGKILL
    int status=0;
    kill(-child,SIGINT);
    for(int i=0;i<50;++i){if(waitpid(child,&status,WNOHANG)==child)return;std::this_thread::sleep_for(100ms);}
    kill(-child,SIGTERM);
    for(int i=0;i<20;++i){if(waitpid(child,&status,WNOHANG)==child)return;std::this_thread::sleep_for(100ms);}
    kill(-child,SIGKILL);waitpid(child,&status,0);
  };
  pid_t child=launch();
  unsigned idle=0; int status=0;
  while(!host_stop) {
    const auto done=waitpid(child,&status,WNOHANG);
    if(done==child) {kill(-child,SIGINT);return WIFEXITED(status)?WEXITSTATUS(status):1;} // launch 自行退出：回收其余进程组后返回其退出码
    if(done<0&&errno!=EINTR) return 1;
    if(flock(clients.value,LOCK_EX|LOCK_NB)==0) {flock(clients.value,LOCK_UN);++idle;} else idle=0; // 能取到独占锁说明当前无入口持有共享锁
    // Hold the exclusive client lock through shutdown so a new entry cannot
    // attach to a backend that is already being torn down.
    if(idle>=10) { // 连续 10×200ms=2s 无客户端后才尝试关停
      if(flock(clients.value,LOCK_EX|LOCK_NB)==0)break; // 仍取不到独占锁说明新入口刚接入，继续服务
      idle=0;
    }
    if(!tx&&requested()) {stop_child(child);tx=true;child=launch();} // 运行中收到控制请求：重启为 motor_tx:=true
    std::this_thread::sleep_for(200ms);
  }
  stop_child(child);
  flock(owner.value,LOCK_UN);
  flock(clients.value,LOCK_UN); // 先释放独占再释放共享，允许后续入口重新拉起后台
  return 0;
}
}
