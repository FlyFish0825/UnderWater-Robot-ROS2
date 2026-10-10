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
struct Fd {
  int value=-1;
  explicit Fd(int v=-1):value(v) {}
  ~Fd(){if(value>=0) ::close(value);}
  Fd(const Fd &)=delete;
  Fd &operator=(const Fd &)=delete;
};
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
BackendLease::BackendLease(const std::shared_ptr<rclcpp::Node> &node) {
  const auto ns=backend_namespace(node);
  const bool need_tx=node->has_parameter("preview")&&!node->get_parameter("preview").as_bool();
  const char *domain=std::getenv("ROS_DOMAIN_ID");
  const std::string key=std::string(domain?domain:"0")+":"+ns;
  const std::string dir="/tmp/underwater-entry-"+std::to_string(getuid())+"-"+std::to_string(std::hash<std::string>{}(key));
  if(mkdir(dir.c_str(),0700)!=0&&errno!=EEXIST) throw std::runtime_error("Cannot create runtime directory");
  struct stat st{};
  if(lstat(dir.c_str(),&st)!=0||!S_ISDIR(st.st_mode)||st.st_uid!=getuid()||(st.st_mode&0077))
    throw std::runtime_error("Runtime directory is not private");
  Fd clients(open_private(dir+"/clients.lock"));
  if(flock(clients.value,LOCK_SH)!=0) throw std::runtime_error("Cannot acquire runtime lease");
  if(!node->get_parameter("auto_start").as_bool()) { clients_=clients.value; clients.value=-1; return; }
  bool tx_ready=false;
  auto status=node->create_subscription<underwater_interfaces::msg::GatewayStatus>(ns+"/gateway/status",10,
    [&](underwater_interfaces::msg::GatewayStatus::ConstSharedPtr m){tx_ready=m->tx_enabled;});
  const auto ready=[&]{return node->count_publishers(ns+"/gateway/status")>0;};
  // Nano 上新进程的 DDS 发现可能超过 1.5 秒；先给已有 launch 足够的发现时间。
  for(int i=0;i<50&&running()&&!ready();++i) {rclcpp::spin_some(node); std::this_thread::sleep_for(100ms);}
  if(ready()) {
    Fd owner(open_private(dir+"/owner.lock"));
    const bool owned=flock(owner.value,LOCK_EX|LOCK_NB)!=0;
    if(owned&&need_tx) {
      Fd request(open_private(dir+"/request_tx"));
      if(pwrite(request.value,"1",1,0)!=1)throw std::runtime_error("Cannot request control backend");
      for(int i=0;i<150&&running()&&!tx_ready;++i){rclcpp::spin_some(node);std::this_thread::sleep_for(100ms);}
      if(running()&&!tx_ready)throw std::runtime_error("Control backend transition timed out; see "+dir+"/backend.log");
    }
    RCLCPP_INFO(node->get_logger(),"Reusing existing robot backend at %s",ns.c_str());
    clients_=clients.value; clients.value=-1; return;
  }
  Fd start(open_private(dir+"/start.lock"));
  if(flock(start.value,LOCK_EX)!=0) throw std::runtime_error("Cannot serialize backend startup");
  if(ready()&&need_tx) {
    Fd owner(open_private(dir+"/owner.lock"));
    if(flock(owner.value,LOCK_EX|LOCK_NB)!=0) {
      Fd request(open_private(dir+"/request_tx"));
      if(pwrite(request.value,"1",1,0)!=1)throw std::runtime_error("Cannot request control backend");
      for(int i=0;i<150&&running()&&!tx_ready;++i){rclcpp::spin_some(node);std::this_thread::sleep_for(100ms);}
      if(running()&&!tx_ready)throw std::runtime_error("Control backend transition timed out");
    }
  }
  if(!ready()&&running()) {
    Fd owner(open_private(dir+"/owner.lock"));
    if(flock(owner.value,LOCK_EX|LOCK_NB)==0) {
      // The helper acquires this lock itself. start.lock serializes entry processes.
      flock(owner.value,LOCK_UN);
      Fd request(open_private(dir+"/request_tx"));
      if(pwrite(request.value,need_tx?"1":"0",1,0)!=1)throw std::runtime_error("Cannot initialize backend mode");
      const std::string host=ament_index_cpp::get_package_prefix("sensors_cpp")+"/lib/sensors_cpp/backend_host";
      const auto port=node->get_parameter("serial_port").as_string();
      Fd logfile(open_private(dir+"/backend.log"));
      lseek(logfile.value,0,SEEK_END);
      const pid_t pid=fork();
      if(pid<0) throw std::runtime_error("Cannot fork backend helper");
      if(pid==0) {
        setsid();
        dup2(logfile.value,STDOUT_FILENO); dup2(logfile.value,STDERR_FILENO);
        const int nullfd=::open("/dev/null",O_RDONLY);
        if(nullfd>=0){dup2(nullfd,STDIN_FILENO);close(nullfd);}
        execl(host.c_str(),host.c_str(),dir.c_str(),ns.c_str(),port.c_str(),static_cast<char *>(nullptr));
        _exit(127);
      }
      RCLCPP_INFO(node->get_logger(),"Starting shared backend (%s); allocator remains disarmed. Log: %s/backend.log",need_tx?"control requested":"monitor only",dir.c_str());
    } else if(need_tx) {
      Fd request(open_private(dir+"/request_tx"));
      if(pwrite(request.value,"1",1,0)!=1)throw std::runtime_error("Cannot request control backend");
    }
    for(int i=0;i<150&&running()&&(!ready()||(need_tx&&!tx_ready));++i) { rclcpp::spin_some(node); std::this_thread::sleep_for(100ms); }
    if(running()&&(!ready()||(need_tx&&!tx_ready))) throw std::runtime_error("Backend did not become ready; see "+dir+"/backend.log");
  }
  clients_=clients.value; clients.value=-1;
}
BackendLease::~BackendLease(){if(clients_>=0)close(clients_);}
static volatile sig_atomic_t host_stop=0;
static void stop_host(int){host_stop=1;}
int backend_host(const std::string &dir,const std::string &ns,const std::string &port) {
  Fd owner(open_private(dir+"/owner.lock"));
  if(flock(owner.value,LOCK_EX|LOCK_NB)!=0) return 0;
  Fd clients(open_private(dir+"/clients.lock"));
  Fd request(open_private(dir+"/request_tx"));
  signal(SIGINT,stop_host); signal(SIGTERM,stop_host);
  const std::string argns="namespace:="+ns.substr(1),argport="serial_port:="+port;
  auto requested=[&]{char value='0';return pread(request.value,&value,1,0)==1&&value=='1';};
  bool tx=requested();
  auto launch=[&]() {
  pid_t child=fork();
  if(child<0) throw std::runtime_error("Cannot start ROS launch");
  if(child==0) {
    setpgid(0,0);
    execlp("ros2","ros2","launch","underwater_bringup","manual.launch.xml",
      tx?"motor_tx:=true":"motor_tx:=false",argns.c_str(),argport.c_str(),static_cast<char *>(nullptr));
    _exit(127);
  }
  setpgid(child,child);
  return child;
  };
  auto stop_child=[&](pid_t child) {
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
    if(done==child) {kill(-child,SIGINT);return WIFEXITED(status)?WEXITSTATUS(status):1;}
    if(done<0&&errno!=EINTR) return 1;
    if(flock(clients.value,LOCK_EX|LOCK_NB)==0) {flock(clients.value,LOCK_UN);++idle;} else idle=0;
    // Hold the exclusive client lock through shutdown so a new entry cannot
    // attach to a backend that is already being torn down.
    if(idle>=10) {
      if(flock(clients.value,LOCK_EX|LOCK_NB)==0)break;
      idle=0;
    }
    if(!tx&&requested()) {stop_child(child);tx=true;child=launch();}
    std::this_thread::sleep_for(200ms);
  }
  stop_child(child);
  flock(owner.value,LOCK_UN);
  flock(clients.value,LOCK_UN);
  return 0;
}
}
