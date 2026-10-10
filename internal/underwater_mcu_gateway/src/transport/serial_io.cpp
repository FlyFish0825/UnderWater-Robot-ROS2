// 串口底层实现（termios）：打开、配置、关闭与限时读写。
// 打开失败的清理与 termios 恢复都在这里完成，调用方只需处理异常。
// 读端不阻塞（poll 超时为 0），写端按 timeout_ms 限时并支持部分写重试。
#include "underwater_mcu_gateway/serial_io.hpp"
#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <stdexcept>
#include <sys/ioctl.h>
#include <unistd.h>
namespace underwater {
void SerialIo::open(const std::string & path, bool allow_write) {
  close();
  // 非阻塞 + 非控制终端 + close-on-exec；默认只读，避免误向 MCU 发数据。
  fd_=::open(path.c_str(),(allow_write?O_RDWR:O_RDONLY)|O_NOCTTY|O_NONBLOCK|O_CLOEXEC);
  if(fd_<0) throw std::runtime_error(std::strerror(errno));
  writable_=allow_write;
  if(tcgetattr(fd_,&saved_)!=0) { close(); throw std::runtime_error("tcgetattr failed"); }
  saved_valid_=true;
  if(ioctl(fd_,TIOCEXCL)!=0) { close(); throw std::runtime_error("exclusive serial open failed"); } // 独占打开，拒绝其他进程同时读写
  auto cfg=saved_; // 以打开前的配置为基础，只改必要项
  cfmakeraw(&cfg); cfsetispeed(&cfg,B115200); cfsetospeed(&cfg,B115200); // raw 模式 + 115200 波特率
  cfg.c_cflag=(cfg.c_cflag & ~(CSIZE|PARENB|CSTOPB|CRTSCTS|HUPCL))|CS8|CREAD|CLOCAL;
  cfg.c_cc[VMIN]=0; cfg.c_cc[VTIME]=0; // 完全非阻塞：无数据时 read 立即返回
  if(tcsetattr(fd_,TCSANOW,&cfg)!=0) { close(); throw std::runtime_error("tcsetattr failed"); }
}
void SerialIo::close() noexcept {
  if(fd_>=0) {
    if(saved_valid_) tcsetattr(fd_,TCSANOW,&saved_);
    ioctl(fd_,TIOCNXCL); ::close(fd_);
  }
  fd_=-1; saved_valid_=false; writable_=false;
}
// 最多一次读 64KiB，避免单轮轮询长时间占用线程。
wire::Bytes SerialIo::read_available() {
  wire::Bytes bytes;
  if(fd_<0) return bytes;
  pollfd p{fd_,POLLIN,0};
  const int ready=poll(&p,1,0);
  if(ready<0 && errno!=EINTR) throw std::runtime_error("serial poll failed");
  if(ready<=0) return bytes;
  if(p.revents&(POLLHUP|POLLERR|POLLNVAL)) throw std::runtime_error("serial disconnected");
  std::uint8_t chunk[4096];
  while(bytes.size()<65536) {
    auto n=::read(fd_,chunk,sizeof(chunk));
    if(n>0) { bytes.insert(bytes.end(),chunk,chunk+n); continue; }
    if(n==0 || errno==EAGAIN || errno==EWOULDBLOCK) break;
    if(errno==EINTR) continue;
    throw std::runtime_error(std::strerror(errno));
  }
  return bytes;
}
void SerialIo::write_all(const wire::Bytes & bytes,int timeout_ms) {
  if(fd_<0 || !writable_) throw std::runtime_error("serial TX is disabled"); // 未授权写则立即失败
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(timeout_ms);
  std::size_t offset=0;
  while(offset<bytes.size()) {
    auto n=::write(fd_,bytes.data()+offset,bytes.size()-offset);
    if(n>0) { offset+=n; continue; }
    if(n<0 && errno!=EAGAIN && errno!=EWOULDBLOCK && errno!=EINTR)
      throw std::runtime_error(std::strerror(errno));
    // 部分写/EAGAIN：等到 POLLOUT 再续写，超时即抛错（调用方按串口故障处理）。
    auto left=std::chrono::duration_cast<std::chrono::milliseconds>(deadline-std::chrono::steady_clock::now()).count();
    if(left<=0) throw std::runtime_error("serial TX deadline exceeded");
    pollfd p{fd_,POLLOUT,0};
    if(::poll(&p,1,static_cast<int>(left))<0 && errno!=EINTR)
      throw std::runtime_error("serial TX poll failed");
  }
}
}
