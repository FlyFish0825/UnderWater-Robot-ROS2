// 伪终端验证关闭/重开时没有终端回显或软件流控，不连接真实硬件。
// ------------------------------------------------------------------------------
// 【初学者导读】本文件测试 SerialIo 串口封装的 Linux 行规(termios)设置，属于
// internal/underwater_mcu_gateway/test。它用“伪终端(PTY)”模拟串口设备：
// 先人为打开回显与软件流控，再让 SerialIo::open() 打开，验证它确实关闭了
// ECHO/ICANON 和 IXON/IXOFF/IXANY（否则二进制数据会被改写或阻塞）；
// 还验证 close 后终端属性被还原，以及只读模式下写串口会被拒绝。
// 涉及概念：POSIX open/tcgetattr/tcsetattr、poll、文件描述符、异常断言。
// ------------------------------------------------------------------------------
#include "underwater_mcu_gateway/serial_io.hpp"
#include <cstdlib>
#include <fcntl.h>
#include <iostream>
#include <poll.h>
#include <stdexcept>
#include <unistd.h>

// require()：本测试的断言，条件为假抛异常，main 统一捕获。
static void require(bool ok,const char * message) {
  if(!ok) throw std::runtime_error(message);
}
// 用 posix_openpt 创建主/从伪终端：主端 master 模拟 MCU，从端 path 交给 SerialIo。
int main() {
  int master=-1,keeper=-1;
  try {
    master=posix_openpt(O_RDWR|O_NOCTTY|O_NONBLOCK);
    require(master>=0,"posix_openpt");
    require(grantpt(master)==0 && unlockpt(master)==0,"prepare PTY");
    const std::string path=ptsname(master);
    keeper=::open(path.c_str(),O_RDWR|O_NOCTTY|O_NONBLOCK);
    require(keeper>=0,"open PTY keeper");
    termios initial{};
    require(tcgetattr(keeper,&initial)==0,"get initial termios");
    initial.c_lflag|=ECHO|ICANON;
    initial.c_iflag|=IXON|IXOFF|IXANY;
    require(tcsetattr(keeper,TCSANOW,&initial)==0,"set terminal defaults");
    underwater::SerialIo serial;
    for(int attempt=0;attempt<3;++attempt) {
      serial.open(path);
      termios current{};
      require(tcgetattr(keeper,&current)==0,"get raw termios");
      require(!(current.c_iflag&(IXON|IXOFF|IXANY)),"software flow control still enabled");
      bool rejected=false;
      try { serial.write_all({0x01}); } catch(const std::runtime_error &) { rejected=true; }
      require(rejected,"default open permits TX");
      const unsigned char sample[]={0xAA,0x55,0x13,0x11,0x00};
      require(::write(master,sample,sizeof(sample))==sizeof(sample),"inject binary input");
      underwater::wire::Bytes rx;
      for(int i=0;i<20 && rx.size()<sizeof(sample);++i) {
        auto bytes=serial.read_available(); rx.insert(rx.end(),bytes.begin(),bytes.end());
        usleep(1000);
      }
      require(rx==underwater::wire::Bytes(sample,sample+sizeof(sample)),"binary RX changed or blocked");
      serial.close(); serial.close();
      require(tcgetattr(keeper,&current)==0,"get closed termios");
      require(!(current.c_lflag&(ECHO|ICANON)),"close restored terminal echo/canonical mode");
      require(::write(master,"x",1)==1,"inject after close");
      pollfd p{master,POLLIN,0};
      require(poll(&p,1,30)==0,"MCU input echoed back after close");
      require(tcflush(keeper,TCIFLUSH)==0,"clear fixture input");
    }
    ::close(keeper); ::close(master);
    std::cout<<"PASS serial raw/no-echo close, reopen, binary RX and read-only TX gate\n";
    return 0;
  } catch(const std::exception & e) {
    if(keeper>=0) ::close(keeper);
    if(master>=0) ::close(master);
    std::cerr<<"FAIL "<<e.what()<<'\n'; return 1;
  }
}
