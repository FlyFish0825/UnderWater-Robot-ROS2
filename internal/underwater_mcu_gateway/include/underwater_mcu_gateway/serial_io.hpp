#pragma once
// POSIX 串口封装：以 115200 8N1 raw 模式打开 MCU UART，读写均为非阻塞。
// 打开时用 TIOCEXCL 独占端口，close() 会恢复打开前的 termios 配置。
// 默认只读打开，只有 open(path, allow_write=true) 后才允许 write_all()。
#include "underwater_mcu_gateway/protocol.hpp"
#include <string>
#include <termios.h>
namespace underwater {
class SerialIo {
public:
  SerialIo()=default;
  ~SerialIo() { close(); }
  SerialIo(const SerialIo &)=delete;
  SerialIo & operator=(const SerialIo &)=delete;
  // 打开失败或配置失败会抛异常，且不会留下已打开的 fd。
  void open(const std::string &, bool allow_write=false);
  // 可重复调用：恢复原串口配置、释放独占并置为未连接。
  void close() noexcept;
  bool connected() const { return fd_>=0; }
  // 读走当前可读字节（单次上限 64KiB）；无数据返回空，断链抛异常。
  wire::Bytes read_available();
  // 在 timeout_ms 内写完整个缓冲；未以可写方式打开时直接抛异常。
  void write_all(const wire::Bytes &, int timeout_ms=20);
private:
  int fd_=-1;
  termios saved_{};
  bool saved_valid_=false;
  bool writable_=false;
};
}
