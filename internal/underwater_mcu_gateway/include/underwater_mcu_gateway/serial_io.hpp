// ==============================================================================
// internal/underwater_mcu_gateway/include/underwater_mcu_gateway/serial_io.hpp
// ==============================================================================
// POSIX 串口封装：以 115200 8N1 raw 模式打开 MCU UART，读写均为非阻塞。
// 打开时使用 TIOCEXCL 独占端口，close() 会恢复打开前的 termios 配置。
// 默认只读打开，只有 open(path, allow_write=true) 后才允许 write_all()。
// ==============================================================================
#pragma once

#include "underwater_mcu_gateway/protocol.hpp"

#include <string>
#include <termios.h>

namespace underwater {

class SerialIo {
public:
  SerialIo() = default;
  ~SerialIo() { close(); }

  SerialIo(const SerialIo &) = delete;
  SerialIo &operator=(const SerialIo &) = delete;

  // 打开串口并配置为原始模式；失败抛出 std::runtime_error，保证不泄漏 fd
  void open(const std::string &path, bool allow_write = false);

  // 恢复原串口 termios 配置、释放独占锁定并关闭 fd
  void close() noexcept;

  // 查询是否处于连接状态
  bool connected() const { return fd_ >= 0; }

  // 读走当前可读字节流 (单次上限 64KiB)；无数据返回空，断链抛出异常
  wire::Bytes read_available();

  // 在 timeout_ms 内完整写入字节缓冲；未以可写模式打开时直接抛异常
  void write_all(const wire::Bytes &bytes, int timeout_ms = 20);

private:
  int fd_ = -1;
  bool writable_ = false;
};

}  // namespace underwater
