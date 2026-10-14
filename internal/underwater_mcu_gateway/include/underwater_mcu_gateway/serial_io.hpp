// ==============================================================================
// internal/underwater_mcu_gateway/include/underwater_mcu_gateway/serial_io.hpp
// ==============================================================================
// POSIX 串口封装：以 115200 8N1 raw 模式打开 MCU UART，读写均为非阻塞。
// 打开时使用 TIOCEXCL 独占端口，close() 会恢复打开前的 termios 配置。
// 默认只读打开，只有 open(path, allow_write=true) 后才允许 write_all()。
// ==============================================================================
// 初学者导读：
// 1) 所属：internal/underwater_mcu_gateway 的传输层。它封装 POSIX 串口，
//    是网关与 STM32 之间唯一的物理字节通道。
// 2) 交互：节点每轮循环调用 read_available() 取字节交给 Parser，写方向用 write_all() 发下行帧。
// 3) ROS 2 概念：本文件不涉及 ROS API，只处理串口 fd。
// 4) C++ 概念：析构函数 ~SerialIo() 里 close() 属于 RAII（对象销毁自动释放资源）；
//    拷贝构造/赋值用 = delete 禁止，避免同一个 fd 被两个对象重复关闭；noexcept 表示不抛异常。
#pragma once

#include "underwater_mcu_gateway/protocol.hpp"

#include <string>
#include <termios.h>

namespace underwater {

// 串口句柄类：一个实例对应一个打开的串口；打开失败抛 std::runtime_error。
// 注意：拷贝被禁止，只能通过引用或指针传递。
class SerialIo {
public:
  SerialIo() = default;
  ~SerialIo() { close(); }

  SerialIo(const SerialIo &) = delete;
  SerialIo &operator=(const SerialIo &) = delete;

  // 打开串口并配置为原始模式；失败抛出 std::runtime_error，保证不泄漏 fd
// 打开并配置串口为 raw 模式（115200 8N1）；allow_write 决定是否允许 write_all()。
  void open(const std::string &path, bool allow_write = false);

  // 恢复原串口 termios 配置、释放独占锁定并关闭 fd
// 关闭串口并恢复打开的 termios 配置；析构时自动调用，可重复调用。
  void close() noexcept;

  // 查询是否处于连接状态
  bool connected() const { return fd_ >= 0; }

  // 读走当前可读字节流 (单次上限 64KiB)；无数据返回空，断链抛出异常
// 非阻塞读走当前可读字节（单次最多 64KiB）；无数据返回空，断链抛异常。
  wire::Bytes read_available();

  // 在 timeout_ms 内完整写入字节缓冲；未以可写模式打开时直接抛异常
// 在 timeout_ms 内尽力写完全部字节；未以可写模式打开会直接抛异常。
  void write_all(const wire::Bytes &bytes, int timeout_ms = 20);

private:
// fd_ < 0 表示未打开；writable_ 记录是否允许写（默认只读打开）。
  int fd_ = -1;
  bool writable_ = false;
};

}  // namespace underwater
