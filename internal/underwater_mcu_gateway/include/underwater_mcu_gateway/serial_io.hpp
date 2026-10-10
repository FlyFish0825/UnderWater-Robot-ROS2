#pragma once
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
  void open(const std::string &, bool allow_write=false);
  void close() noexcept;
  bool connected() const { return fd_>=0; }
  wire::Bytes read_available();
  void write_all(const wire::Bytes &, int timeout_ms=20);
private:
  int fd_=-1;
  bool writable_=false;
};
}
