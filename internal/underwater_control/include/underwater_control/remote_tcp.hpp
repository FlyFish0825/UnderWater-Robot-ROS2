#pragma once
#include "underwater_control/remote_manual.hpp"
#include <functional>
namespace underwater::control::remote {
// Linux/POSIX adapter. Nonblocking bounded work; never owns the MCU serial FD.
class TcpServer {
public:
  enum class Event { Connected, Line, Disconnected };
  using Handler=std::function<void(Event,const std::string &)>;
  TcpServer(const std::string &address,std::uint16_t port);
  ~TcpServer();
  TcpServer(const TcpServer &)=delete;
  TcpServer &operator=(const TcpServer &)=delete;
  void poll(Millis at,const Handler &handler);
  bool queue(const std::string &line,Millis at);
  void drop();
  void reserve(bool reserved) { reserved_=reserved; ever_reserved_|=reserved; }
  bool connected() const { return client_>=0; }
  std::uint64_t generation() const { return generation_; }
private:
  int listener_=-1,client_=-1;
  std::uint64_t generation_=0;
  bool reserved_=false;
  bool ever_reserved_=false;
  Millis accepted_at_=0;
  LineBuffer input_;
  OutputBuffer output_;
};
}
