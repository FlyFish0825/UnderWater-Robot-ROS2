#pragma once
#include "underwater_mcu_gateway/control_ownership.hpp"
#include "underwater_mcu_gateway/protocol.hpp"
#include <cstdint>
#include <string>

namespace underwater {
// A single nonblocking TCP session. Bytes are never parsed, wrapped or replayed.
// Serial ownership and command arbitration remain in the existing gateway.
class TcpSerialBridge {
public:
  ~TcpSerialBridge();
  TcpSerialBridge()=default;
  TcpSerialBridge(const TcpSerialBridge &)=delete;
  TcpSerialBridge &operator=(const TcpSerialBridge &)=delete;
  void open(const std::string &bind_address,std::uint16_t port);
  void close() noexcept;
  void set_controller(ControllerOwner owner,const std::string &token);
  void set_serial_ready(bool ready);
  wire::Bytes poll();
  void forward_rx(const wire::Bytes &bytes);
  bool take_disconnect();
  bool take_write_disconnect();
  bool connected() const {return client_>=0;}
  bool write_authorized() const {return client_>=0 && owner_==ControllerOwner::Gui && authenticated_;}
  std::uint16_t port() const {return port_;}
  std::uint64_t rx_bytes() const {return rx_bytes_;}
  std::uint64_t tx_bytes() const {return tx_bytes_;}
  std::uint64_t rejected_bytes() const {return rejected_bytes_;}
  std::uint64_t sessions() const {return sessions_;}
  const std::string &detail() const {return detail_;}
private:
  void drop(const std::string &reason);
  void flush();
  int listener_=-1,client_=-1;
  std::uint16_t port_=0;
  bool ready_=false,authenticated_=false,disconnected_=false,write_disconnected_=false;
  ControllerOwner owner_=ControllerOwner::Nano;
  std::string token_,handshake_;
  wire::Bytes outgoing_;
  std::size_t offset_=0;
  std::uint64_t rx_bytes_=0,tx_bytes_=0,rejected_bytes_=0,sessions_=0;
  std::string detail_="disabled";
};
}
