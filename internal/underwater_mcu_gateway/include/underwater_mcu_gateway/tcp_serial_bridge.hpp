#pragma once
// 原始串口透传 TCP 服务端（单客户端、非阻塞）：字节原样转发，不解析、不包装、不重放。
// 只有控制权属于 GUI 且客户端完成 "OWNER <token>" 握手后才能写串口，否则仅接收。
// 串口未就绪、控制权变更或收发失败都会断开客户端，要求其重连并重新认证。
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
  // 控制权或 token 变化时会断开现有客户端，避免旧会话继续持有写权限。
  void set_controller(ControllerOwner owner,const std::string &token);
  // 串口未就绪（含尚未过 settling）时断开客户端。
  void set_serial_ready(bool ready);
  // 处理 accept/收发/握手，返回允许透传给串口的字节；每轮轮询都必须调用。
  wire::Bytes poll();
  // 把串口收到的字节排入发送缓冲；GUI 未认证或积压超限时不会外发。
  void forward_rx(const wire::Bytes &bytes);
  bool take_disconnect();
  bool take_write_disconnect();
  bool connected() const {return client_>=0;}
  // 写权限 = 已连接 + 控制权属于 GUI + 本会话已通过 token 握手。
  bool write_authorized() const {return client_>=0 && owner_==ControllerOwner::Gui && authenticated_;}
  std::uint16_t port() const {return port_;}
  std::uint64_t rx_bytes() const {return rx_bytes_;}
  std::uint64_t tx_bytes() const {return tx_bytes_;}
  std::uint64_t rejected_bytes() const {return rejected_bytes_;}
  std::uint64_t sessions() const {return sessions_;}
  const std::string &detail() const {return detail_;}
private:
  void drop(const std::string &reason,bool holder_disconnect=true);
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
