#include "underwater_mcu_gateway/tcp_serial_bridge.hpp"
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <iostream>
#include <stdexcept>
using namespace underwater;
int checks=0;
void check(bool ok,const char *why){++checks;if(!ok)throw std::runtime_error(why);}
int connect_client(std::uint16_t port,const char *source=nullptr) {
  int fd=socket(AF_INET,SOCK_STREAM,0);check(fd>=0,"socket");
  if(source){sockaddr_in local{};local.sin_family=AF_INET;inet_pton(AF_INET,source,&local.sin_addr);
    check(bind(fd,reinterpret_cast<sockaddr *>(&local),sizeof(local))==0,"changed client source address");}
  sockaddr_in a{};a.sin_family=AF_INET;a.sin_port=htons(port);inet_pton(AF_INET,"127.0.0.1",&a.sin_addr);
  check(connect(fd,reinterpret_cast<sockaddr *>(&a),sizeof(a))==0,"connect");
  timeval timeout{1,0};setsockopt(fd,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout));return fd;
}
int main(){try {
  TcpSerialBridge server;server.open("127.0.0.1",0,true);
  int fd=connect_client(server.port());server.poll();char b[4096];check(recv(fd,b,sizeof(b),0)==0,"reject when serial unavailable");close(fd);
  server.set_serial_ready(true);fd=connect_client(server.port());server.poll();check(server.connected(),"connected");
  wire::Bytes payload{0,0xaa,0x55,0xff,0x58,0,10,13};
  server.forward_rx(payload);auto n=recv(fd,b,sizeof(b),0);
  check(n==static_cast<ssize_t>(payload.size()) && wire::Bytes(b,b+n)==payload,"exact serial bytes without envelope");
  wire::Bytes incoming;
  for(auto value:payload){send(fd,&value,1,MSG_NOSIGNAL);auto part=server.poll();incoming.insert(incoming.end(),part.begin(),part.end());}
  for(int i=0;i<50 && incoming.size()<payload.size();++i){std::this_thread::sleep_for(std::chrono::milliseconds(2));auto p=server.poll();incoming.insert(incoming.end(),p.begin(),p.end());}
  check(incoming==payload,"fragmented commands delivered exactly");
  int second=connect_client(server.port());server.poll();check(recv(second,b,sizeof(b),0)==0,"second client rejected");close(second);
  close(fd);server.poll();check(!server.connected() && server.take_disconnect(),"disconnect event");
  server.forward_rx(payload);fd=connect_client(server.port(),"127.0.0.2");server.poll();
  check(server.connected(),"reconnect from a changed client IP");
  check(recv(fd,b,sizeof(b),MSG_DONTWAIT)<0,"no previous-session data replayed");
  server.forward_rx(wire::Bytes(256*1024+1,0x55));check(!server.connected()&&server.take_disconnect(),"slow-client bound");close(fd);
  fd=connect_client(server.port());server.poll();server.set_serial_ready(false);check(server.take_disconnect(),"serial loss disconnects client");close(fd);
  server.close();server.open("127.0.0.1",0,false);server.set_serial_ready(true);fd=connect_client(server.port());server.poll();
  send(fd,payload.data(),payload.size(),MSG_NOSIGNAL);
  for(int i=0;i<20 && !server.rejected_bytes();++i){check(server.poll().empty(),"read-only commands never forwarded");std::this_thread::sleep_for(std::chrono::milliseconds(2));}
  check(server.rejected_bytes()==payload.size(),"read-only rejected byte count");
  server.forward_rx(payload);check(recv(fd,b,sizeof(b),0)==static_cast<ssize_t>(payload.size()),"monitor remains usable");close(fd);
  std::cout<<"PASS "<<checks<<" TCP raw bridge checks\n";return 0;
}catch(const std::exception &e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}}
