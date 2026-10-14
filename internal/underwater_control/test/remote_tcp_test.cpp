#include "underwater_control/remote_tcp.hpp"
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <iostream>
#include <stdexcept>
namespace rm=underwater::control::remote;
static int checks=0;
static void check(bool value,const char *why) { ++checks; if(!value) throw std::runtime_error(why); }
static int client(std::uint16_t port) {
  const int fd=socket(AF_INET,SOCK_STREAM,0);
  sockaddr_in address{}; address.sin_family=AF_INET; address.sin_port=htons(port);
  inet_pton(AF_INET,"127.0.0.1",&address.sin_addr);
  if(fd<0 || connect(fd,reinterpret_cast<sockaddr *>(&address),sizeof(address))!=0)
    throw std::runtime_error("loopback connection failed");
  timeval timeout{1,0}; setsockopt(fd,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout));
  return fd;
}
int main() {
  try {
    const int probe=socket(AF_INET,SOCK_STREAM,0);
    sockaddr_in address{}; address.sin_family=AF_INET; address.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
    check(bind(probe,reinterpret_cast<sockaddr *>(&address),sizeof(address))==0,"ephemeral loopback bind");
    socklen_t size=sizeof(address); check(getsockname(probe,reinterpret_cast<sockaddr *>(&address),&size)==0,"test port");
    const auto port=ntohs(address.sin_port); close(probe);
    rm::TcpServer server("127.0.0.1",port);
    int disconnected=0,connected=0;
    const auto handler=[&](auto event,const std::string &) {
      if(event==rm::TcpServer::Event::Connected) ++connected;
      if(event==rm::TcpServer::Event::Disconnected) ++disconnected;
    };
    char bytes[1024];
    const int first=client(port); server.poll(0,handler);
    const auto n=recv(first,bytes,sizeof(bytes),0);
    check(n>0 && std::string(bytes,n)=="HELLO ROV_MANUAL 2\n","v2 read-only greeting");
    const int second=client(port); server.poll(1,handler);
    check(server.generation()==2 && connected==2 && disconnected==1,"unverified peer cannot monopolize slot");
    check(recv(first,bytes,sizeof(bytes),0)==0,"unverified peer replaced"); close(first);
    check(recv(second,bytes,sizeof(bytes),0)>0,"replacement greeted");
    server.reserve(true);
    const int third=client(port); server.poll(2,handler);
    check(server.generation()==2 && connected==2,"authenticated peer cannot be replaced");
    check(recv(third,bytes,sizeof(bytes),0)==0,"second controlling peer rejected"); close(third);
    server.reserve(false); server.poll(3001,handler);
    check(server.connected(),"already authenticated connection does not regain initial handshake deadline");
    server.drop();
    check(recv(second,bytes,sizeof(bytes),0)==0,"explicit drop closes peer"); close(second);
    const int fourth=client(port); server.poll(4000,handler);
    check(recv(fourth,bytes,sizeof(bytes),0)>0,"new unverified peer greeted");
    server.poll(7000,handler);
    check(!server.connected() && disconnected==2,"never-authenticated handshake has bounded deadline");
    check(recv(fourth,bytes,sizeof(bytes),0)==0,"deadline closes peer"); close(fourth);
    std::cout<<"remote_tcp: "<<checks<<" loopback checks passed\n";
  } catch(const std::exception &e) { std::cerr<<e.what()<<'\n'; return 1; }
}
