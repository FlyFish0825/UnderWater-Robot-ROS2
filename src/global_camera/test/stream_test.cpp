#include "global_camera/stream_server.hpp"
#include "global_camera/wire.hpp"
#include <arpa/inet.h>
#include <sys/socket.h>
#include <poll.h>
#include <unistd.h>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <vector>
using namespace std::chrono_literals;
namespace {
void require(bool condition,const char *message) { if(!condition)throw std::runtime_error(message); }
std::uint64_t get(const std::vector<std::uint8_t>& bytes,std::size_t start,std::size_t count) {
  std::uint64_t value=0;for(std::size_t i=0;i<count;++i)value=(value<<8)|bytes.at(start+i);return value;
}
int connect_to(std::uint16_t port) {
  int fd=socket(AF_INET,SOCK_STREAM,0);require(fd>=0,"client socket");
  sockaddr_in a{};a.sin_family=AF_INET;a.sin_port=htons(port);inet_pton(AF_INET,"127.0.0.1",&a.sin_addr);
  if(connect(fd,reinterpret_cast<sockaddr*>(&a),sizeof(a))<0){close(fd);throw std::runtime_error("client connect");}
  return fd;
}
std::vector<std::uint8_t> read_bytes(int fd,std::size_t size) {
  std::vector<std::uint8_t> out(size);std::size_t offset=0;
  while(offset<size) {
    pollfd p{fd,POLLIN,0};require(poll(&p,1,1500)>0,"client read timeout");
    // Small reads deliberately exercise fragmented stream delivery.
    const auto n=recv(fd,out.data()+offset,std::min<std::size_t>(31,size-offset),0);
    require(n>0,"unexpected client EOF");offset+=static_cast<std::size_t>(n);
  }
  return out;
}
std::uint64_t read_frame(int fd,const std::vector<std::uint8_t>& expected) {
  const auto header=read_bytes(fd,40);
  require(header[0]=='U' && header[1]=='W' && header[2]=='S' && header[3]=='C',"wire magic");
  require(get(header,4,2)==1 && get(header,6,2)==1,"version and side-by-side flag");
  require(get(header,16,8)==0x0102030405060708ULL,"64-bit receipt timestamp byte order");
  require(get(header,24,4)==2560 && get(header,28,4)==720,"dimensions byte order");
  require(get(header,32,4)==expected.size() && get(header,36,4)==0,"JPEG length and reserved");
  require(read_bytes(fd,expected.size())==expected,"JPEG bytes changed or frames interleaved");
  return get(header,8,8);
}
}
int main() {
  try {
    for(const auto size:{0U,8U*1024*1024+1}) {
      bool rejected=false;
      try { global_camera::make_header(1,2,2560,720,size); }
      catch(const std::invalid_argument&) {rejected=true;}
      require(rejected,"invalid JPEG size accepted");
    }
    bool rejected=false;try{global_camera::make_header(1,2,2559,720,12);}catch(const std::invalid_argument&){rejected=true;}
    require(rejected,"odd stereo width accepted");
    global_camera::StreamServer server("127.0.0.1",0);
    const std::vector<std::uint8_t> jpeg{0xff,0xd8,1,2,3,4,5,0xff,0xd9};
    const std::uint64_t stamp=0x0102030405060708ULL;
    // With no clients, old frames are replaced, never accumulated.
    for(unsigned i=1;i<=100;++i)server.publish(i,stamp,2560,720,jpeg);
    int a=connect_to(server.port()),b=connect_to(server.port());
    require(read_frame(a,jpeg)==100 && read_frame(b,jpeg)==100,"new clients did not get latest frame");
    pollfd idle{a,POLLIN,0};require(poll(&idle,1,100)==0,"same frame sent twice");
    server.publish(101,stamp,2560,720,jpeg);
    require(read_frame(a,jpeg)==101 && read_frame(b,jpeg)==101,"multi-client frame ordering");
    server.clear();
    pollfd closed{a,POLLIN,0};require(poll(&closed,1,1000)>0,"offline did not close client");
    char c;require(recv(a,&c,1,0)==0,"offline delivered stale frame");close(a);close(b);
    a=connect_to(server.port());idle={a,POLLIN,0};require(poll(&idle,1,100)==0,"reconnect got cached stale JPEG");
    server.publish(102,stamp,2560,720,jpeg);require(read_frame(a,jpeg)==102,"reconnect lost next frame");close(a);
    // A stalled consumer must not retain a partial multi-megabyte frame forever.
    server.clear();std::this_thread::sleep_for(50ms);
    a=connect_to(server.port());
    const std::vector<std::uint8_t> large(6*1024*1024,0x45);
    server.publish(103,stamp,2560,720,large);
    std::this_thread::sleep_for(2300ms);
    require(server.clients()==0,"slow client was not disconnected within bound");close(a);
    server.clear();std::this_thread::sleep_for(50ms);
    a=connect_to(server.port());server.publish(104,stamp,2560,720,jpeg);
    require(read_frame(a,jpeg)==104,"slow consumer damaged later client");close(a);
    require(server.healthy(),"stream worker failed");
    std::cout<<"PASS stereo TCP: framing, endian, latest-only, two clients, offline/reconnect, slow-client bound\n";
  } catch(const std::exception &e) {std::cerr<<"FAIL: "<<e.what()<<'\n';return 1;}
}
