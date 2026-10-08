#include "sensors_cpp/entry.hpp"
#include <iostream>
int main(int argc,char **argv) {
  if(argc!=4) return 64;
  try { return compatibility::backend_host(argv[1],argv[2],argv[3]); }
  catch(const std::exception &e) { std::cerr << e.what() << "\n"; return 1; }
}
