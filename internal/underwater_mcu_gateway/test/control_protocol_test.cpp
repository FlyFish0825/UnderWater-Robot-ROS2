#include "underwater_mcu_gateway/control_ownership.hpp"
#include "underwater_mcu_gateway/control_protocol.hpp"
#include <iostream>
#include <stdexcept>
#include <string>
using namespace underwater;
namespace {
int checks = 0;
void check(bool ok, const char *why) {
  ++checks;
  if (!ok) throw std::runtime_error(why);
}
}
int main() {
  try {
    ControlOwnership owner;
    auto request = [&](int session, const std::string &line) {
      return control_reply(owner, session, line);
    };
    check(request(10, "STATE") == "STATE NANO\n", "startup ownership remains Nano");
    check(request(10, "CLAIM GUI_MANUAL").rfind("BUSY NANO", 0) == 0,
          "business claim does not preempt Nano");
    check(request(10, "TAKEOVER GUI_MANUAL").rfind("OK GUI_MANUAL ", 0) == 0,
          "business takeover accepted on 9002");
    const auto token = owner.gui_token();
    check(request(10, "STATE") == "STATE GUI_MANUAL\n", "STATE never exposes tokens");
    check(request(10, "PING " + token) == "HEARTBEAT GUI_MANUAL\n", "heartbeat reports actual business mode");
    check(request(11, "PING " + token) == "REVOKED GUI_MANUAL\n", "foreign socket cannot heartbeat copied token");
    check(request(11, "RELEASE GUI_MANUAL").rfind("BUSY GUI_MANUAL", 0) == 0,
          "foreign socket cannot release business owner");
    check(request(10, "RELEASE GUI").rfind("BUSY GUI_MANUAL", 0) == 0,
          "release command must match active mode");
    check(request(10, "CLAIM GUI").rfind("BUSY GUI_MANUAL", 0) == 0,
          "same socket mode switch cannot silently claim");
    check(request(10, "TAKEOVER GUI").rfind("OK GUI ", 0) == 0 && owner.gui_token() != token,
          "explicit switch to raw mode rotates management token");
    check(request(10, "PING " + token) == "REVOKED GUI\n", "old mode token is rejected");
    check(request(10, "PING " + owner.gui_token()) == "HEARTBEAT GUI\n", "legacy raw heartbeat preserved");
    check(request(10, "RELEASE GUI").rfind("OK NANO", 0) == 0, "raw release restores Nano");
    check(request(10, "TAKEOVER GUI_MANUAL").rfind("OK GUI_MANUAL ", 0) == 0,
          "business mode can be reacquired");
    check(request(10, "RELEASE GUI_MANUAL").rfind("OK NANO", 0) == 0, "business release restores Nano");
    check(request(10, "TAKEOVER GUI_MANUAL extra").rfind("ERROR", 0) == 0,
          "extra arguments rejected");
    check(request(10, "TAKEOVER GUI_MANUAL").rfind("OK", 0) == 0, "business granted for isolation check");
    const auto lease = owner.gui_token();
    auto busy = request(11, "CLAIM GUI_MANUAL");
    check(busy.rfind("BUSY GUI_MANUAL", 0) == 0 && busy.find(lease) == std::string::npos,
          "BUSY reply never exposes another client's token");
    std::cout << "PASS " << checks << " portable 9002 command checks\n";
    return 0;
  } catch(const std::exception &e) {
    std::cerr << "FAIL " << e.what() << '\n';
    return 1;
  }
}
