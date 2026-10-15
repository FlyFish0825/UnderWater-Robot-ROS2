// ==============================================================================
// internal/underwater_mcu_gateway/test/control_protocol_test.cpp
// ==============================================================================
// 9002 管理通道控制协议文本行解析单元测试：
// - 校验 STATE / CLAIM / TAKEOVER / RELEASE / PING 命令字
// - 校验 GUI 与 GUI_MANUAL 模式的状态互斥与应答格式
// ==============================================================================
// 【初学者导读】本文件是 9002 管理通道“文本协议解析”的单元测试，属于
// internal/underwater_mcu_gateway/test。它直接调用 control_reply(ownership, session, "命令")
// 这个纯函数，逐条验证 STATE/CLAIM/TAKEOVER/RELEASE/PING 等命令字与 GUI/GUI_MANUAL 两种模式，
// 不涉及真实网络和 ROS。可以把它当成“协议说明书”来读：每个 check 就是一条协议约定。
#include "underwater_mcu_gateway/control_ownership.hpp"
#include "underwater_mcu_gateway/control_protocol.hpp"

#include <iostream>
#include <stdexcept>
#include <string>

using namespace underwater;

namespace {

int checks = 0;

void check(bool condition, const std::string &message) {
  ++checks;
  if (!condition) {
    throw std::runtime_error("check failed: " + message);
  }
}

}  // namespace

int main() {
  try {
    ControlOwnership owner;
    auto request = [&](int session, const std::string &line) {
      return control_reply(owner, session, line);
    };

    // 1. 默认所有权归属 Nano
    check(request(10, "STATE") == "STATE NANO\n", "startup ownership remains Nano");

    // 2. 默认被占用时 CLAIM 不能抢占 Nano，必须用 TAKEOVER
    check(request(10, "CLAIM GUI_MANUAL").rfind("BUSY NANO", 0) == 0,
          "business claim does not preempt Nano");
    check(request(10, "TAKEOVER GUI_MANUAL").rfind("OK GUI_MANUAL ", 0) == 0,
          "business takeover accepted on 9002");

    const auto token = owner.gui_token();
    // 3. STATE 响应中绝不泄露 token
    check(request(10, "STATE") == "STATE GUI_MANUAL\n", "STATE never exposes tokens");

    // 4. 正确的会话和 token 才能心跳成功
    check(request(10, "PING " + token) == "HEARTBEAT GUI_MANUAL\n",
          "heartbeat reports actual business mode");
    check(request(11, "PING " + token) == "REVOKED GUI_MANUAL\n",
          "foreign socket cannot heartbeat copied token");

    // 5. 异构套接字不能释放其他会话的业务模式
    check(request(11, "RELEASE GUI_MANUAL").rfind("BUSY GUI_MANUAL", 0) == 0,
          "foreign socket cannot release business owner");
    check(request(10, "RELEASE GUI").rfind("BUSY GUI_MANUAL", 0) == 0,
          "release command must match active mode");

    // 6. 切换模式不能静默 CLAIM，必须显式 TAKEOVER 刷新 token
    check(request(10, "CLAIM GUI").rfind("BUSY GUI_MANUAL", 0) == 0,
          "same socket mode switch cannot silently claim");
    check(request(10, "TAKEOVER GUI").rfind("OK GUI ", 0) == 0 && owner.gui_token() != token,
          "explicit switch to raw mode rotates management token");

    // 7. 旧 token 被拒绝，新 token 生效
    check(request(10, "PING " + token) == "REVOKED GUI\n", "old mode token is rejected");
    check(request(10, "PING " + owner.gui_token()) == "HEARTBEAT GUI\n", "legacy raw heartbeat preserved");

    // 8. 释放后控制权归还 Nano
    check(request(10, "RELEASE GUI").rfind("OK NANO", 0) == 0, "raw release restores Nano");
    check(request(10, "TAKEOVER GUI_MANUAL").rfind("OK GUI_MANUAL ", 0) == 0,
          "business mode can be reacquired");
    check(request(10, "RELEASE GUI_MANUAL").rfind("OK NANO", 0) == 0, "business release restores Nano");

    // 9. 异常语法与隔离检查
    check(request(10, "TAKEOVER GUI_MANUAL extra").rfind("ERROR", 0) == 0,
          "extra arguments rejected");
    check(request(10, "TAKEOVER GUI_MANUAL").rfind("OK", 0) == 0, "business granted for isolation check");

    const auto lease = owner.gui_token();
    auto busy = request(11, "CLAIM GUI_MANUAL");
    check(busy.rfind("BUSY GUI_MANUAL", 0) == 0 && busy.find(lease) == std::string::npos,
          "BUSY reply never exposes another client's token");

    std::cout << "All " << checks << " control protocol tests passed successfully.\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "FAIL " << e.what() << '\n';
    return 1;
  }
}
