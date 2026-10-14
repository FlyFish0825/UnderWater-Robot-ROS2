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

// 断言计数，方便最后打印一共验证了多少条协议约定。
int checks = 0;

// check()：条件为假就抛异常，由 main 捕获统一处理。
void check(bool condition, const std::string &message) {
  ++checks;
  if (!condition) {
    throw std::runtime_error("check failed: " + message);
  }
}

}  // namespace

// 按编号依次验证：默认状态、申请、心跳、幂等与冲突、抢占、释放、手动模式、非法命令。
int main() {
  ControlOwnership ownership;

  // 1. 默认状态查询
  check(control_reply(ownership, 1, "STATE") == "STATE NANO\n", "initial state must be NANO");

  // 2. 正常申请 GUI 权限 (CLAIM GUI)
  auto reply = control_reply(ownership, 1, "CLAIM GUI");
  check(reply.rfind("OK GUI ", 0) == 0, "claim gui should succeed");
  check(control_reply(ownership, 1, "STATE") == "STATE GUI\n", "state should transition to GUI");

  // 3. 提取 Token 并测试心跳 (PING <token>)
  const auto token = ownership.gui_token();
  check(control_reply(ownership, 1, "PING " + token) == "HEARTBEAT GUI\n", "valid ping succeeds");
  check(control_reply(ownership, 1, "PING badtoken") == "REVOKED GUI\n", "bad token ping revoked");

  // 4. 重复声明幂等性与占用冲突测试
  check(control_reply(ownership, 1, "CLAIM GUI").rfind("OK GUI ", 0) == 0, "idempotent claim");
  check(control_reply(ownership, 2, "CLAIM GUI").rfind("BUSY GUI ", 0) == 0, "occupied claim fails");

  // 5. 抢占机制测试 (TAKEOVER GUI)
  check(control_reply(ownership, 2, "TAKEOVER GUI").rfind("OK GUI ", 0) == 0, "takeover succeeds");
  check(control_reply(ownership, 1, "PING " + token) == "REVOKED GUI\n", "old session revoked");

  // 6. 释放权限测试 (RELEASE GUI)
  const auto token2 = ownership.gui_token();
  check(control_reply(ownership, 2, "RELEASE GUI").rfind("OK NANO ", 0) == 0, "release returns to Nano");
  check(control_reply(ownership, 2, "STATE") == "STATE NANO\n", "state back to NANO");

  // 7. GUI_MANUAL 租约测试
  check(control_reply(ownership, 3, "CLAIM GUI_MANUAL").rfind("OK GUI_MANUAL ", 0) == 0, "claim gui manual succeeds");
  check(control_reply(ownership, 3, "STATE") == "STATE GUI_MANUAL\n", "state is GUI_MANUAL");
  check(control_reply(ownership, 3, "RELEASE GUI_MANUAL").rfind("OK NANO ", 0) == 0, "release gui manual");

  // 8. 非法语法防御测试
  check(control_reply(ownership, 3, "FOOBAR").rfind("ERROR ", 0) == 0, "unknown verb returns error");

  std::cout << "All " << checks << " control protocol tests passed successfully.\n";
  return 0;
}
