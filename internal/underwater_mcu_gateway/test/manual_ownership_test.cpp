// ==============================================================================
// internal/underwater_mcu_gateway/test/manual_ownership_test.cpp
// ==============================================================================
// 【初学者导读】本文件聚焦“GUI_MANUAL 手动业务权限”的租约规则，属于
// internal/underwater_mcu_gateway/test。验证令牌格式、500ms 证明有效期、
// 其他管理连接无法冒用/释放、模式切换轮换令牌、断线与 Nano 抢占回收权限等。
// 纯逻辑测试，不启动 ROS 或网络。涉及概念：32 位十六进制随机令牌、
// generation 授权代号、ControllerOwner 枚举、std::chrono 时间点。
#include "underwater_mcu_gateway/control_ownership.hpp"

#include <chrono>
#include <iostream>
#include <stdexcept>
#include <string>

using namespace underwater;
namespace {
int checks = 0;
// 断言：失败即抛异常，由 main 捕获。
void check(bool ok, const char *why) {
  ++checks;
  if (!ok) throw std::runtime_error(why);
}

// 手动业务权限的核心契约检查：在一个函数里集中验证多条规则。
void manual_contract(ControlOwnership &owner) {
  using Clock = ControlOwnership::Clock;
  using namespace std::chrono_literals;
  const auto before = owner.generation();
  auto grant = owner.claim_gui(10, true, true);
  check(grant.success && static_cast<unsigned>(grant.owner) == 3,
        "manual takeover owns GUI_MANUAL, not raw GUI");
  check(grant.token.size() == 32 && grant.token.find_first_not_of("0123456789abcdef") == std::string::npos,
        "lease token is 32 lowercase hexadecimal characters");
  const auto epoch = owner.generation();
  check(epoch != 0 && epoch != before, "takeover advances nonzero generation");
  const auto start = Clock::now();
  check(!owner.manual_authorized(start), "claim alone never authorizes ROS motor output");
  check(owner.validate_manual_lease("", start) == 0, "empty token rejected");
  check(owner.validate_manual_lease("wrong", start) == 0, "foreign token rejected");
  check(owner.validate_manual_lease(grant.token, start) == epoch, "matching business lease produces generation");
  check(owner.manual_authorized(start + 499ms), "proof valid before 500ms");
  check(!owner.manual_authorized(start + 500ms), "proof expires at 500ms boundary");
  check(owner.validate_manual_lease(grant.token, start + 100ms) == epoch, "valid proof renewal preserves generation");
  check(owner.validate_manual_lease("foreign", start + 200ms) == 0, "foreign renewal rejected");
  check(owner.manual_authorized(start + 599ms), "foreign request cannot revoke current holder");
  check(!owner.manual_authorized(start + 600ms), "foreign request cannot extend current holder");
  check(!owner.heartbeat_gui(11, grant.token), "another management session cannot heartbeat");
  check(!owner.release_gui(11).success, "another management session cannot release");
  check(!owner.claim_gui(11, false, true).success, "another session cannot share business ownership");
  check(owner.heartbeat_gui(10, grant.token), "business owner heartbeat accepted");
  check(owner.generation() == epoch, "heartbeat does not replace authority generation");
  check(!owner.claim_gui(10, false, false).success, "same session mode change requires takeover");
  check(owner.gui_token() == grant.token && owner.generation() == epoch, "rejected mode change preserves lease");
  auto raw = owner.claim_gui(10, true, false);
  check(raw.success && raw.owner == ControllerOwner::Gui && raw.token != grant.token,
        "explicit raw mode switch rotates token");
  check(owner.validate_manual_lease(raw.token) == 0 && owner.validate_manual_lease(grant.token) == 0,
        "raw and old business tokens cannot authorize manual output");
  check(!owner.manual_authorized(), "raw mode forbids ROS motor authority");
  auto next = owner.claim_gui(10, true, true);
  check(next.success && next.token != raw.token && next.token != grant.token,
        "returning to business mode creates fresh token");
  check(owner.validate_manual_lease(next.token) == owner.generation(), "new token accepted");
  const auto next_epoch = owner.generation();
  check(owner.claim_gui(10, false, true).token == next.token && owner.generation() == next_epoch,
        "idempotent claim preserves same mode lease");
  auto repeated = owner.claim_gui(10, true, true);
  check(repeated.token != next.token && owner.generation() != next_epoch,
        "explicit repeated takeover rotates business authority");
  check(!owner.manual_authorized() && owner.validate_manual_lease(next.token) == 0,
        "takeover clears proof and rejects old token");
  auto takeover = owner.claim_gui(12, true, true);
  check(takeover.success && owner.validate_manual_lease(repeated.token) == 0,
        "other session takeover revokes old business token");
  check(owner.validate_manual_lease(takeover.token) != 0, "current other session token validates");
  check(!owner.disconnect_gui(10), "old socket disconnect cannot release new holder");
  check(owner.disconnect_gui(12) && owner.owner() == ControllerOwner::Nano && !owner.manual_authorized(),
        "current disconnect returns Nano and revokes proof");
  auto expired = owner.claim_gui(13, true, true);
  const auto future = Clock::now() + 3s;
  check(owner.validate_manual_lease(expired.token, future) == 0,
        "validation cannot refresh an expired management heartbeat");
  check(!owner.manual_authorized(future), "expired management heartbeat also defeats existing proof");
  check(owner.expire_gui(future) && owner.owner() == ControllerOwner::Nano,
        "business management lease expires at three seconds");
  auto release = owner.claim_gui(14, true, true);
  check(owner.validate_manual_lease(release.token) != 0, "lease available for explicit release");
  check(owner.release_gui(14).success && !owner.manual_authorized(), "explicit release clears proof");
  auto nano = owner.claim_gui(15, true, true);
  check(owner.validate_manual_lease(nano.token) != 0, "lease available for Nano takeover");
  check(owner.claim_nano(true).success && owner.validate_manual_lease(nano.token) == 0,
        "Nano takeover revokes business lease");
}
}

// 先验证控制器名字映射与同一会话重复抢占会轮换令牌，再进入完整契约检查。
int main() {
  try {
    check(std::string(controller_name(static_cast<ControllerOwner>(3))) == "GUI_MANUAL",
          "business owner has distinct GUI_MANUAL wire name");
    ControlOwnership owner;
    auto first = owner.claim_gui(1, true);
    auto second = owner.claim_gui(1, true);
    check(first.success && second.success && first.token != second.token,
          "explicit same-session raw takeover rotates token");
    manual_contract(owner);
    std::cout << "PASS " << checks << " portable manual ownership checks\n";
    return 0;
  } catch(const std::exception &e) {
    std::cerr << "FAIL " << e.what() << '\n';
    return 1;
  }
}
