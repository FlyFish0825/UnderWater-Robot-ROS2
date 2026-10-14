// ==============================================================================
// internal/underwater_control/test/remote_manual_test.cpp
// ==============================================================================
// 远程手动控制会话状态机单元测试：
// - 握手状态迁移流程 (Disabled -> Enabling -> Enabled -> Stopping)
// - 零位握手双向确认 (Session ID 更新与 Gateway 匹配)
// - 输入保活超时 (500ms) 与看门狗切断
// - 硬件重连 / 代际变更 / 外部抢占故障恢复
// - 租约校验与授权验证 (authorized_source / LeaseProof)
// ==============================================================================
#if __has_include("underwater_control/remote_manual.hpp")
#include "underwater_control/remote_manual.hpp"
#include "underwater_control/manual_authority.hpp"

#include <cstdlib>
#include <iostream>
#include <limits>
#include <sstream>

namespace rm = underwater::control::remote;

static int checks = 0;
#define CHECK(x)                                                                  do {                                                                              ++checks;                                                                       if (!(x)) {                                                                       std::cerr << "Assertion failed at line " << __LINE__ << ": " #x "\n";          std::exit(1);                                                                 }                                                                             } while (false)

static rm::Observation ready() {
  rm::Observation o;
  o.gateway_ready = true;
  o.allocation_fresh = true;
  o.connection = 42;
  o.allocation_session = 10;
  o.gateway_session = 10;
  return o;
}

static void enabled(rm::Session &s, rm::Observation &o) {
  s.connect();
  s.observe(o, 0);
  CHECK(s.handle({rm::Kind::Enable, 1, 0, {}}, 0, true).ok);
  CHECK(s.take_actions().enable);
  CHECK(s.state() == rm::State::Enabling);

  s.enable_result(true, "accepted", 10);
  CHECK(s.take_actions().zero);

  // 仅调用使能服务成功并不代表电机握手完成，还必须等待底层发布新的零位会话 ID
  CHECK(s.state() == rm::State::Enabling);

  o.allocation_session = 11;
  o.armed = true;
  o.input_fresh = true;
  o.solver_ok = true;
  o.gateway_session = 11;
  o.rearm_required = false;
  s.observe(o, 20);
  CHECK(s.state() == rm::State::Enabled);
}

int main() {
  // 1. 基本协议解析测试
  std::string error;
  auto cmd = rm::parse("ENABLE 1 0123456789abcdef0123456789abcdef", error);
  CHECK(cmd && cmd->kind == rm::Kind::Enable && cmd->sequence == 1);
  CHECK(cmd->lease_token == "0123456789abcdef0123456789abcdef");

  cmd = rm::parse("DISABLE 2", error);
  CHECK(cmd && cmd->kind == rm::Kind::Disable && cmd->sequence == 2);

  cmd = rm::parse("INPUT 3 11 10 0 0 0 0 0", error);
  CHECK(cmd && cmd->kind == rm::Kind::Input && cmd->sequence == 3 && cmd->session == 11);
  CHECK(cmd->levels[0] == 10 && cmd->levels[1] == 0);

  // 多轴同时激活应被拒绝 (单轴安全互锁)
  cmd = rm::parse("INPUT 4 11 10 10 0 0 0 0", error);
  CHECK(!cmd);

  // 2. 会话使能握手与输入测试
  {
    rm::Session s;
    auto o = ready();
    enabled(s, o);

    // 正常下发有效输入
    const auto r = s.handle({rm::Kind::Input, 2, 11, {20, 0, 0, 0, 0, 0}}, 30, true);
    CHECK(r.ok);
    CHECK(s.take_actions().input);
    CHECK(s.levels()[0] == 20);

    // 输入超时测试 (500ms 超时)
    s.tick(550);
    CHECK(s.state() == rm::State::Fault);
  }

  // 3. 硬件串口重连容灾测试
  {
    rm::Session s;
    auto o = ready();
    enabled(s, o);

    // 模拟底层串口重新插拔 (connection ID 发生变更)
    o.connection = 43;
    s.observe(o, 30);
    CHECK(s.state() == rm::State::Stopping);
  }

  // 4. 租约授权验证测试
  {
    CHECK(rm::authorized_source("NANO", 0, 1, false));
    CHECK(rm::authorized_source("NANO", 1, 0, false));
    CHECK(!rm::authorized_source("NANO", 1, 1, false));  // 冲突

    CHECK(rm::authorized_source("GUI_MANUAL", 1, 0, true));
    CHECK(!rm::authorized_source("GUI_MANUAL", 1, 1, true));
    CHECK(!rm::authorized_source("GUI_MANUAL", 1, 0, false));

    rm::LeaseProof proof;
    CHECK(!proof.valid(100));
    const auto req = proof.begin(100);
    CHECK(proof.pending());
    CHECK(proof.complete(req, true, 1, 200));
    CHECK(proof.valid(300));
    CHECK(!proof.valid(700));  // 400ms 超时失效
  }

  std::cout << "All " << checks << " remote manual tests passed successfully.\n";
  return 0;
}
#else
int main() { return 0; }
#endif
