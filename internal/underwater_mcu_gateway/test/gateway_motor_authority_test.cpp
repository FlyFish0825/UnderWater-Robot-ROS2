#include "underwater_mcu_gateway/control_ownership.hpp"
#include "underwater_mcu_gateway/motor_commands.hpp"
#include "underwater_mcu_gateway/motor_authority.hpp"
#include <chrono>
#include <iostream>
#include <stdexcept>
using namespace underwater;
namespace {
int checks = 0;
void check(bool ok, const char *why) {
  ++checks;
  if (!ok) throw std::runtime_error(why);
}
void renewal_boundary_contract() {
  using namespace std::chrono_literals;
  ControlOwnership owner;
  wire::MotorGate gate;
  MotorAuthority authority(owner, gate);
  auto grant = owner.claim_gui(20, true, true);
  authority.synchronize();
  const auto start = ControlOwnership::Clock::now();
  check(owner.validate_manual_lease(grant.token, start) != 0, "boundary proof established");
  authority.synchronize(start);
  check(gate.accept(500, 1, true, 255, 0, {}, 1), "boundary old session handshakes");
  check(gate.accept(500, 2, true, 255, 1, {1200,0,0,0,0,0,0,0}, 2), "boundary old session active");
  const auto result = authority.renew_manual_lease(grant.token, start + 501ms);
  check(result.stop_required, "renewal across 499/501ms boundary cannot hide proof expiry");
  check(result.generation == owner.generation(), "valid token may renew after retiring old authority");
  check(gate.blocked() && !gate.active() && gate.session() == 0, "boundary renewal resets motor gate before granting proof");
  check(!gate.accept(500, 3, true, 255, 0, {}, 3), "boundary renewal retires old zero session");
  check(gate.accept(501, 1, true, 255, 0, {}, 4), "new session can explicitly handshake after expiry");

  auto timely = authority.renew_manual_lease(grant.token, start + 999ms);
  check(!timely.stop_required && gate.session() == 501 && !gate.blocked(),
        "renewal before expiry preserves current enabled session");
  auto exact = authority.renew_manual_lease(grant.token, start + 1499ms);
  check(exact.stop_required && exact.generation == owner.generation() && gate.blocked(),
        "renewal exactly at expiry retires current enabled session");
  check(!gate.accept(501, 2, true, 255, 0, {}, 5), "exact-expiry renewal rejects old zero handshake");
}
}
int main() {
  try {
    using namespace std::chrono_literals;
    renewal_boundary_contract();
    ControlOwnership owner;
    wire::MotorGate gate;
    MotorAuthority authority(owner, gate);
    check(authority.ros_motor_allowed(), "legacy Nano ROS authority preserved");
    check(!authority.synchronize(), "initial Nano requires no additional owner stop");
    wire::MotorRpm zeros{};
    wire::MotorRpm moving{1200, 0, 0, 0, 0, 0, 0, 0};
    auto manual = owner.claim_gui(10, true, true);
    check(authority.synchronize() && gate.blocked(), "mode change requests stop and resets gate");
    check(!authority.ros_motor_allowed(), "business mode alone forbids ROS motor commands");
    auto start = ControlOwnership::Clock::now();
    check(owner.validate_manual_lease(manual.token, start) != 0, "business proof established");
    check(!authority.synchronize(start) && authority.ros_motor_allowed(start),
          "proof permits Nano executor, without creating a motor session");
    check(gate.blocked(), "proof alone never enables motors");
    check(!gate.accept(100, 1, true, 255, 1, moving, 1), "fresh authority requires zero handshake");
    check(gate.accept(100, 1, true, 255, 0, zeros, 1), "zero handshake accepted under current proof");
    check(gate.accept(100, 2, true, 255, 1, moving, 2), "existing motor gate accepts subsequent vector");
    check(!authority.synchronize(start + 499ms), "live proof does not reset motor session");
    check(authority.synchronize(start + 500ms), "expired proof requests protective stop");
    check(!authority.ros_motor_allowed(start + 500ms) && gate.blocked() && !gate.active() && gate.session() == 0,
          "expiry independently rejects ROS commands and resets gate");
    check(!authority.synchronize(start + 501ms), "expired proof does not repeat stop continuously");
    check(owner.validate_manual_lease(manual.token, start + 501ms) != 0, "matching renewed lease can authorize again");
    check(!authority.synchronize(start + 501ms) && gate.blocked(), "renewal never restores old motor state");
    check(!gate.accept(100, 3, true, 255, 1, moving, 3), "renewed proof still requires fresh zero handshake");
    check(!gate.accept(100, 3, true, 255, 0, zeros, 3),
          "expired motor session cannot rearm itself using a delayed zero vector");
    auto raw = owner.claim_gui(10, true, false);
    check(authority.synchronize() && !authority.ros_motor_allowed(), "raw mode requests stop and blocks Nano ROS output");
    check(owner.validate_manual_lease(raw.token) == 0, "raw token cannot authorize executor");
    auto next = owner.claim_gui(10, true, true);
    check(authority.synchronize(), "business mode switch resets motor session");
    check(owner.validate_manual_lease(next.token) != 0, "new business lease validates");
    check(!authority.synchronize() && authority.ros_motor_allowed(), "new proof permits fresh handshaking");
    check(gate.accept(200, 1, true, 255, 0, zeros, 10), "new enable session starts with zero");
    check(owner.release_gui(10).success && authority.synchronize() && gate.blocked(),
          "release resets gate before restoring legacy Nano authority");
    check(authority.ros_motor_allowed(), "Nano authority restored, not old motor enable state");
    auto expiry = owner.claim_gui(11, true, true);
    check(authority.synchronize(), "heartbeat test starts new business authority");
    start = ControlOwnership::Clock::now();
    check(owner.validate_manual_lease(expiry.token, start + 2900ms) != 0, "proof near management expiry validates");
    check(!authority.synchronize(start + 2900ms), "near-expiry proof observed");
    check(authority.synchronize(start + 3001ms) && owner.owner() == ControllerOwner::Nano && gate.blocked(),
          "management expiry forces stop even when short proof would otherwise remain valid");
    owner.release_nano();
    check(authority.synchronize() && !authority.ros_motor_allowed(), "unowned mode forbids ROS output");
    std::cout << "PASS " << checks << " portable gateway motor authority checks\n";
    return 0;
  } catch(const std::exception &e) {
    std::cerr << "FAIL " << e.what() << '\n';
    return 1;
  }
}
