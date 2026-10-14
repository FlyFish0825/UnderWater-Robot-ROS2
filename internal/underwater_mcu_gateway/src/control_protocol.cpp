// ==============================================================================
// internal/underwater_mcu_gateway/src/control_protocol.cpp
// ==============================================================================
// 9002 管理通道控制协议解析实现：
// - 负责 STATE, CLAIM/TAKEOVER, RELEASE, PING 命令的解析
// - 将执行结果组装为规范应答行："<前缀> <OWNER> <token> <message>\n"
// ==============================================================================
#include "underwater_mcu_gateway/control_protocol.hpp"

namespace underwater {

namespace {

std::string owner_line(const char *prefix, const OwnershipResult &result) {
  return std::string(prefix) + " " + controller_name(result.owner) + " " +
         result.token + " " + result.message + "\n";
}

}  // namespace

std::string control_reply(ControlOwnership &ownership, const int session, const std::string &line) {
  // 先处理心跳超时，避免旧租约指令复活
  ownership.expire_gui();

  if (line == "STATE") {
    return std::string("STATE ") + controller_name(ownership.owner()) + "\n";
  }

  if (line == "CLAIM GUI" || line == "TAKEOVER GUI" ||
      line == "CLAIM GUI_MANUAL" || line == "TAKEOVER GUI_MANUAL") {
    const bool takeover = (line.rfind("TAKEOVER ", 0) == 0);
    const bool manual = (line == "CLAIM GUI_MANUAL" || line == "TAKEOVER GUI_MANUAL");
    const auto result = ownership.claim_gui(session, takeover, manual);
    return owner_line(result.success ? "OK" : "BUSY", result);
  }

  if (line == "RELEASE GUI" || line == "RELEASE GUI_MANUAL") {
    const auto expected = (line == "RELEASE GUI_MANUAL" ? ControllerOwner::GuiManual : ControllerOwner::Gui);
    if (ownership.owner() != expected) {
      return owner_line("BUSY", {false, ownership.owner(), {}, "Release mode does not match the current owner"});
    }
    const auto result = ownership.release_gui(session);
    return owner_line(result.success ? "OK" : "BUSY", result);
  }

  if (line.rfind("PING ", 0) == 0) {
    const auto prefix = ownership.heartbeat_gui(session, line.substr(5)) ? "HEARTBEAT " : "REVOKED ";
    return std::string(prefix) + controller_name(ownership.owner()) + "\n";
  }

  return "ERROR expected STATE, CLAIM/TAKEOVER/RELEASE GUI or GUI_MANUAL, or PING <token>\n";
}

}  // namespace underwater
