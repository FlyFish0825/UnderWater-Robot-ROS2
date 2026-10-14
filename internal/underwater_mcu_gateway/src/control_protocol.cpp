// ==============================================================================
// internal/underwater_mcu_gateway/src/control_protocol.cpp
// ==============================================================================
// 9002 管理通道控制协议解析实现：
// - 负责 STATE, CLAIM/TAKEOVER, RELEASE, PING 命令的解析
// - 将执行结果组装为规范应答行："<前缀> <OWNER> <token> <message>\n"
// ==============================================================================
// 初学者导读：
// 1) 本文件实现 control_protocol.hpp：解析 9002 管理通道的文本命令行并生成应答。
// 2) 交互：TcpControlServer::handle_line() 调用 control_reply()；后者再调用 ControlOwnership。
// 3) ROS 2 概念：无 ROS API，纯文本协议。
// 4) C++ 概念：匿名 namespace 内部函数；std::string 拼接；rfind 前缀匹配。
#include "underwater_mcu_gateway/control_protocol.hpp"

namespace underwater {

namespace {

// 统一应答格式："前缀 所有者 token 说明\n"（Nano 时 token 为空串）。
std::string owner_line(const char *prefix, const OwnershipResult &result) {
  return std::string(prefix) + " " + controller_name(result.owner) + " " +
         result.token + " " + result.message + "\n";
}

}  // namespace

// 主分发：识别 STATE / CLAIM / TAKEOVER / RELEASE / PING，其余一律回 ERROR。
std::string control_reply(ControlOwnership &ownership, const int session, const std::string &line) {
  // 先处理心跳超时，避免旧租约指令复活
  ownership.expire_gui();

  if (line == "STATE") {
    return std::string("STATE ") + controller_name(ownership.owner()) + "\n";
  }

  if (line == "CLAIM GUI" || line == "TAKEOVER GUI" ||
      line == "CLAIM GUI_MANUAL" || line == "TAKEOVER GUI_MANUAL") {
// rfind(x, 0) == 0 等价于"以 x 开头"，这里判断命令是否为 TAKEOVER 抢占。
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
// 跳过 "PING " 的 5 个字符，剩下的是 GUI token。
    const auto prefix = ownership.heartbeat_gui(session, line.substr(5)) ? "HEARTBEAT " : "REVOKED ";
    return std::string(prefix) + controller_name(ownership.owner()) + "\n";
  }

  return "ERROR expected STATE, CLAIM/TAKEOVER/RELEASE GUI or GUI_MANUAL, or PING <token>\n";
}

}  // namespace underwater
