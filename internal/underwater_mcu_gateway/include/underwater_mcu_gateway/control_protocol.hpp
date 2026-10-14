// ==============================================================================
// internal/underwater_mcu_gateway/include/underwater_mcu_gateway/control_protocol.hpp
// ==============================================================================
// 9002 管理通道行协议解析：
// 处理 STATE / CLAIM GUI / TAKEOVER GUI / RELEASE GUI / PING <token> 等文本命令。
// ==============================================================================
#pragma once

#include "underwater_mcu_gateway/control_ownership.hpp"
#include <string>

namespace underwater {

// 解析单行 9002 指令并返回格式化响应文本 (transport 将 session 与底层套接字绑定)
std::string control_reply(ControlOwnership &ownership, int session, const std::string &line);

}  // namespace underwater
