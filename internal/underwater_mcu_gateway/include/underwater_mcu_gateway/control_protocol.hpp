// ==============================================================================
// internal/underwater_mcu_gateway/include/underwater_mcu_gateway/control_protocol.hpp
// ==============================================================================
// 9002 管理通道行协议解析：
// 处理 STATE / CLAIM GUI / TAKEOVER GUI / RELEASE GUI / PING <token> 等文本命令。
// ==============================================================================
// 初学者导读：
// 1) 所属：internal/underwater_mcu_gateway。它是 9002 管理通道的文本命令解析器，
//    负责"控制权"这套字符串协议，不碰串口原始字节。
// 2) 交互：TcpControlServer 每收到一整行就调用 control_reply()；本函数再调用
//    ControlOwnership 完成 CLAIM/TAKEOVER/RELEASE/PING。
// 3) ROS 2 概念：本文件与 ROS 话题无关，是网关内部的纯文本管理协议。
// 4) C++ 概念：std::string 按值返回（交给调用方，不共享）；引用参数 & 表示直接操作外部对象、不发生拷贝。
#pragma once

#include "underwater_mcu_gateway/control_ownership.hpp"
#include <string>

namespace underwater {

// 解析单行 9002 指令并返回格式化响应文本 (transport 将 session 与底层套接字绑定)
// 输入：ownership 仲裁器、session（客户端 TCP fd）、line（去掉换行的整行命令）。
// 输出：一行可直接回给客户端、且以 '\n' 结尾的应答文本。
// 坑：line 必须是完整一行；调用前应先 expire_gui() 处理心跳超时。
std::string control_reply(ControlOwnership &ownership, int session, const std::string &line);

}  // namespace underwater
