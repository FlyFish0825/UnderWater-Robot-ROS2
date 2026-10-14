#pragma once

#include "underwater_mcu_gateway/control_ownership.hpp"
#include <string>

namespace underwater {
// One already bounded/CR-stripped 9002 line. The transport binds session to the
// accepted socket; tokens are returned only to successful claim/takeover calls.
std::string control_reply(ControlOwnership &ownership, int session, const std::string &line);
}  // namespace underwater
