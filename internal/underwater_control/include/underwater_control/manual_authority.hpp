// ==============================================================================
// internal/underwater_control/include/underwater_control/manual_authority.hpp
// ==============================================================================
// 手动控制权授权与租约证明机制：
// 在多源或网络控制场景下，防止未经授权的指令或冲突指令注入推力分配链路。
// 本地 ROS 计算图具备信任关系；网络控制权则通过 MCU 网关的 9002 租约独立校验。
// ==============================================================================
#pragma once

#include "underwater_control/remote_manual.hpp"
#include <string_view>

namespace underwater::control::remote {

// 校验控制源的排他合法性：
// - NANO 模式允许最多 1 个手动或 wrench 发布者；
// - GUI_MANUAL 模式要求恰好 1 个远程手动发布者、0 个 wrench 发布者，且仅允许专有远程链。
inline bool authorized_source(
    std::string_view owner,
    std::size_t manual_count,
    std::size_t wrench_count,
    bool only_remote_manual) {
  if (owner == "NANO") {
    return manual_count + wrench_count <= 1;
  }
  return owner == "GUI_MANUAL" && manual_count == 1 && wrench_count == 0 && only_remote_manual;
}

// 租约证明验证器：跟踪请求序列号、租约代际(generation)以及租约有效时长 (400ms 心跳超时)
class LeaseProof {
public:
  // 发起新一轮租约校验请求，记录开始时刻并递增请求序列
  std::uint64_t begin(Millis at) {
    pending_ = true;
    started_ = at;
    return ++request_;
  }

  // 取消当前等待中的租约请求
  void cancel_request() {
    ++request_;
    pending_ = false;
  }

  // 清除全部租约状态并复位代际
  void clear() {
    ++request_;
    pending_ = false;
    generation_ = 0;
    confirmed_ = false;
  }

  // 完成租约校验：核验序列号、有效性、租约代际一致性以及往返响应时间 (<400ms)
  bool complete(std::uint64_t request, bool valid, std::uint64_t generation, Millis at) {
    if (!pending_ || request != request_) {
      return false;
    }
    pending_ = false;
    if (!valid || !generation || (generation_ && generation_ != generation) || at - started_ >= 400) {
      clear();
      return false;
    }
    generation_ = generation;
    confirmed_ = true;
    refreshed_ = at;
    return true;
  }

  // 判定当前租约是否仍然有效（必须在 400ms 刷新窗口内）
  bool valid(Millis at) const {
    return confirmed_ && at >= refreshed_ && (at - refreshed_ < 400);
  }

  // 查询是否存在等待响应中的校验请求
  bool pending() const {
    return pending_;
  }

  // 检查请求是否已经超时 (>400ms 未应答)
  bool expired_request(Millis at) const {
    return pending_ && (at - started_ >= 400);
  }

private:
  std::uint64_t request_ = 0;      // 校验请求序号
  std::uint64_t generation_ = 0;   // 远端下发的租约代际标号
  Millis started_ = 0;             // 本次校验开始时刻 (毫秒)
  Millis refreshed_ = 0;           // 最近一次校验成功时刻 (毫秒)
  bool pending_ = false;           // 请求等待应答标志
  bool confirmed_ = false;         // 租约已确认有效标志
};

}  // namespace underwater::control::remote
