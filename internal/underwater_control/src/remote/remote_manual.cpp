// ==============================================================================
// internal/underwater_control/src/remote/remote_manual.cpp
// ==============================================================================
// 远程手动控制协议解析与会话状态机实现：
// - 文本行解析与单轴互锁校验 (valid_levels)
// - 安全受控使能、零位握手与自锁保护 (Session)
// - 心跳超时与异常切断 (tick / stop)
// ==============================================================================
#include "underwater_control/remote_manual.hpp"

#include <algorithm>
#include <charconv>
#include <sstream>
#include <utility>

namespace underwater::control::remote {

namespace {

// 将字符串切片解析为十进制整数（严禁以 '+' 开头，必须精确匹配完整字符串）
template <class T>
bool decimal(std::string_view text, T &value) {
  if (text.empty() || text.front() == '+') {
    return false;
  }
  const auto result = std::from_chars(text.data(), text.data() + text.size(), value, 10);
  return result.ec == std::errc{} && result.ptr == text.data() + text.size();
}

// 状态枚举转文本可读字符串
const char *name(State s) {
  switch (s) {
    case State::Disabled: return "DISABLED";
    case State::Enabling: return "ENABLING";
    case State::Enabled:  return "ENABLED";
    case State::Stopping: return "STOPPING";
    case State::Fault:    return "FAULT";
  }
  return "FAULT";
}

}  // namespace

// 校验手动档位：每个轴取值在 -100..100 之间，且同一时刻最多允许激活 1 个轴（单轴安全互锁）
bool valid_levels(const Levels &v) {
  unsigned active = 0;
  for (int n : v) {
    if (n < -100 || n > 100) {
      return false;
    }
    active += (n != 0);
  }
  return active <= 1;
}

// 协议行解析函数：解析客户端发送的 ENABLE, DISABLE, INPUT, AUTH 文本指令
std::optional<Command> parse(std::string_view line, std::string &error) {
  error = "invalid v2 command";
  if (line.empty() || line.size() > 1024) {
    return {};
  }

  // 严禁包含非 ASCII 可打印字符
  for (unsigned char c : line) {
    if (c < 32 || c > 126) {
      return {};
    }
  }

  // 按空格拆分 token 词素
  std::vector<std::string_view> tokens;
  for (std::size_t i = 0; i < line.size();) {
    if (line[i] == ' ') {
      ++i;
      continue;
    }
    const auto end = line.find(' ', i);
    tokens.push_back(line.substr(i, end == line.npos ? line.size() - i : end - i));
    if (tokens.size() > 9) {
      return {};
    }
    i = (end == line.npos ? line.size() : end);
  }

  if (tokens.size() < 2) {
    return {};
  }

  Command c{};
  // 所有指令的第 2 个参数均为单调递增的序列号 sequence (必须 > 0)
  if (!decimal(tokens[1], c.sequence) || c.sequence == 0) {
    return {};
  }

  // ENABLE 或 AUTH 命令：携带 32 位十六进制租约 token
  if ((tokens[0] == "ENABLE" || tokens[0] == "AUTH") && tokens.size() == 3) {
    if (tokens[2].size() != 32 ||
        !std::all_of(tokens[2].begin(), tokens[2].end(), [](char ch) {
          return (ch >= '0' && ch <= '9') || (ch >= 'a' && ch <= 'f');
        })) {
      return {};
    }
    c.kind = (tokens[0] == "AUTH" ? Kind::Auth : Kind::Enable);
    c.lease_token = std::string(tokens[2]);
  } else if (tokens[0] == "DISABLE" && tokens.size() == 2) {
    c.kind = Kind::Disable;
  } else if (tokens[0] == "INPUT" && tokens.size() == 9) {
    c.kind = Kind::Input;
    if (!decimal(tokens[2], c.session) || c.session == 0) {
      return {};
    }
    for (std::size_t i = 0; i < 6; ++i) {
      if (!decimal(tokens[i + 3], c.levels[i])) {
        return {};
      }
    }
    if (!valid_levels(c.levels)) {
      error = "levels must be integers -100..100 with at most one active axis";
      return {};
    }
  } else {
    return {};
  }

  error.clear();
  return c;
}

// 接收缓冲区追加数据并提取完整行
bool LineBuffer::append(std::string_view bytes, std::vector<std::string> &lines, std::string &error) {
  if (bytes.size() > 4096 || pending_.size() + bytes.size() > 4096) {
    error = "receive buffer exceeded";
    return false;
  }

  for (unsigned char c : bytes) {
    if ((c < 32 && c != '\r' && c != '\n') || c > 126) {
      error = "non-ASCII control byte";
      return false;
    }
  }

  pending_.append(bytes.data(), bytes.size());
  std::size_t end = 0;
  while ((end = pending_.find('\n')) != std::string::npos) {
    if (end > 1024) {
      error = "line exceeded 1024 bytes";
      return false;
    }
    auto line = pending_.substr(0, end);
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    lines.push_back(std::move(line));
    pending_.erase(0, end + 1);
  }

  if (pending_.size() > 1024) {
    error = "unfinished line exceeded 1024 bytes";
    return false;
  }
  return true;
}

// 将明细字符串转义编码为十六进制 ASCII 字符串，防御换行符污染
std::string hex_detail(std::string_view detail) {
  if (detail.empty()) {
    return "-";
  }
  constexpr char digits[] = "0123456789abcdef";
  std::string result;
  auto limit = std::min<std::size_t>(detail.size(), 256);

  // 截断时避免在 UTF-8 续字节中间截断造成乱码
  while (limit > 0 && limit < detail.size() && (static_cast<unsigned char>(detail[limit]) & 0xc0) == 0x80) {
    --limit;
  }

  for (unsigned char c : detail.substr(0, limit)) {
    result += digits[c >> 4];
    result += digits[c & 15];
  }
  return result;
}

// 构造 RESULT 响应文本行
std::string result_line(const Result &r) {
  return "RESULT " + std::to_string(r.sequence) + " " + (r.ok ? "1 " : "0 ") +
         std::to_string(r.session) + " " + hex_detail(r.detail) + "\n";
}

// 发送缓冲区追加输出数据
bool OutputBuffer::push(std::string_view bytes, Millis at) {
  if (bytes.size() > 4096 || bytes_.size() + bytes.size() > 4096) {
    return false;
  }
  if (bytes_.empty()) {
    started_ = at;
  }
  bytes_.append(bytes.data(), bytes.size());
  return true;
}

// 消费已成功发送的字节数据
void OutputBuffer::consume(std::size_t size) {
  bytes_.erase(0, size);
}

// 客户端连接接入处理
void Session::connect() {
  connected_ = true;
  sequence_ = 0;
  last_input_sequence_ = 0;
  clear_completed_fault();
}

// 复位已彻底结束的故障状态（若仍处于使能或停机中，则保留锁存）
void Session::clear_completed_fault() {
  if (!owned_ && !enable_pending_) {
    fault_ = false;
    state_ = State::Disabled;
    session_ = 0;
    levels_ = {};
    detail_ = "disabled; explicit authorized enable required";
    actions_ = {};
  }
}

// 客户端断开连接处理：触发紧急停机并置位故障
void Session::disconnect(Millis at) {
  connected_ = false;
  stop("TCP client disconnected", true, at);
}

// 提取并清空当前待执行动作
Actions Session::take_actions() {
  const auto a = actions_;
  actions_ = {};
  return a;
}

// 命令调度执行核心函数
Result Session::handle(const Command &c, Millis at, bool ready) {
  Result r{c.sequence, false, session_, ""};

  auto reject = [&](const char *why, bool fatal) {
    r.detail = why;
    if (fatal) {
      stop(why, true, at);
    }
    return r;
  };

  // AUTH 专属于租约校验适配器，严禁用于电机使能
  if (c.kind == Kind::Auth) {
    return reject("AUTH belongs to lease adapter, never motor enable", false);
  }

  // 严格核查序号单调性与连接状态
  if (!connected_ || c.sequence == 0 || c.sequence <= sequence_) {
    return reject("disconnected or duplicate/out-of-order sequence", true);
  }
  sequence_ = c.sequence;

  // 停用指令
  if (c.kind == Kind::Disable) {
    stop("operator disabled", false, at);
    r.ok = true;
    r.detail = detail_;
    return r;
  }

  // 使能申请指令
  if (c.kind == Kind::Enable) {
    if (owned_) {
      return reject("previous session still active/stopping", false);
    }
    if (!ready || !observation_.gateway_ready || !observation_.allocation_fresh || observation_.armed) {
      return reject("authenticated GUI_MANUAL TX gateway, disarmed allocator and exclusive source required", false);
    }

    // 登记使能请求并进入 Enabling 握手状态
    owned_ = true;
    enable_pending_ = true;
    disable_confirmed_ = false;
    fault_ = false;
    state_ = State::Enabling;
    started_ = at;
    baseline_session_ = observation_.allocation_session;
    connection_ = observation_.connection;
    session_ = 0;
    levels_ = {};
    last_input_sequence_ = 0;
    detail_ = "waiting for enable service and zero-session acknowledgement";
    actions_.enable = true;

    r.ok = true;
    r.session = 0;
    r.detail = detail_;
    return r;
  }

  // 手动档位输入指令
  if (state_ != State::Enabled || c.session != session_ || !valid_levels(c.levels)) {
    return reject("input before zero acknowledgement, stale session or invalid single-axis levels", true);
  }

  levels_ = c.levels;
  input_at_ = at;
  last_input_sequence_ = c.sequence;
  actions_.input = true;
  input_observation_ = observation_.allocation_update;

  r.ok = true;
  r.session = session_;
  r.detail = "accepted manual levels (not measured thrust)";
  return r;
}

// 触发停机或故障紧急断开
void Session::stop(const std::string &why, bool fault, Millis at) {
  levels_ = {};
  fault_ = fault_ || fault;
  detail_ = why;

  if (!owned_) {
    state_ = (fault_ ? State::Fault : State::Disabled);
    return;
  }
  if (state_ == State::Stopping) {
    return;
  }

  state_ = State::Stopping;
  actions_.zero = true;
  actions_.input = false;
  ++disable_generation_;
  disable_at_ = at;

  // 下发停机请求并等待底层确认
  actions_.disable = true;
}

// 使能服务调用结果回调
void Session::enable_result(bool ok, const std::string &why, Millis at) {
  if (!enable_pending_) {
    return;
  }
  enable_pending_ = false;

  if (!ok) {
    detail_ = why;
    fault_ = true;
    levels_ = {};
    if (state_ == State::Stopping) {
      finish_stop();
      return;
    }
    owned_ = false;
    state_ = State::Fault;
    actions_.release = true;
    return;
  }

  actions_.zero = true;
  // 若在握手期间客户端断开或主动停用，则立即转入停机流程
  if (state_ == State::Stopping || !connected_) {
    ++disable_generation_;
    disable_confirmed_ = false;
    state_ = State::Stopping;
    actions_.disable = true;
    disable_at_ = at;
  }
}

// 停机服务调用结果回调
void Session::disable_result(bool ok, Millis at) {
  disable_result(ok, at, disable_generation_);
}

void Session::disable_result(bool ok, Millis at, std::uint64_t generation) {
  if (state_ != State::Stopping || generation != disable_generation_) {
    return;
  }
  disable_confirmed_ = ok;
  disable_at_ = at;
  finish_stop();
}

// 确认停机全部完成并彻底释放资源
void Session::finish_stop() {
  if (state_ != State::Stopping || enable_pending_ || !disable_confirmed_ ||
      !observation_.allocation_fresh || observation_.armed || observation_.motor_active) {
    return;
  }
  owned_ = false;
  session_ = 0;
  levels_ = {};
  actions_.release = true;
  state_ = (fault_ ? State::Fault : State::Disabled);
}

// 观测底层状态并执行安全护栏检查
void Session::observe(const Observation &o, Millis at) {
  observation_ = o;
  if (!owned_) {
    return;
  }
  if (state_ == State::Stopping) {
    finish_stop();
    return;
  }

  // 网关掉线、推力分配超时或串口物理重连均视为致命异常
  if (!o.gateway_ready || !o.allocation_fresh || o.connection != connection_) {
    stop("gateway/source/allocator stale, unavailable or serial connection changed", true, at);
    return;
  }

  // Enabling 握手判定：推力分配器已武装、会话 ID 更新且已与网关一致、零位转速已就绪
  if (state_ == State::Enabling && !enable_pending_ && o.armed &&
      o.allocation_session != 0 && o.allocation_session != baseline_session_ &&
      o.gateway_session == o.allocation_session && !o.rearm_required && !o.motor_active) {
    session_ = o.allocation_session;
    state_ = State::Enabled;
    input_at_ = at;
    detail_ = "enabled; zero handshake confirmed (not motor rotation feedback)";
  } else if (state_ == State::Enabled &&
             (!o.armed ||
              (last_input_sequence_ != 0 && o.allocation_update > input_observation_ && !o.solver_ok) ||
              o.allocation_session != session_ ||
              o.gateway_session != session_ ||
              o.rearm_required)) {
    stop("allocator disarmed, solver failed or motor session lost", true, at);
  }
}

// 周期时钟节拍：超时巡检与重试
void Session::tick(Millis at) {
  // 握手 2.5 秒超时保护
  if (state_ == State::Enabling && at - started_ >= 2500) {
    stop("enable/zero handshake timed out", true, at);
  }
  // 控制输入 500 毫秒心跳保活超时保护
  if (state_ == State::Enabled && at - input_at_ >= 500) {
    stop("INPUT timed out; explicit enable required", true, at);
  }
  // 停机重试：若停机服务 500 毫秒未应答则重发停机
  if (state_ == State::Stopping && !disable_confirmed_ && at - disable_at_ >= 500) {
    actions_.disable = true;
    disable_at_ = at;
  }
}

// 构造向上层广播的 STATE 状态文本行
std::string state_line(const Session &s, std::uint64_t counter, const Observation &o) {
  const bool active = (s.state() == State::Enabled);
  std::ostringstream line;
  line << "STATE " << counter << ' ' << s.session() << ' ' << name(s.state()) << ' '
       << (active && o.armed) << ' ' << (active && o.input_fresh) << ' ' << o.gateway_ready << ' '
       << (o.allocation_fresh && o.solver_ok) << ' ' << s.last_input_sequence();

  // 档位数据 [l0..l5]
  for (int n : s.levels()) {
    line << ' ' << n;
  }
  // 电机转速 [rpm0..rpm7]
  for (int n : o.rpm) {
    line << ' ' << (active && o.input_fresh ? n : 0);
  }
  line << ' ' << hex_detail(s.detail()) << '\n';
  return line.str();
}

}  // namespace underwater::control::remote
