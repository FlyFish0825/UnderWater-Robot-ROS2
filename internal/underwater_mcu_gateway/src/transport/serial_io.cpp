// ==============================================================================
// internal/underwater_mcu_gateway/src/transport/serial_io.cpp
// ==============================================================================
// 串口底层异步 I/O 驱动实现：
// - 配置 termios 虚拟串口模式波特率无需配置 8N1 raw 模式
// - 使用 TIOCEXCL 独占串口，防止多进程抢占冲突
// - 封装超时可控的非阻塞读取 (read_available) 与写出 (write_all)
// ==============================================================================
#include "underwater_mcu_gateway/serial_io.hpp"

#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <stdexcept>
#include <sys/ioctl.h>
#include <unistd.h>

namespace underwater {

void SerialIo::open(const std::string &path, bool allow_write) {
  close();

  fd_ = ::open(
      path.c_str(),
      (allow_write ? O_RDWR : O_RDONLY) | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0) {
    throw std::runtime_error(std::strerror(errno));
  }
  writable_ = allow_write;

  termios cfg{};
  if (tcgetattr(fd_, &cfg) != 0) {
    close();
    throw std::runtime_error("tcgetattr failed");
  }

  if (ioctl(fd_, TIOCEXCL) != 0) {
    close();
    throw std::runtime_error("exclusive serial open failed");
  }

  // 配置为原始模式 (8N1, 115200 波特率)
  cfmakeraw(&cfg);
  cfsetispeed(&cfg, B115200);
  cfsetospeed(&cfg, B115200);

  cfg.c_iflag &= ~(IXOFF | IXANY);
  cfg.c_cflag = (cfg.c_cflag & ~(CSIZE | PARENB | CSTOPB | CRTSCTS | HUPCL)) | CS8 | CREAD | CLOCAL;
  cfg.c_cc[VMIN] = 0;
  cfg.c_cc[VTIME] = 0;

  if (tcsetattr(fd_, TCSANOW, &cfg) != 0) {
    close();
    throw std::runtime_error("tcsetattr failed");
  }
}

void SerialIo::close() noexcept {
  if (fd_ >= 0) {
    // 释放独占并关闭
    ioctl(fd_, TIOCNXCL);
    ::close(fd_);
  }
  fd_ = -1;
  writable_ = false;
}

wire::Bytes SerialIo::read_available() {
  wire::Bytes bytes;
  if (fd_ < 0) {
    return bytes;
  }

  pollfd p{fd_, POLLIN, 0};
  const int ready = poll(&p, 1, 0);
  if (ready < 0 && errno != EINTR) {
    throw std::runtime_error("serial poll failed");
  }
  if (ready <= 0) {
    return bytes;
  }

  if (p.revents & (POLLHUP | POLLERR | POLLNVAL)) {
    throw std::runtime_error("serial disconnected");
  }

  std::uint8_t chunk[4096];
  while (bytes.size() < 65536) {
    const auto n = ::read(fd_, chunk, sizeof(chunk));
    if (n > 0) {
      bytes.insert(bytes.end(), chunk, chunk + n);
      continue;
    }
    if (n == 0 || errno == EAGAIN || errno == EWOULDBLOCK) {
      break;
    }
    if (errno == EINTR) {
      continue;
    }
    throw std::runtime_error(std::strerror(errno));
  }
  return bytes;
}

void SerialIo::write_all(const wire::Bytes &bytes, int timeout_ms) {
  if (fd_ < 0 || !writable_) {
    throw std::runtime_error("serial TX is disabled");
  }

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  std::size_t offset = 0;

  while (offset < bytes.size()) {
    const auto n = ::write(fd_, bytes.data() + offset, bytes.size() - offset);
    if (n > 0) {
      offset += static_cast<std::size_t>(n);
      continue;
    }
    if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
      throw std::runtime_error(std::strerror(errno));
    }

    const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(
        deadline - std::chrono::steady_clock::now()).count();
    if (left <= 0) {
      throw std::runtime_error("serial TX deadline exceeded");
    }

    pollfd p{fd_, POLLOUT, 0};
    if (::poll(&p, 1, static_cast<int>(left)) < 0 && errno != EINTR) {
      throw std::runtime_error("serial TX poll failed");
    }
  }
}

}  // namespace underwater
