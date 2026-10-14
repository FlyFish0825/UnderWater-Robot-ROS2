// ==============================================================================
// internal/underwater_devices/src/sensors/imu/imu_decoder.cpp
// ==============================================================================
// IMU 帧解码：AA5B target=1、flags=8，按 command 区分三种负载：
// - 0x80: 原始数据 (40B)
// - 0x81: 姿态四元数与欧拉角 (32B)
// - 0x85: 气压计与相对高度 (20B)
// 各物理量的有效性由 device_status 位与有限性检查共同决定，无效时填 NaN。
// ==============================================================================
// 初学者导读（underwater_devices / sensors/imu/imu_decoder.cpp）
// 所属：IMU 帧解码实现。上游串口网关 → gateway/frames，下游是 devices 节点的话题。
// 谁调用谁：devices_node.cpp 的 receive() 调用本文件的 imu()。
// 协议：family 0x5b、target 1、flags 8；command 0x80/0x81/0x85 分别对应
//   原始九轴、融合姿态(四元数+欧拉角)、气压计。
// 设计原则：能拿到的量先填；拿不到或校验失败的物理量统一填 NaN，绝不用 0 冒充。
// C++ 概念：std::array::fill() 用同一个值填满数组；lambda 传给 std::all_of 做判断；
//   位运算 & 与 1<<n 用来读取设备状态字的某一位。
#include "underwater_devices/adapters.hpp"
#include "underwater_mcu_gateway/protocol.hpp"

#include <algorithm>
#include <cmath>

namespace underwater::devices {

// imu()：识别并解码一帧 IMU 数据；返回 false 时 message 不可用。
bool imu(const ui::GatewayFrame &f, ui::ImuState &m) {
  if (f.family != 0x5b || f.target != 1 || f.flags != 8 ||
      !((f.command == 0x80 && f.payload.size() == 40) ||
        (f.command == 0x81 && f.payload.size() == 32) ||
        (f.command == 0x85 && f.payload.size() == 20))) {
    return false;
  }

  m = ui::ImuState{};
  m.header = f.header;
  m.connection_id = f.connection_id;
  m.device_timestamp_us = f.device_timestamp_us;
  m.sequence = f.sequence;
  m.device_status = wire::u32(f.payload, 0);
  m.group = f.command;
  m.fresh = true;

  // 先把所有可能字段填成 NaN，之后只覆盖有效的字段。
  // 默认全部填充 NaN
  m.acceleration_m_s2.fill(invalid());
  m.angular_velocity_rad_s.fill(invalid());
  m.magnetic_protocol_units.fill(invalid());
  m.quaternion_wxyz.fill(invalid());
  m.euler_rpy_rad.fill(invalid());
  m.baro_pressure_pa = invalid();
  m.baro_temperature_c = invalid();
  m.baro_height_m = invalid();
  m.baro_reference_pa = invalid();

  // 0x80：九个 float（加速度 3 + 角速度 3 + 磁力计 3），从偏移 4 开始。
  // 1. 0x80: 原始传感器九轴数据 (加速度、角速度、磁力计)
  if (f.command == 0x80) {
    std::array<double, 9> v;
    for (std::size_t i = 0; i < v.size(); ++i) {
      v[i] = wire::f32(f.payload, 4 + i * 4);
    }

    // 状态字 bit1 为 1，且九个数都是有限值，才认为原始数据有效。
    // bit 1 为原始数据有效位，且九个分量必须全部是有限值
    m.raw_valid = (m.device_status & 2) &&
                  std::all_of(v.begin(), v.end(), [](double x) { return std::isfinite(x); });

    if (m.raw_valid) {
      for (std::size_t i = 0; i < 3; ++i) {
        m.acceleration_m_s2[i] = v[i] * 9.80665;       // g 转换为 m/s²
        m.angular_velocity_rad_s[i] = v[i + 3];         // rad/s
        m.magnetic_protocol_units[i] = v[i + 6];       // 磁力计原始协议单位
      }
    }
  // 0x81：前 4 个 float 是四元数 w,x,y,z，后 3 个是欧拉角。
  } else if (f.command == 0x81) {
    // 2. 0x81: 融合姿态数据 (四元数与欧拉角)
    std::array<double, 7> v;
    for (std::size_t i = 0; i < v.size(); ++i) {
      v[i] = wire::f32(f.payload, 4 + i * 4);
    }

    m.quaternion_valid = (m.device_status & 4) &&
                         std::all_of(v.begin(), v.begin() + 4, [](double x) { return std::isfinite(x); });

    // 四元数范数平方应接近 1，超出 0.5..1.5 视为坏解。
    // 模长平方限制在 0.5..1.5 之间，排除零向量与未归一化的坏解
    const double norm = v[0] * v[0] + v[1] * v[1] + v[2] * v[2] + v[3] * v[3];
    m.quaternion_valid = m.quaternion_valid && (norm > 0.5) && (norm < 1.5);

    m.euler_valid = (m.device_status & 8) &&
                    std::all_of(v.begin() + 4, v.end(), [](double x) { return std::isfinite(x); });

    if (m.quaternion_valid) {
      std::copy(v.begin(), v.begin() + 4, m.quaternion_wxyz.begin());
    }
    if (m.euler_valid) {
      std::copy(v.begin() + 4, v.end(), m.euler_rpy_rad.begin());
    }
  // 0x85：气压计高度、温度、气压与参考气压。
  } else {
    // 3. 0x85: 气压计数据 (height, temperature, pressure, reference)
    const double height = wire::f32(f.payload, 4);
    const double temperature = wire::f32(f.payload, 8);
    const double pressure = wire::f32(f.payload, 12);
    const double reference = wire::f32(f.payload, 16);

    // 每个量都必须状态位有效且数值有限（气压还要大于 0）。
    m.baro_pressure_valid = (m.device_status & (1u << 4)) && std::isfinite(pressure) && (pressure > 0);
    m.baro_temperature_valid = (m.device_status & (1u << 5)) && std::isfinite(temperature);
    m.baro_reference_valid = (m.device_status & (1u << 4)) && std::isfinite(reference) && (reference > 0);
    m.baro_height_valid = m.baro_pressure_valid && m.baro_reference_valid && std::isfinite(height);

    if (m.baro_pressure_valid) {
      m.baro_pressure_pa = pressure;
    }
    if (m.baro_temperature_valid) {
      m.baro_temperature_c = temperature;
    }
    if (m.baro_reference_valid) {
      m.baro_reference_pa = reference;
    }
    if (m.baro_height_valid) {
      m.baro_height_m = height;
    }
  }
  return true;
}

}  // namespace underwater::devices
