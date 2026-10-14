// ==============================================================================
// internal/underwater_devices/src/sensors/imu/imu_decoder.cpp
// ==============================================================================
// IMU 帧解码：AA5B target=1、flags=8，按 command 区分三种负载：
// - 0x80: 原始数据 (40B)
// - 0x81: 姿态四元数与欧拉角 (32B)
// - 0x85: 气压计与相对高度 (20B)
// 各物理量的有效性由 device_status 位与有限性检查共同决定，无效时填 NaN。
// ==============================================================================
#include "underwater_devices/adapters.hpp"
#include "underwater_mcu_gateway/protocol.hpp"

#include <algorithm>
#include <cmath>

namespace underwater::devices {

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

  // 1. 0x80: 原始传感器九轴数据 (加速度、角速度、磁力计)
  if (f.command == 0x80) {
    std::array<double, 9> v;
    for (std::size_t i = 0; i < v.size(); ++i) {
      v[i] = wire::f32(f.payload, 4 + i * 4);
    }

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
  } else if (f.command == 0x81) {
    // 2. 0x81: 融合姿态数据 (四元数与欧拉角)
    std::array<double, 7> v;
    for (std::size_t i = 0; i < v.size(); ++i) {
      v[i] = wire::f32(f.payload, 4 + i * 4);
    }

    m.quaternion_valid = (m.device_status & 4) &&
                         std::all_of(v.begin(), v.begin() + 4, [](double x) { return std::isfinite(x); });

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
  } else {
    // 3. 0x85: 气压计数据 (height, temperature, pressure, reference)
    const double height = wire::f32(f.payload, 4);
    const double temperature = wire::f32(f.payload, 8);
    const double pressure = wire::f32(f.payload, 12);
    const double reference = wire::f32(f.payload, 16);

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
