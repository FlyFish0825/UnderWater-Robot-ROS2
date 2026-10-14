// ==============================================================================
// src/global_camera/include/global_camera/stereo_transform.hpp
// ==============================================================================
// 双目拼接图像朝向处理：
// 当双目相机模块采用上下颠倒方式倒装在机器人前舱时，执行整幅图像 180° 旋转矫正。
// 规则：BGR8 格式，总宽度必须为偶数 (左半幅为左目、右半幅为右目)。
// 旋转 180° 后，原画面的左右半幅自动自然互换对齐。
// ==============================================================================
#pragma once

#include <opencv2/core.hpp>
#include <stdexcept>

namespace global_camera {

// rotate_180=false 时原样保留；true 时整幅画面旋转 180°
// 校验：输入非空、宽度 >= 2 且为偶数、格式为 CV_8UC3，不合规抛出 std::invalid_argument
inline void orient_stereo(cv::Mat &frame, bool rotate_180) {
  if (frame.empty() || frame.cols < 2 || (frame.cols % 2 != 0) || frame.type() != CV_8UC3) {
    throw std::invalid_argument("expected an even-width BGR8 stereo image");
  }

  if (!rotate_180) {
    return;
  }

  cv::Mat upright;
  cv::rotate(frame, upright, cv::ROTATE_180);
  frame = upright;
}

}  // namespace global_camera
