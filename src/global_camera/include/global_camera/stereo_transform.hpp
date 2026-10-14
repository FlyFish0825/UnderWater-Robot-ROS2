// ==============================================================================
// src/global_camera/include/global_camera/stereo_transform.hpp
// ==============================================================================
// 双目拼接图像朝向处理：
// 当双目相机模块采用上下颠倒方式倒装在机器人前舱时，执行整幅图像 180° 旋转矫正。
// 规则：BGR8 格式，总宽度必须为偶数 (左半幅为左目、右半幅为右目)。
// 旋转 180° 后，原画面的左右半幅自动自然互换对齐。
// ==============================================================================

// ── 初学者导读 ────────────────────────────────────────────────────────────
// 这个头文件只做一件小事：在需要时把整幅双目图像旋转 180°。
// 为什么需要它？相机可能“倒装”在机器人前舱，图像上下颠倒；旋转 180° 后，
// 原本的左半幅/右半幅会自动换到对面，从而与“左目在左、右目在右”的约定重新对齐。
// 本文件没有 ROS 2 依赖，是纯 OpenCV 图像处理函数，常被 global_cam_pub 调用。
// 会出现的概念：
//   - cv::Mat：OpenCV 的图像矩阵；cols=宽、rows=高，type() 描述像素格式（CV_8UC3 = 8 位 3 通道 BGR）。
//   - inline：建议编译器把函数体展开到调用处的小函数，放在头文件里可被多处 include 而不冲突。
//   - 引用 &：形参 cv::Mat &frame 是“别名”，函数内改动会直接影响调用者那张图。
//   - 异常：非法输入时 throw；调用方要么用 try/catch，要么保证输入合法。
// ─────────────────────────────────────────────────────────────────────────

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
