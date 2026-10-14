// ==============================================================================
// src/global_camera/test/stereo_transform_test.cpp
// ==============================================================================
// orient_stereo() 单元测试：
// - rotate_180=false 保持原图逐像素不变；
// - rotate_180=true 时整幅画面旋转 180° (左右半幅自然互换且上下翻转)；
// - 连续旋转两次可无损还原原图；
// - 奇数宽度或非 CV_8UC3 图像抛出 std::invalid_argument 异常。
// ==============================================================================

// ── 初学者导读 ────────────────────────────────────────────────────────────
// 这是一个不依赖 ROS 的单元测试：直接用几张很小的假图验证 orient_stereo() 的行为，
// 包括“不旋转保持原样、旋转 180°、旋转两次还原、奇数宽度要报错”。
// C++ 概念：cv::Mat 图像矩阵、cv::Vec3b 三通道像素、try/catch 断言异常。
// 返回值 0 表示通过；非 0（1/2/3/4）分别对应某个断言失败。
// ─────────────────────────────────────────────────────────────────────────

#include "global_camera/stereo_transform.hpp"
#include <iostream>

// 测试入口：逐条执行 4 个用例，任一失败即返回非 0 退出码。
int main() {
  // 创建 2x4 拼接测试图：左半幅 [[1, 2], [3, 4]]，右半幅 [[11, 12], [13, 14]]
  cv::Mat image(2, 4, CV_8UC3);
  const unsigned char source[2][4] = {{1, 2, 11, 12}, {3, 4, 13, 14}};
  for (int y = 0; y < 2; ++y) {
    for (int x = 0; x < 4; ++x) {
      image.at<cv::Vec3b>(y, x) = cv::Vec3b::all(source[y][x]);
    }
  }

  const auto original = image.clone();

  // 1. 关闭旋转：必须逐像素不变
  global_camera::orient_stereo(image, false);
  if (cv::norm(image, original, cv::NORM_INF) != 0) {
    return 1;
  }

  // 2. 开启旋转：左右半幅互换并各自上下翻转
  global_camera::orient_stereo(image, true);
  const unsigned char expected[2][4] = {{14, 13, 4, 3}, {12, 11, 2, 1}};
  for (int y = 0; y < 2; ++y) {
    for (int x = 0; x < 4; ++x) {
      if (image.at<cv::Vec3b>(y, x) != cv::Vec3b::all(expected[y][x])) {
        return 2;
      }
    }
  }

  // 3. 旋转两次应还原
  global_camera::orient_stereo(image, true);
  if (cv::norm(image, original, cv::NORM_INF) != 0) {
    return 3;
  }

  // 4. 奇数宽度必须被拒绝
  bool rejected = false;
  try {
    cv::Mat odd(2, 3, CV_8UC3);
    global_camera::orient_stereo(odd, true);
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  if (!rejected) {
    return 4;
  }

  std::cout << "PASS stereo orientation: whole frame rotated 180, halves exchanged, opt-out, invalid dimensions\n";
  return 0;
}
