// stereo_transform.hpp —— 双目拼接图的朝向处理（上下颠倒安装时整幅旋转）。
// 输入约定：BGR8、宽度为偶数（左半为左目、右半为右目），否则抛异常。
#pragma once
#include <opencv2/core.hpp>
#include <stdexcept>
namespace global_camera {
// rotate_180=false 时原样返回；true 时整幅旋转 180°，左右半幅会随之互换。
// 校验：非空、宽度>=2 且为偶数、类型为 CV_8UC3，不满足则抛 std::invalid_argument。
inline void orient_stereo(cv::Mat &frame,bool rotate_180) {
  if(frame.empty() || frame.cols<2 || frame.cols%2 || frame.type()!=CV_8UC3)
    throw std::invalid_argument("expected an even-width BGR8 stereo image");
  if(!rotate_180)return;
  cv::Mat upright;
  // The entire camera is mounted upside down: rotate the complete sensor output.
  cv::rotate(frame,upright,cv::ROTATE_180);
  frame=upright;
}
}
