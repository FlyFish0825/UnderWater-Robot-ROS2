#pragma once
#include <opencv2/core.hpp>
#include <stdexcept>
namespace global_camera {
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
