#include "global_camera/stereo_transform.hpp"
#include <iostream>
int main() {
  cv::Mat image(2,4,CV_8UC3);
  const unsigned char source[2][4]={{1,2,11,12},{3,4,13,14}};
  for(int y=0;y<2;++y)for(int x=0;x<4;++x)image.at<cv::Vec3b>(y,x)=cv::Vec3b::all(source[y][x]);
  const auto original=image.clone();
  global_camera::orient_stereo(image,false);
  if(cv::norm(image,original,cv::NORM_INF)!=0)return 1;
  global_camera::orient_stereo(image,true);
  const unsigned char expected[2][4]={{14,13,4,3},{12,11,2,1}};
  for(int y=0;y<2;++y)for(int x=0;x<4;++x)
    if(image.at<cv::Vec3b>(y,x)!=cv::Vec3b::all(expected[y][x]))return 2;
  global_camera::orient_stereo(image,true);
  if(cv::norm(image,original,cv::NORM_INF)!=0)return 3;
  bool rejected=false;
  try {cv::Mat odd(2,3,CV_8UC3);global_camera::orient_stereo(odd,true);}
  catch(const std::invalid_argument&){rejected=true;}
  if(!rejected)return 4;
  std::cout<<"PASS stereo orientation: whole frame rotated 180, halves exchanged, opt-out, invalid dimensions\n";
}
