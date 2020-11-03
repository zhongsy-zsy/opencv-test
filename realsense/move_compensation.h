#ifndef _MOVE_COMPENSATION_H_
#define _MOVE_COMPENSATION_H_

#include <algorithm>
#include <iostream>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

namespace nameof_move_compensation {
const std::vector<std::vector<int>> nine{{0, 0}, {-1, 0}, {1, 0},
                                         {0, 1}, {0, -1}, {-1, -1},
                                         {1, 1}, {-1, 1}, {1, -1}};
const std::vector<std::vector<int>> Direction_matrix = {
    {-1, 0}, {1, 0}, {-1, -1}};

class move_compensation {
 public:
  move_compensation() {}
  ~move_compensation() {}
  virtual std::vector<cv::Mat> compensation_motion(std::vector<cv::Mat>& data,
                                                   double delta_z);

 private:
  //   std::vector<cv::Mat> data_;
  rs2_intrinsics depth_intrin;
  float ration_angle;
};

class move_compensation_method1 : public move_compensation {
 public:
  move_compensation_method1() {}
  ~move_compensation_method1() {}
  std::vector<cv::Mat> compensation_motion(std::vector<cv::Mat>& data);
};

}  // namespace nameof_move_compensation
#endif  // REALSENSE_MOVE_COMPENSATION_H_
