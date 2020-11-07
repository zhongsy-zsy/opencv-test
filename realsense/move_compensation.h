#ifndef _MOVE_COMPENSATION_H_
#define _MOVE_COMPENSATION_H_

#include <algorithm>
#include <iostream>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

namespace nameof_move_compensation {
// const std::vector<std::vector<int>> nine{{0, 0},  {0, -1}, {0, 1},
//                                          {-1, 0}, {1, 0},  {-1, -1},
//                                          {1, 1},  {1, -1}, {-1, 1}};

//创建一个梯形
const std::vector<std::vector<int>> nine{
    {0, 0},   {0, -1},  {0, -2}, {0, -3}, {0, 1},  {0, 2},   {0, 3}, {-1, 0},
    {-1, -1}, {-1, -2}, {-1, 1}, {-1, 2}, {-2, 0}, {-2, -1}, {-2, 1}};

const std::vector<std::vector<int>> Direction_matrix = {
    {0, -1}, {-1, 0}, {-1, -1}};

class move_compensation {
 public:
  move_compensation() {}
  move_compensation(rs2_intrinsics depth_intrin, float ration_angle)
      : depth_intrin_(depth_intrin), ration_angle_(ration_angle) {}
  ~move_compensation() {}
  virtual std::vector<cv::Mat> compensation_motion(std::vector<cv::Mat>& data,
                                                   double delta_z);

 private:
  //   std::vector<cv::Mat> data_;
  rs2_intrinsics depth_intrin_;
  float ration_angle_;
  cv::Mat depth_data_;
};

class move_compensation_method1 : public move_compensation {
 public:
  move_compensation_method1() {}
  ~move_compensation_method1() {}
  void compensation_motion(std::vector<cv::Mat>& data);
};

}  // namespace nameof_move_compensation
#endif  // REALSENSE_MOVE_COMPENSATION_H_
