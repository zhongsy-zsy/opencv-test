#ifndef _TEST_TEST_D435_H_
#define _TEST_TEST_D435_H_

#include <gtest/gtest.h>

#include <opencv2/opencv.hpp>
#include <vector>

#include "../realsense/common_op.h"
#include "../realsense/d435_driver.h"

TEST(mytest, test_max_thresholding) {
  cv::Mat depth_mean(2, 4, CV_16UC1);
  for (int i = 0; i < depth_mean.rows; i++) {
    for (int j = 0; j < depth_mean.cols; j++) {
      depth_mean.at<ushort>(i, j) = i + j;
    }
  }

  cv::Mat raw_data_1 = cv::Mat::ones(2, 4, CV_16UC1);
  std::vector<cv::Mat> raw_data;
  raw_data.push_back(raw_data_1);
  std::vector<double> test;
  test = calculate_max_threshold(depth_mean, raw_data);

  EXPECT_EQ(2, test[0]);
  EXPECT_EQ(3, test[1]);
}

#endif
