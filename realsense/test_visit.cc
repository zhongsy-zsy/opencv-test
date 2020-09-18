#include <time.h>

#include <iostream>
#include <opencv2/opencv.hpp>

#include "common_op.h"

int main() {
  clock_t start, stop;
  double duration;
  double k = 0;
  cv::Mat database = cv::imread("mean_depth.png", cv::IMREAD_ANYDEPTH);
  ushort tmp;

  start = clock();
  for (int j = 0; j < 100; j++) {
    while (k != 1000000) {
      k++;
      for (int i = 0; i < database.rows; i++) {
        for (int j = 0; j < database.cols; j++) {
          tmp = database.at<ushort>(i, j);
        }
      }
    }
  }
  stop = clock();
  duration = static_cast<double>(stop - start) / 100.0;
  std::cout << RED;
  std::cout << "at: " << duration << std::endl;

  k = 0;
  start = clock();
  for (int j = 0; j < 100; j++) {
    while (k != 1000000) {
      k++;
      for (int i = 0; i < database.rows; i++) {
        ushort *ptr_data = database.ptr<ushort>(i);
        for (int j = 0; j < database.cols; j++) {
          tmp = ptr_data[j];
        }
      }
    }
  }
  stop = clock();
  duration = static_cast<double>(stop - start) / 100;
  std::cout << RED;
  std::cout << "ptr: " << duration << std::endl;

  k = 0;
  start = clock();
  for (int j = 0; j < 100; j++) {
    while (k != 1000000) {
      k++;
      for (int i = 0; i < database.rows; i++) {
        ushort *ptr_data = database.ptr<ushort>(i);
        for (int j = 0; j < database.cols; j++) {
          tmp = *(database.data + database.step[0] * i + database.step[1] * j);
        }
      }
    }
  }
  stop = clock();
  duration = static_cast<double>(stop - start) / 100;
  std::cout << RED;
  std::cout << "data: " << duration << std::endl;
}