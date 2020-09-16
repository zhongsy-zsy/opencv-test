
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
using namespace cv;
using namespace std;

Mat polyfit(vector<Point>& in_point, int n);

int main() {
  //数据输入
  cv::Mat data_base =
      cv::imread("/home/zhongsy/Desktop/test/test_opencv/build/mean_depth.png",
                 IMREAD_ANYDEPTH);
  std::cout << data_base << std::endl;
  // vector<Point> in = {
  //     Point(50, 120), Point(74, 110), Point(98, 100), Point(122, 100),
  //     Point(144, 80), Point(168, 80), Point(192, 70), Point(214, 50),
  //     Point(236, 40), Point(262, 20), Point(282, 20), Point(306, 30),
  //     Point(328, 40), Point(356, 50), Point(376, 50), Point(400, 50),
  //     Point(424, 50), Point(446, 40), Point(468, 30)};

  //   std::vector<std::vector<cv::Point>> nihe_sample;
  //   for (int i = 0; i < data_base.rows; i++) {
  //     std::vector<cv::Point> tmp;
  //     for (int j = 0; j < data_base.cols; j++) {
  //       tmp.push_back(cv::Point(j, data_base.at<ushort>(i, j)));
  //     }
  //     nihe_sample.push_back(tmp);
  //     tmp.clear();
  //   }
  //   for (auto a : nihe_sample[340]) {
  //     std::cout << a << std::endl;
  //   }
  //   vector<Point> in_point(begin(in), end(in));

  // 找最大值拟合
  std::vector<cv::Point> nihe_samples;
  for (int i = 0; i < 480; i++) {
    std::vector<ushort> role;
    for (int j = 0; j < data_base.cols; j++) {
      role.push_back(data_base.at<ushort>(i, j));
    }
    ushort max = *max_element(role.begin(), role.end());
    std::cout << max << std::endl;
    nihe_samples.push_back(cv::Point(i, max));
    role.clear();
  }

  // n:多项式阶次
  int n = 9;
  Mat mat_k = polyfit(nihe_samples, n);

  //计算结果可视化
  Mat out(15000, 10000, CV_8UC3, Scalar::all(0));

  //画出拟合曲线
  for (int i = 0; i < 480; ++i) {
    Point2d ipt;
    ipt.x = i;
    ipt.y = 0;
    for (int j = 0; j < n + 1; ++j) {
      ipt.y += mat_k.at<double>(j, 0) * pow(i, j);
    }
    std::cout << nihe_samples[i] << std::endl;
    std::cout << ipt << std::endl;
    std::cout << std::endl;
    circle(out, ipt, 5, Scalar(255, 255, 255), CV_FILLED, CV_AA);
  }

  //画出原始散点
  for (int i = 0; i < 480; ++i) {
    Point ipt = nihe_samples[i];
    circle(out, ipt, 5, Scalar(0, 0, 255), CV_FILLED, CV_AA);
  }
  cv::namedWindow("image", WINDOW_NORMAL);
  imshow("image", out);
  waitKey();

  return 0;
}

Mat polyfit(vector<Point>& in_point, int n) {
  int size = in_point.size();
  //所求未知数个数
  int x_num = n + 1;
  //构造矩阵U和Y
  Mat mat_u(size, x_num, CV_64F);
  Mat mat_y(size, 1, CV_64F);

  for (int i = 0; i < mat_u.rows; ++i)
    for (int j = 0; j < mat_u.cols; ++j) {
      mat_u.at<double>(i, j) = pow(in_point[i].x, j);
    }

  for (int i = 0; i < mat_y.rows; ++i) {
    mat_y.at<double>(i, 0) = in_point[i].y;
  }

  // 矩阵运算，获得系数矩阵K
  Mat mat_k(x_num, 1, CV_64F);
  mat_k = (mat_u.t() * mat_u).inv() * mat_u.t() * mat_y;
  cout << mat_k << endl;
  return mat_k;
}