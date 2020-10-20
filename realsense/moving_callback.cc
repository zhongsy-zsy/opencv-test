#include <opencv2/opencv.hpp>
#include <vector>

const int Width = 848;
const int Height = 480;

/* 计算投影曲线 */
void GetProjCueve(cv::Mat src, std::vector<float>& result_W,
                  std::vector<float>& result_H) {
  for (int i = 0; i < src.rows; i++) {
    float tmp = 0;
    for (int j = 0; j < src.cols; j++) {
      tmp += src.at<float>(i, j);  // 计算每一列
    }
    result_H[i] = tmp;
  }
  for (int i = 0; i < src.cols; i++) {
    double tmp = 0;
    for (int j = 0; j < src.rows; j++) {
      tmp += src.at<float>(j, i);  // 计算每一行
    }
    result_W[i] = tmp;
  }
}

int main() {
  /* 读取三次图像 */
  cv::Mat src1 = cv::imread("/home/zhongsy/1.jpg");

  cv::Mat src2 = cv::imread("/home/zhongsy/2.jpg");

  cv::cvtColor(src1, src1, CV_RGB2GRAY);
  cv::cvtColor(src2, src2, CV_RGB2GRAY);
  cv::imwrite("1.jpg", src1);

  src1.convertTo(src1, CV_64FC1);
  src2.convertTo(src2, CV_64FC1);
  cv::imshow("src1", src1);
  cv::waitKey(300);
  cv::imshow("src2", src2);
  cv::waitKey(300);
  std::cout << src2.type() << std::endl;

  std::vector<float> Result1_H(src1.rows, 0);
  std::vector<float> Result1_W(src1.cols, 0);
  std::vector<float> Result2_H(src2.rows, 0);
  std::vector<float> Result2_W(src2.cols, 0);
  std::vector<float> Result;

  GetProjCueve(src1, Result1_W, Result1_H);
  //   for (auto value : Result1_H) {
  //     std::cout << value << " ";
  //   }
  //   std::cout << std::endl;
  //   for (auto value : Result1_W) {
  //     std::cout << value << " ";
  //   }
  //   std::cout << std::endl;
  GetProjCueve(src2, Result2_W, Result2_H);
  std::vector<float> Result2_H_tmp =
      std::vector<float>(Result2_H.begin() + 200, Result2_H.end() - 200);

  cv::matchTemplate(Result1_H, Result2_H_tmp, Result, 5);
  for (auto value : Result) {
    std::cout << value << " ";
  }
  std::vector<float>::iterator itmax =
      std::max_element(Result.begin(), Result.end());
  int dis = std::distance(Result.begin(), itmax);
  std::cout << "dis: " << dis << " " << Result.size() << std::endl;
  std::cout << std::endl;
  cv::Mat out1(src1.size(), CV_8UC1, cv::Scalar(0));
  cv::Mat roi = src2(cv::Rect(0, dis, src2.cols, src2.rows - dis));
  roi.copyTo(out1(cv::Rect(0, 0, src2.cols, src2.rows - dis)));
  cv::imshow("out", out1);
  cv::imwrite("2.jpg", out1);
  
}