#include <opencv2/opencv.hpp>
#include <vector>

#include "move_compensation.h"

const int Width = 848;
const int Height = 480;

/* 计算投影曲线 */
void GetProjCueve(cv::Mat src, std::vector<float>& result_W,
                  std::vector<float>& result_H) {
  for (int i = 0; i < src.rows; i++) {
    float tmp = 0;
    for (int j = 0; j < src.cols; j++) {
      tmp += static_cast<float>(src.at<uchar>(i, j));  // 计算每一列
    }
    result_H[i] = tmp;
  }
  for (int i = 0; i < src.cols; i++) {
    double tmp = 0;
    for (int j = 0; j < src.rows; j++) {
      tmp += static_cast<float>(src.at<uchar>(j, i));  // 计算每一行
    }
    result_W[i] = tmp;
  }
}

#if 0
int main() {
  /* 读取三次图像 */
  cv::Mat src1 = cv::imread("/home/zhongsy/0.png", cv::IMREAD_ANYDEPTH);

  cv::Mat src2 = cv::imread("/home/zhongsy/2.png", cv::IMREAD_ANYDEPTH);
  //   src1.convertTo(src1, CV_8UC1);
  //   src2.convertTo(src2, CV_8UC1);

  //   cv::cvtColor(src1, src1, CV_8UC1);
  //   cv::cvtColor(src2, src2, CV_8UC1);
  // cv::imwrite("1.jpg", src1);

  //   src1.convertTo(src1, CV_64FC1);
  //   src2.convertTo(src2, CV_64FC1);
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
  std::vector<float> Result1_H_tmp =
      std::vector<float>(Result1_H.begin() + 20, Result1_H.end() - 20);

  cv::matchTemplate(Result2_H, Result1_H_tmp, Result, 5);
  std::cout << "value is: ";

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
  cv::imwrite("/home/zhongsy/result_2.png", out1);
}

#endif

#if 0

int main() {
  nameof_move_compensation::move_compensation_method1 method1;
  std::vector<cv::Mat> raw_datas;
  for (int i = 0; i < 3; i++) {  // 开始读取图片
    std::string file_name = "/home/zhongsy/" + std::to_string(i) + ".jpg";
    raw_datas.emplace_back(std::move(cv::imread(file_name)));
  }
  std::cout << "image size" << raw_datas.size() << std::endl;

  method1.compensation_motion(raw_datas);

  for (int i = 0; i < 3; i++) {  // 开始读取图片
    std::string file_name = "/home/zhongsy/res_" + std::to_string(i) + ".jpg";
    cv::imwrite(file_name, raw_datas[i]);
  }
}

#endif

#if 1

int main() {
  cv::Mat src1 = cv::imread("/home/zhongsy/0.png", cv::IMREAD_ANYDEPTH);
  cv::Mat src2 = cv::imread("/home/zhongsy/1.png", cv::IMREAD_ANYDEPTH);
  cv::Mat src3 = cv::imread("/home/zhongsy/2.png", cv::IMREAD_ANYDEPTH);
  cv::Mat res_src1 = cv::imread("/home/zhongsy/0.png", cv::IMREAD_ANYDEPTH);
  cv::Mat res_src2 =
      cv::imread("/home/zhongsy/result_1.png", cv::IMREAD_ANYDEPTH);
  cv::Mat res_src3 =
      cv::imread("/home/zhongsy/result_2.png", cv::IMREAD_ANYDEPTH);

  std::vector<cv::Mat> res_1;
  std::vector<cv::Mat> res_2;
  res_1.emplace_back(src1.clone());
  res_1.emplace_back(src2.clone());
  res_1.emplace_back(src3.clone());
  res_2.emplace_back(res_src1.clone());
  res_2.emplace_back(res_src2.clone());
  res_2.emplace_back(res_src3.clone());

  cv::Mat result_2(src1.size(), CV_8UC1, cv::Scalar(0));
  cv::Mat depth_sum(src1.size(), CV_8UC1, cv::Scalar(0));
  cv::Mat diff = src2 - src1;
  for (int i = 0; i < src1.rows; i++) {
    for (int j = 0; j < src1.cols; j++) {
      int count = 0;
      for (int k = 0; k < res_1.size(); k++) {
        if (res_1[k].at<uchar>(i, j) == 255) {
          count++;
        }
      }
      if (count > 1) {
        result_2.at<uchar>(i, j) = 255;
      }
      if (count != 0) {
        depth_sum.at<uchar>(i, j) = 255;
      }
    }
  }

  cv::imwrite("/home/zhongsy/result_3_1.png", result_2);
  cv::imwrite("/home/zhongsy/sum.png", depth_sum);
  cv::imwrite("/home/zhongsy/diff.png", diff);

  cv::Mat result_1(res_src1.size(), CV_8UC1, cv::Scalar(0));
  std::cout << result_1;

  for (int i = 0; i < res_src1.rows; i++) {
    for (int j = 0; j < res_src1.cols; j++) {
      int count = 0;
      for (int k = 0; k < res_2.size(); k++) {
        if (res_2[k].at<uchar>(i, j) == 255) {
          count++;
        }
      }
      if (count > 1) {
        result_1.at<uchar>(i, j) = 255;
      }
    }
  }
  cv::imwrite("/home/zhongsy/result_3_2.png", result_1);
}

#endif