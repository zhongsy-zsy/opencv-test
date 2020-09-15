/*
 "Copyright [year] <Copyright Owner>"
*/
#include "common_op.h"

#include <algorithm>

std::vector<double> calculate_threshold(cv::Mat mean_depth,
                                        const std::vector<cv::Mat>& raw_data) {
  std::vector<double> threshold;  // 用来存放阈值
  std::vector<double> max_value;
  // std::cout << mean_depth << std::endl;
  // std::cout << raw_data[0] << std::endl;
  // std::cout << raw_data[1] << std::endl;

  //   std::cout << mean_depth.rows << mean_depth.cols << std::endl;
  for (int i = 0; i < mean_depth.rows; i++) {
    int max_diff = 0;
    for (int j = 0; j < mean_depth.cols; j++) {
      /* 求出最大的偏差值 */
      for (int k = 0; k < raw_data.size(); k++) {
        //   std::cout << "mean_depth: " << mean_depth <<
        //   std::endl;
        //   std::cout << "raw_data: "
        //             << (int)mean_depth.at<ushort>(0, 2)
        //             << std::endl;
        max_diff = std::max(max_diff, (mean_depth.at<ushort>(i, j) -
                                       raw_data[k].at<ushort>(i, j)));
      }
    }
    std::cout << "max_diff" << i << ":" << max_diff << std::endl;

    max_value.push_back(max_diff);
    max_diff = 0;
  }

  /* 开始统计在最大值差值分段的情况下，占据97%的阈值是哪一个 */

  for (int i = 0; i < mean_depth.rows; i++) {
    std::vector<double> ratio(10, 0);  // 保存占比
    double counts = 0;
    // std::cout << "raw_data_size" << raw_data.size() << std::endl;
    for (int j = 0; j < mean_depth.cols; j++) {
      for (int k = 0; k < raw_data.size(); k++) {
        // std::cout << mean_depth.at<ushort>(i, j) - raw_data[k].at<ushort>(i,
        // j)
        //           << std::endl;
        for (int h = 1; h <= 10; h++) {
          if (h == 1) {
            if (mean_depth.at<ushort>(i, j) - raw_data[k].at<ushort>(i, j) >=
                    0 &&
                mean_depth.at<ushort>(i, j) - raw_data[k].at<ushort>(i, j) <
                    max_value[i] / 10.0) {
              ratio[0]++;
              counts++;
              break;
            }
          } else if (h == 10) {
            if (mean_depth.at<ushort>(i, j) - raw_data[k].at<ushort>(i, j) >=
                    (h - 1) * max_value[i] / 10.0 &&
                mean_depth.at<ushort>(i, j) - raw_data[k].at<ushort>(i, j) <=
                    h * max_value[i] / 10) {
              ratio[h - 1]++;
              counts++;
              break;
            }
          } else {
            if (mean_depth.at<ushort>(i, j) - raw_data[k].at<ushort>(i, j) >=
                    (h - 1) * max_value[i] / 10.0 &&
                mean_depth.at<ushort>(i, j) - raw_data[k].at<ushort>(i, j) <
                    h * max_value[i] / 10) {
              ratio[h - 1]++;
              counts++;
              break;
            }
          }
        }
      }
    }
    // std::cout << "count:" << counts << std::endl;
    // for (auto c : ratio) {
    //   std::cout << c << " ";
    // }
    // std::cout << std::endl;
    std::vector<double> res;
    double value_ratio = 0;
    for (auto value : ratio) {
      value_ratio += value;
      res.push_back(value_ratio / counts);
    }

    // for (auto c : res) {
    //   std::cout << c << " ";
    // }
    // std::cout << std::endl;

    for (int f = 0; f < res.size(); f++) {
      if (res[f] > 0.98) {
        // std::cout << "max_val" << max_value[i] << std::endl;
        // std::cout << "f: " << f << std::endl;
        threshold.push_back((f + 1) * (max_value[i]) / 10.0);
        break;
      }
    }
  }
  return threshold;
}

// int main() {
//   cv::Mat a(2, 6, CV_16UC1);
//   for (int i = 0; i < a.rows; i++) {
//     for (int j = 0; j < a.cols; j++) {
//       a.at<ushort>(i, j) = j * 6;
//     }
//   }
//   std::cout << a << std::endl;
//   //   std::cout << a.at<ushort>(0, 1) << std::endl;

//   cv::Mat b(2, 6, CV_16UC1);
//   for (int i = 0; i < b.rows; i++) {
//     for (int j = 0; j < b.cols; j++) {
//       b.at<ushort>(i, j) = j + i*3;
//     }
//   }
//   std::cout << b << std::endl;

//   cv::Mat c(2, 6, CV_16UC1);
//   for (int i = 0; i < c.rows; i++) {
//     for (int j = 0; j < c.cols; j++) {
//       c.at<ushort>(i, j) = j * 5 + i*2;
//     }
//   }
//   std::cout << c << std::endl;

//   std::vector<cv::Mat> raw_data;
//   raw_data.push_back(b);
//   raw_data.push_back(c);

//   std::vector<double> result;
//   std::cout << "jinru calculate" << std::endl;
//   result = calculate_threshold(a, raw_data);
//   for (auto a : result) {
//     std::cout << a << "  ";
//   }
//   std::cout << std::endl;
// }