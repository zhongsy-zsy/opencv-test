#include "move_compensation.h"
namespace nameof_move_compensation {

std::vector<cv::Mat> move_compensation::compensation_motion(
    std::vector<cv::Mat>& data, double delta_z) {
  // 运动补偿方法一
  // 以上一帧的深度图作为依据
  double gain = depth_intrin.fy * std::sin(ration_angle) +
                depth_intrin.ppy * std::cos(ration_angle);  // 计算增益
  // v(t1)已知 求 v(t2)
  try {
    for (int k = 1; k < data.size(); k++) {
      cv::Mat image_1 = data.at(k - 1);
      for (int i = 0; i < image_1.rows; i++) {
        for (int j = 0; j < image_1.cols; j++) {
          /* code */
          // 为了减少计算量，不移动距离大于5m的
          if (data[k].at<ushort>(i, j) == 0 ||
              data[k].at<ushort>(i, j) > 5000) {
            continue;
          }
          int dela_v = ceil(k * delta_z / image_1.at<ushort>(i, j));
          if (i - dela_v > 0) {
            ushort tmp = data[k].at<ushort>(i, j);
            data[k].at<ushort>(i, j) = 0;
            data[k].at<ushort>(i + dela_v, j) = tmp;
          } else {
            continue;
          }
        }
      }
    }
  } catch (std::out_of_range const& exc) {
    std::cout << exc.what() << std::endl;
  }
  return data;
}

// input:输入三张已将分割好的二值图
std::vector<cv::Mat> move_compensation_method1::compensation_motion(
    std::vector<cv::Mat>& data) {
  try {
    for (int i = 2; i < data[0].rows; i++) {
      for (int j = 2; j < data[0].cols; j++) {
        std::vector<int> SAD(9, 1000);  // 用来保存结果
        for (int k = 0; k < SAD.size(); k++) {
          SAD[k] = std::abs(
              data[1].at<ushort>(i + Direction_matrix[k][0],
                                 j + Direction_matrix[k][1]) -
              data[0].at<ushort>(i + nine[k][0] + Direction_matrix[k][0],
                                 j + nine[k][1] + Direction_matrix[k][1]));
        }
        // 找到vector最小值的地方
        auto smallest = std::min_element(SAD.begin(), SAD.end());
        int min_positon = std::distance(SAD.begin(), smallest);
        data[1].at<ushort>(i, j) =
            data[0].at<ushort>(i + Direction_matrix[min_positon][0],
                               j + Direction_matrix[min_positon][1]);
      }
    }

    for (int i = 2; i < data[0].rows; i++) {
      for (int j = 2; j < data[0].cols; j++) {
        std::vector<int> SAD(9, 1000);  // 用来保存结果
        for (int k = 0; k < SAD.size(); k++) {
          SAD[k] = std::abs(
              data[2].at<ushort>(i + Direction_matrix[k][0],
                                 j + Direction_matrix[k][1]) -
              data[1].at<ushort>(i + nine[k][0] + Direction_matrix[k][0],
                                 j + nine[k][1] + Direction_matrix[k][1]));
        }
        // 找到vector最小值的地方
        auto smallest = std::min_element(SAD.begin(), SAD.end());
        int min_positon = std::distance(SAD.begin(), smallest);
        data[2].at<ushort>(i, j) =
            data[1].at<ushort>(i + Direction_matrix[min_positon][0],
                               j + Direction_matrix[min_positon][1]);
      }
    }
  } catch (std::out_of_range const& exc) {
    std::cout << exc.what() << std::endl;
  }
}
}  // namespace nameof_move_compensation
