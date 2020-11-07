#include "move_compensation.h"
namespace nameof_move_compensation {
//这里输入的也是分割好的阈值图
std::vector<cv::Mat> move_compensation::compensation_motion(
    std::vector<cv::Mat>& data, double delta_z) {
  // 运动补偿方法一
  // 以上一帧的深度图作为依据，因为时间间隔十分小所以那么就认定是两段移动的距离是一样的
  double gain = depth_intrin_.fy * std::sin(ration_angle_) +
                depth_intrin_.ppy * std::cos(ration_angle_);  // 计算增益
  // v(t1)已知 求 v(t2)
  // 移动后的图是一副新的图
  try {
    cv::Mat image_raw = depth_data_;
    for (int k = 1; k < data.size(); k++) {
      cv::Mat new_depth(image_raw.rows, image_raw.cols, CV_8UC1,
                        cv::Scalar(255));
      for (int i = 0; i < image_raw.rows; i++) {
        for (int j = 0; j < image_raw.cols; j++) {
          /* code */
          // 为了减少计算量，不移动距离大于5m的
          if (image_raw.at<ushort>(i, j) == 0 ||
              image_raw.at<ushort>(i, j) > 5000) {
            continue;
          }
          int delta_v = ceil(gain * delta_z * k / image_raw.at<ushort>(i, j) /
                             1000);  //注意单位
          std::cout << "delta_v is :" << delta_v << std::endl;
          if (i - delta_v > 0) {
            // 开始移动像素
            new_depth.at<ushort>(i - delta_v, j) = data[k].at<ushort>(i, j);
          } else {
            continue;
          }
        }
      }
      new_depth.copyTo(data[k]);
    }
  } catch (std::out_of_range const& exc) {
    std::cout << exc.what() << std::endl;
  }
  return data;
}

// input:输入三张已将分割好的二值图
void move_compensation_method1::compensation_motion(
    std::vector<cv::Mat>& data) {
  try {
    for (int i = 7; i < data[0].rows; i++) {
      for (int j = 7; j < data[0].cols; j++) {
        std::vector<double> SAD(15, 1000);  // 用来保存结果
        // 开始计算每一个点的SAD
        for (int k = 0; k < SAD.size(); k++) {
          double tmp_value = 0;
          for (int h = 0; h < 3; h++) {
            tmp_value += std::fabs(
                static_cast<double>(data[1].at<ushort>(
                    i + Direction_matrix[h][0], j + Direction_matrix[h][1])) -
                static_cast<double>(data[0].at<ushort>(
                    i + nine[k][0] + Direction_matrix[h][0],
                    j + nine[k][1] + Direction_matrix[h][1])));
          }
          SAD[k] = tmp_value;
        }
        // for (auto value : SAD) {
        //   std::cout << " " << value << " ";
        // }
        std::cout << std::endl;
        // 找到vector最小值的地方
        auto smallest = std::min_element(SAD.begin(), SAD.end());
        int min_positon = std::distance(SAD.begin(), smallest);
        std::cout << " " << min_positon << " ";
        data[1].at<ushort>(i, j) = data[0].at<ushort>(i + nine[min_positon][0],
                                                      j + nine[min_positon][1]);
      }
    }
    std::cout << "sucess" << std::endl;
    for (int i = 7; i < data[1].rows; i++) {
      for (int j = 7; j < data[1].cols; j++) {
        std::vector<double> SAD(15, 1000);  // 用来保存结果
        // 开始计算每一个点的SAD
        for (int k = 0; k < SAD.size(); k++) {
          double tmp_value = 0;
          for (int h = 0; h < 3; h++) {
            tmp_value += std::fabs(
                static_cast<double>(data[2].at<ushort>(
                    i + Direction_matrix[h][0], j + Direction_matrix[h][1])) -
                static_cast<double>(data[1].at<ushort>(
                    i + nine[k][0] + Direction_matrix[h][0],
                    j + nine[k][1] + Direction_matrix[h][1])));
          }
          SAD[k] = tmp_value;
        }
        // 找到vector最小值的地方
        auto smallest = std::min_element(SAD.begin(), SAD.end());
        int min_positon = std::distance(SAD.begin(), smallest);
        data[2].at<ushort>(i, j) = data[1].at<ushort>(i + nine[min_positon][0],
                                                      j + nine[min_positon][1]);
      }
    }

  } catch (std::out_of_range const& exc) {
    std::cout << exc.what() << std::endl;
    std::cout << "ERROR" << std::endl;
  }
}
}  // namespace nameof_move_compensation
