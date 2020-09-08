// "Copyright [2020] <zhongsy>"

#include <stdlib.h>

#include <iostream>
#include <map>
#include <opencv2/core/core.hpp>
#include <vector>

/**
 * author:johnny zen
 * date:2017-09-20 11:19
 * function:Calculate Ternary system of equations
 * notice:时间仓促，仅仅实现功能，方便使用，代码质量不可参考！！！
 */

using namespace std;

template <class T>
void input(T matrix[4][5]) {
  cout << "please input matrix element's data" << endl;
  for (int i = 1; i < 4; i++) {
    for (int j = 1; j < 5; j++) {
      cin >> matrix[i][j];
    }
  }
  cout << "input ok";
}

template <class T>
void calc(T matrix[4][5]) {
  T base_D = matrix[1][1] * matrix[2][2] * matrix[3][3] +
             matrix[2][1] * matrix[3][2] * matrix[1][3] +
             matrix[3][1] * matrix[1][2] * matrix[2][3];  //计算行列式
  base_D = base_D - (matrix[1][3] * matrix[2][2] * matrix[3][1] +
                     matrix[1][1] * matrix[2][3] * matrix[3][2] +
                     matrix[1][2] * matrix[2][1] * matrix[3][3]);

  if (base_D != 0) {
    T x_D = matrix[1][4] * matrix[2][2] * matrix[3][3] +
            matrix[2][4] * matrix[3][2] * matrix[1][3] +
            matrix[3][4] * matrix[1][2] * matrix[2][3];
    x_D = x_D - (matrix[1][3] * matrix[2][2] * matrix[3][4] +
                 matrix[1][4] * matrix[2][3] * matrix[3][2] +
                 matrix[1][2] * matrix[2][4] * matrix[3][3]);
    T y_D = matrix[1][1] * matrix[2][4] * matrix[3][3] +
            matrix[2][1] * matrix[3][4] * matrix[1][3] +
            matrix[3][1] * matrix[1][4] * matrix[2][3];
    y_D = y_D - (matrix[1][3] * matrix[2][4] * matrix[3][1] +
                 matrix[1][1] * matrix[2][3] * matrix[3][4] +
                 matrix[1][4] * matrix[2][1] * matrix[3][3]);
    T z_D = matrix[1][1] * matrix[2][2] * matrix[3][4] +
            matrix[2][1] * matrix[3][2] * matrix[1][4] +
            matrix[3][1] * matrix[1][2] * matrix[2][4];
    z_D = z_D - (matrix[1][4] * matrix[2][2] * matrix[3][1] +
                 matrix[1][1] * matrix[2][4] * matrix[3][2] +
                 matrix[1][2] * matrix[2][1] * matrix[3][4]);

    T x = x_D / base_D;
    T y = y_D / base_D;
    T z = z_D / base_D;
    cout << "[ x:" << x << "; y:" << y << "; z:" << z << " ]" << endl;
  } else {
    cout << "【无解】";
    //        return DBL_MIN;
  }
}
//计算原理：行列式
// int main() {
//   double matrix[4][5];  //三元一次方程组

// //   input<double>(matrix);
//   calc<double>(matrix);
//   return 0;
// }
/*
demo

2x-y+z=10;
3x+2y-z=16;
x+6y-z=28;

2 -1 1 10
3 2 -1 16
1 6 -1 28

output:input ok[ x:4.18182; y:5.09091; z:6.72727 ]

*/
class Ransic {
 public:
  std::vector<double> get_sample() {
    double min;
    cv::Point min_dis;
    cv::Mat tmp = data_base.clone();
    while (sample_init.size() <= 2) {
      cv::minMaxLoc(tmp, &min, NULL, &min_dis, NULL);
      if (min == 0) {
        tmp.at<ushort>(min_dis) = 1000;
      } else {
        sample_init.push_back(
            cv::Vec3d(min_dis.x, min_dis.y, tmp.at<ushort>(min_dis)));
        dictory.emplace(std::pair(cv::Vec2i(min_dis.x, min_dis.y),
                                  tmp.at<ushort>(min_dis)));
      }
    }
  }

 private:
  const int numFoeEstimate = 3;  //  生成初始化模型的参数
  const int k = 1000;  // 初始化迭代次数
  std::vector<cv::Vec3b> sample_init;
  cv::Mat data_base;
  std::map<cv::Vec2i, double> dictory;
};
