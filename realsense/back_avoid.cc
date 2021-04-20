/*
 * @Author: your name
 * @Date: 2021-04-16 11:17:29
 * @LastEditTime: 2021-04-16 16:17:25
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /move_compensation/realsense/back_avoid.cc
 */

#include <opencv2/imgproc/types_c.h>

#include <algorithm>
#include <ctime>
#include <fstream>
#include <librealsense2/rs.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "common_op.h"

template <class T>
void CompleteImage(cv::Mat image) {
  for (int i = 0; i < image.rows; i++) {
    for (int j = 0; j < image.cols; j++) {
      if (image.at<T>(i, j) == 0) {
        int k = j + 1;
        int count = 0;
        while (1) {
          if (image.at<T>(i, k % image.cols) != 0) {
            image.at<T>(i, j) = image.at<T>(i, k % image.cols);
            break;
          }
          k++;
          count++;
          if (count > 300) {
            break;
          }
        }
      }
    }
  }

  // for (int j = 0; j < image.cols; j++) {
  //   for (int i = 0; i < image.rows; i++) {
  //     if (image.at<T>(i, j) == 0) {
  //       int k = i - 1;
  //       int count = 0;
  //       while (1) {
  //         if (image.at<T>(k % image.rows, j) != 0) {
  //           image.at<T>(i, j) = image.at<T>(k % image.rows, j);
  //           break;
  //         }
  //         k--;
  //         count++;
  //         if (count > 900) {
  //           break;
  //         }
  //       }
  //     }
  //   }
  // }
}

int main() {
  //获取设备列表
  rs2::context ctx;
  auto dev_list = ctx.query_devices();
  if (dev_list.size() == 0) {
    // ROS_ERROR("D435 not detected.");
    return -1;
  }
  auto dev = dev_list.front();
  rs2::pipeline pipe(ctx);
  //启动滤波器
  // rs2::decimation_filter dec_filter;
  // rs2::spatial_filter spat_filter(0.4f,4.0f,2.0f,0);
  rs2::threshold_filter thd_filter(0.10f, 2.5f);
  //   thd_filter.set_option(RS2_OPTION_MAX_DISTANCE, 15.0);
  rs2::hole_filling_filter hole_filter(2);
  rs2::colorizer colorizered;
  // dec_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE,3);
  // spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA,0.4f);
  // spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA,4.0f);

  rs2::config cfg;
  //   cfg.enable_stream(RS2_STREAM_COLOR, Width, Height, RS2_FORMAT_RGB8, fps);
  cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
  //   cfg.enable_stream(RS2_STREAM_INFRARED, 1, Width, Height, RS2_FORMAT_Y8,
  //   fps); cfg.enable_stream(RS2_STREAM_INFRARED, 2, Width, Height,
  //   RS2_FORMAT_Y8, fps);
  pipe.start(cfg);
  std::cout << "device start" << std::endl;

  // 设置不规则roi区域
  cv::Mat roi = cv::Mat::zeros(cv::Size(640, 480), CV_8UC1);
  std::vector<std::vector<cv::Point>> contour;
  std::vector<cv::Point> pts;
  pts.push_back(cv::Point(210, 180));
  pts.push_back(cv::Point(406, 180));
  pts.push_back(cv::Point(520, 240));
  pts.push_back(cv::Point(120, 240));
  // pts.push_back(cv::Point(450, 280));
  contour.push_back(pts);

  // drawContours(roi, contour, 0, cv::Scalar::all(255), -1);
  cv::fillPoly(roi, contour, cv::Scalar::all(255));

  while (1) {
    cv::imshow("roi", roi);
    cv::waitKey(1);
    rs2::frameset frame = pipe.wait_for_frames();
    rs2::frame deal_frame = frame.get_depth_frame();
    rs2::frame show_frame = frame.get_depth_frame();
    rs2::frame raw_frame = frame.get_depth_frame();

    deal_frame = deal_frame.apply_filter(thd_filter);
    // deal_frame = deal_frame.apply_filter(hole_filter);
    // std::cout << "1" << std::endl;
    show_frame = colorizered.process(deal_frame);

    cv::Mat depth(cv::Size(640, 480), CV_16UC1, (void *)deal_frame.get_data(),
                  cv::Mat::AUTO_STEP);
    cv::Mat raw_depth(cv::Size(640, 480), CV_16UC1,
                      (void *)raw_frame.get_data(), cv::Mat::AUTO_STEP);
    cv::Mat colorized_mat(cv::Size(640, 480), CV_8UC3,
                          (void *)show_frame.get_data(), cv::Mat::AUTO_STEP);
    cv::imshow("coloried", colorized_mat);

    cv::Mat result = cv::Mat::zeros(cv::Size(640, 480), CV_8UC1);
    // 计算每一行像素的差值
    CompleteImage<ushort>(depth);
    // CompleteImage<ushort>(raw_depth);

    // std::cout << depth << std::endl;
    cv::Mat edge, edge_out;
    depth.convertTo(edge, CV_16SC1);
    cv::Mat dy;
    // cv::Sobel()
    cv::Sobel(edge, edge_out, CV_16SC1, 1, 0, 0, 3);
    cv::imshow("edge", edge_out);

    cv::Mat calculate = depth.clone();
    cv::Mat tmp = cv::Mat::zeros(depth.size(), CV_8UC1);
// 每隔20
    for (int i = 0; i < calculate.rows - 20; i++) {
      for (int j = 0; j < calculate.cols; j++) {
        if (calculate.at<ushort>(i, j) == 0 ||
            calculate.at<ushort>(i + 19, j) == 0) {
          calculate.at<ushort>(i, j) = 10000;
        } else {
          calculate.at<ushort>(i, j) = static_cast<ushort>(
              std::fabs((float)calculate.at<ushort>(i, j) -
                        (float)calculate.at<ushort>(i + 19, j)));
        }

        if (calculate.at<ushort>(i, j) < 10) {
          tmp.at<uchar>(i, j) = 255;
        }
      }
    }
    cv::imshow("binary", tmp);
    cv::waitKey(1);

    cv::Mat low_8 = cv::Mat::zeros(depth.size(), CV_8UC1);
    for (int i = 0; i < calculate.rows; i++) {
      for (int j = 0; j < calculate.cols; j++) {
        low_8.at<uchar>(i, j) = (depth.at<ushort>(i, j) >> 4) & 0x00ff;
      }
    }

    // cv::Mat low_8_show(depth.size(), CV_8UC1);

    // cv::normalize(low_8, low_8_show, 0, 255, cv::NORM_MINMAX);
    cv::imshow("low_8", low_8);
    // std::cout << low_8 << std::endl;
    cv::imshow("low_8", low_8);
    // cv::imwrite("/home/zhongsy/1.jpg", low_8);
    CompleteImage<ushort>(raw_depth);

    cv::Mat baohe(raw_depth.size(), CV_8UC1, cv::Scalar::all(255));
    int count = 0;
    for (int i = 0; i < calculate.rows; i++) {
      if (i > 250) {
        continue;
      }
      for (int j = 0; j < calculate.cols; j++) {
        ushort value = raw_depth.at<ushort>(i, j);
        if (value == 0) {
          count++;
        }
        if (value > 2000 || value == 0) {
          baohe.at<uchar>(i, j) = 255;
          continue;
        }
        // baohe.at<uchar>(i, j) = value / 3000.0 * 255;
        baohe.at<uchar>(i, j) = 0;
      }
    }
    std::cout << "zero counts: " << count << std::endl;
    cv::imshow("baohe", baohe);
    // cv::imwrite("/home/zhongsy/baohe.jpg", baohe);

    cv::Mat roi_result, roi_data;
    baohe.copyTo(roi_result, roi);
    cv::imshow("roi_result", roi_result);
    raw_depth.copyTo(roi_data, roi);

    // 开始计算距离
    cv::Mat high_8byte(raw_depth.size(), CV_8UC1);
    cv::Mat low_8byte(raw_depth.size(), CV_8UC1);

    for (int i = 0; i < raw_depth.rows; i++) {
      for (int j = 0; j < raw_depth.cols; j++) {
        //   left = (number >> 8) & 0XFF;  //先取高八位
        //   right = number & 0XFF;        //再取第八位
        high_8byte.at<uchar>(i, j) =
            static_cast<uchar>((raw_depth.at<ushort>(i, j) >> 6) & 0x3FF);
        low_8byte.at<uchar>(i, j) =
            static_cast<uchar>(raw_depth.at<ushort>(i, j) & 0x3F);
      }
    }

    cv::Mat depth_mat1 = high_8byte;

    int channels = 0;
    cv::Mat dstHist;
    int dims = 1;
    float hranges[] = {1, 1024};
    const float *ranges[] = {hranges};  // 这里需要为const类型
    int size = 1024;
    //计算图像的直方图
    /*
    images：输入图像
    nimage：输入图像的个数
    channels：需要统计直方图的第几通道
    mask：掩膜，计算掩膜内的直方图
    hist：输出直方图
    dims：需要统计直方图通道的个数
    histSize：直方图分成多少个区间
    ranges：统计像素值的区间
    uniform：是否对直方图进行归一化处理
    accumulate：在多个图像时，是否累计计算像素值得个数
    */
    cv::calcHist(&depth_mat1, 1, &channels, roi, dstHist, dims, &size, ranges);

    cv::Mat dstImage(size, size, CV_8U, cv::Scalar(0));
    //获取最大值和最小值
    double minValue = 0;
    double maxValue = 0;

    /*
    查找最大元素和最小元素及其位置
    InputArray：输入单通道阵列
    double* minVal：指向返回的最小值的指针;如果不需要，则使用NULL。
    double* maxVal = 0：指向返回的最大值的指针;如果不需要，则使用NULL。
    Point* minLoc =
    0：指向返回的最小位置的指针(二维情况下);如果不需要，则使用NULL。 Point*
    maxLoc = 0：指向返回的最大位置的指针(二维情况下);如果不需要，则使用NULL。
    InputArray 	mask = noArray()：用于选择子数组的可选掩码。
    */
    cv::minMaxLoc(dstHist, &minValue, &maxValue, 0, 0);

    //绘制出直方图
    // saturate_cast函数的作用即是：当运算完之后，结果为负，则转为0，结果超出255，则为255。
    int hpt = cv::saturate_cast<int>(0.9 * size);
    int temp = INT_MAX;
    int distance = 0;
    static bool flag = false;
    static float sumpix;
    if (flag) {
      for (int i = 0; i < 1024; i++) {
        sumpix += dstHist.at<float>(i);
      }
      flag = false;
    }

    for (int i = 0; i < 1024; i++) {
      // 当前距离所对应的像素个数
      float binValue = dstHist.at<float>(i);  //   注意hist中是float类型
      // 当前距离所对应像素个数占总像素个数的百分比
      float percent = binValue / sumpix;
      // 拉伸到0-max
      int realValue = cv::saturate_cast<int>(binValue * hpt / maxValue);
      // 当前距离所对应像素个数占总像素个数的百分比大于3%
      if (percent > 0.03) {
        distance = i + 2;
        break;
      }
    }
    std::cout << (double)(distance) * (double)0.064 << " m" << std::endl;
    // 结束计算距离

    for (int i = 0; i < depth.rows; i++) {
      if (i > 260) {
        break;
      }
      for (int j = 0; j < depth.cols; j++) {
        ushort value = depth.at<ushort>(i, j);
        if (value == 0) {
          continue;
        }
        if (value < 2000) {
          result.at<uchar>(i, j) = 255;
        }
      }
    }

    cv::imshow("result", result);
    cv::waitKey(25);
  }
}
