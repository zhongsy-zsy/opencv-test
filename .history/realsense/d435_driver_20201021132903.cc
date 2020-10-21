/*
 "Copyright [year] <Copyright Owner>"
*/
#include "d435_driver.h"

#include <opencv2/imgproc/types_c.h>

#include <algorithm>
#include <ctime>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>

#include "common_op.h"

void D435::Init() {
  // rs2::context ctx;
  //获取设备列表
  dev_list = ctx.query_devices();
  if (dev_list.size() == 0) {
    // ROS_ERROR("D435 not detected.");
    return;
  }
  dev = dev_list.front();
  rs2::pipeline pipe1(ctx);
  pipe = pipe1;
  //启动滤波器
  // rs2::decimation_filter dec_filter;
  // rs2::spatial_filter spat_filter(0.4f,4.0f,2.0f,0);
  // rs2::threshold_filter thd_filter(0.15f,4.0f);
  // rs2::hole_filling_filter hole_filter(0);
  // rs2::colorizer colorizered;
  // dec_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE,3);
  // spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA,0.4f);
  // spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA,4.0f);
  rs2::config cfg;
  //   cfg.enable_stream(RS2_STREAM_COLOR, Width, Height, RS2_FORMAT_RGB8, fps);
  cfg.enable_stream(RS2_STREAM_DEPTH, Width, Height, RS2_FORMAT_Z16, fps);
  //   cfg.enable_stream(RS2_STREAM_INFRARED, 1, Width, Height, RS2_FORMAT_Y8,
  //   fps); cfg.enable_stream(RS2_STREAM_INFRARED, 2, Width, Height,
  //   RS2_FORMAT_Y8, fps);
  pipe.start(cfg);
  std::cout << "device start" << std::endl;
  // AbstractDriver::SetState(DriverState::READY);
  // filters 初始化
  if (decimation_filter.supports(rs2_option::RS2_OPTION_FILTER_MAGNITUDE)) {
    rs2::option_range option_range = decimation_filter.get_option_range(
        rs2_option::RS2_OPTION_FILTER_MAGNITUDE);
    decimation_filter.set_option(
        rs2_option::RS2_OPTION_FILTER_MAGNITUDE,
        option_range.min);  // 1(min) is not downsampling
  }

  // Set Spatial Filter Option
  if (spatial_filter.supports(rs2_option::RS2_OPTION_HOLES_FILL)) {
    rs2::option_range option_range =
        spatial_filter.get_option_range(rs2_option::RS2_OPTION_HOLES_FILL);
    spatial_filter.set_option(rs2_option::RS2_OPTION_HOLES_FILL,
                              option_range.max);  // 5(max) is fill all holes
  }

  are_threshold.push_back(700);
  are_threshold.push_back(100);
  up_to_nums.push_back(0.5);
  up_to_nums.push_back(1.5);
  //   ration_angle = 30.0 / 180 * pi;
  frames = pipe.wait_for_frames();
  rs2::depth_frame depth_tmp = frames.get_depth_frame();
  dprofile = depth_tmp.get_profile();
  rs2::video_stream_profile dvsprofile(dprofile);
  depth_intrin = dvsprofile.get_intrinsics();
  std::cout << RED;
  std::cout << "\ndepth intrinsics: ";
  std::cout << depth_intrin.width << "  " << depth_intrin.height << "  ";
  std::cout << depth_intrin.ppx << "  " << depth_intrin.ppy << "  ";
  std::cout << depth_intrin.fx << "  " << depth_intrin.fy << std::endl;
  std::cout << "coeffs: ";
  for (auto value : depth_intrin.coeffs) std::cout << value << "  ";
  std::cout << std::endl;
  std::cout << "distortion model: " << depth_intrin.model
            << std::endl;  ///畸变模型
  std::cout << RESET;

  run_executor_ =
      std::make_shared<std::thread>(std::bind(&D435::HandleFeedbackData, this));
}

void D435::GetData(void *data) {
  auto cdata = static_cast<cv::Mat *>(data);
  *cdata = depth_data;
}

void D435::separate_byte() {
  //   ushort a = 258;
  //   std::cout << "si" << sizeof(a) << std::endl;
  //   ushort *point = &a;
  //   void *tmp_point = point;
  //   auto aa = static_cast<char *>(tmp_point);
  //   std::cout << (int)(*aa) << std::endl;
  //   aa += 1;
  //   std::cout << (int)(*aa) << std::endl;

  //   ushort num = 65520;
  //   cv::Mat r(4, 4, CV_16UC1);
  //   for (int i = 0; i < r.rows; i++) {
  //     for (int j = 0; j < r.cols; j++) {
  //       r.at<ushort>(i, j) = num;
  //       num++;
  //     }
  //   }
  //   std::cout << r << std::endl;

  //   cv::Mat *pp = &r;
  //   void *tmp_p = pp;
  //   auto b = static_cast<int *>(tmp_p);
  //   std::cout << sizeof(r.at<ushort>(0, 0)) << std::endl;
  //   std::cout << sizeof(r) << std::endl;
  //   for (int i = 0; i < 48; i++) {
  //     std::cout << static_cast<ushort>(*b) << std::endl;
  //     b += 1;
  //   }
  /*
  进行 separate分离测试一
   */
  //   cv::Mat high_8(4, 4, CV_8UC1);
  //   cv::Mat low_8(4, 4, CV_8UC1);
  //   for (int i = 0; i < r.rows; i++) {
  //     for (int j = 0; j < r.cols; j++) {
  //       //   left = (number >> 8) & 0XFF;  //先取高八位
  //       //   right = number & 0XFF;        //再取第八位
  //       high_8.at<uchar>(i, j) =
  //           static_cast<uchar>((r.at<ushort>(i, j) >> 8) & 0xFF);
  //       low_8.at<uchar>(i, j) =
  //           static_cast<uchar>(r.at<ushort>(i, j) & 0xFF);
  //     }
  //   }
  //   std::cout << high_8 << std::endl;
  //   std::cout << low_8 << std::endl;

  /*
  正式的代码
   */
  cv::Mat high_8byte(Height, Width, CV_8UC1);
  cv::Mat low_8byte(Height, Width, CV_8UC1);
  for (int i = 0; i < depth_data.rows; i++) {
    for (int j = 0; j < depth_data.cols; j++) {
      //   left = (number >> 8) & 0XFF;  //先取高八位
      //   right = number & 0XFF;        //再取第八位
      high_8byte.at<uchar>(i, j) =
          static_cast<uchar>((depth_data.at<ushort>(i, j) >> 8) & 0xFF) * 10;
      low_8byte.at<uchar>(i, j) =
          static_cast<uchar>(depth_data.at<ushort>(i, j) & 0xFF);
    }
  }
  std::cout << high_8byte << std::endl;
  cv::imshow("low_8byte", low_8byte);
  cv::waitKey(1);
  cv::imshow("high_8byte", high_8byte);
  cv::waitKey(1);
}

void D435::get_depth() {
  clock_t start, stop;
  double duration;
  start = clock();
  frames = pipe.wait_for_frames();
  rs2::depth_frame depth_frame = frames.get_depth_frame();
  rs2::frame filtered_frame = depth_frame;
  // 应用抽取滤波器（下采样）

  filtered_frame = decimation_filter.process(filtered_frame);
  //   stop = clock();
  //   duration = static_cast<double>(stop - start) / CLOCKS_PER_SEC;
  std::cout << "consume time for xiacaiyang: " << duration << "second"
            << std::endl;
  //   // 从深度帧转换视差帧
  //   start = clock();
  rs2::disparity_transform disparity_transform(true);
  filtered_frame = disparity_transform.process(filtered_frame);
  //   stop = clock();
  std::cout << "consume time for depth_to_diff: " << duration << "second"
            << std::endl;
  //   start = clock();
  // 应用空间滤镜（保留边缘的平滑，补孔）
  filtered_frame = spatial_filter.process(filtered_frame);
  //   stop = clock();
  //   duration = static_cast<double>(stop - start) / CLOCKS_PER_SEC;
  std::cout << "consume time for spatial_filter: " << duration << "second"
            << std::endl;
  // 应用时间过滤器（使用多个先前的帧进行平滑处理）
  //   start = clock();
  filtered_frame = temporal_filter.process(filtered_frame);
  //   stop = clock();
  //   duration = static_cast<double>(stop - start) / CLOCKS_PER_SEC;
  std::cout << "consume time for time_filter: " << duration << "second"
            << std::endl;
  // 从视差帧变换深度帧
  //   start = clock();
  rs2::disparity_transform depth_transform(false);
  filtered_frame = depth_transform.process(filtered_frame);
  //   stop = clock();
  //   duration = static_cast<double>(stop - start) / CLOCKS_PER_SEC;
  std::cout << "consume time for diff_to_depth: " << duration << "second"
            << std::endl;
  cv::Mat depth(cv::Size(Width, Height), CV_16UC1,
                (void *)filtered_frame.get_data(), cv::Mat::AUTO_STEP);

  //   cv::Mat display = depth.clone();
  //   display.convertTo(display, CV_8UC1, 255.0 / 7000.0);
  //   cv::imshow("depth_display", display);
  //   cv::waitKey(1);
  depth.copyTo(depth_data);
  stop = clock();
  duration = static_cast<double>(stop - start) / CLOCKS_PER_SEC;
  //   std::cout << "consume time for Getdepth(): " << duration << "second"
  // << std::endl;
}

void D435::calibration_angle() {
  if (judge_file("angle_data.csv")) {
    std::ifstream inFile("angle_data.csv", std::ios::in);
    std::string value;
    std::vector<double> data;
    getline(inFile, value);  // 读取整行进value中 读取第一行
    std::stringstream ss(value);
    std::string str;
    while (getline(ss, str, ',')) {  // 以逗号为分隔读取string
      ration_angle = static_cast<float>(atof(str.c_str()));  // string转为double
      ration_angle = ration_angle / 180.0 * pi;
    }
    std::cout << RED << "ration_angle: " << ration_angle << RESET << std::endl;
  } else {
    cv::waitKey(10);
    cv::Vec3f tem;
    cv::Vec3f result;
    bool flag = true;
    // LOG_INFO("start calibration angle");
    while (flag) {
      while (1) {
        get_depth();
        tem[0] = Width / 2;
        tem[1] = Height / 2;
        tem[2] = static_cast<float>(
            depth_data.at<ushort>(cv::Point(Width / 2, Height / 2)));
        result[0] = 400;
        result[1] = 300;
        result[2] =
            static_cast<float>(depth_data.at<ushort>(cv::Point(400, 300)));
        if (result[2] != 0 && result[2] < 4000 && tem[2] != 0 &&
            tem[2] < 4000) {
          break;
        }
      }

      for (float i = 0.0; i < 70.0; i += 0.1) {
        ration_angle = (i / 180.0) * pi;
        cv::Vec3f check1 = pixel_to_world(tem);
        cv::Vec3f check2 = pixel_to_world(result);
        if (std::fabs(check1[2] - check2[2]) < 1) {
          // LOG_INFO("finish calibration angle");
          calibration_data.open("angle_data.csv",
                                std::ios::out | std::ios::trunc);
          calibration_data << i << "," << std::endl;
          calibration_data.close();
          flag = false;
          break;
        }
      }
    }
  }
}

std::vector<cv::Mat> D435::get_depth2calculate(cv::Rect ROI) {
  clock_t start, stop;
  double duration;
  start = clock();
  cv::Mat raw_data;
  std::vector<cv::Mat> res;
  for (int i = 0; i < 3; i++) {
    frames = pipe.wait_for_frames();
    rs2::depth_frame depth_frame = frames.get_depth_frame();
    rs2::frame filtered_frame = depth_frame;
    filtered_frame = colorizered.process(filtered_frame);
    // // 应用抽取滤波器（下采样）
    // filtered_frame = decimation_filter.process(filtered_frame);

    // // 从深度帧转换视差帧
    // rs2::disparity_transform disparity_transform(true);
    // //   filtered_frame = disparity_transform.process(filtered_frame);
    // // 应用空间滤镜（保留边缘的平滑，补孔）
    // //   filtered_frame = spatial_filter.process(filtered_frame);

    // // 应用时间过滤器（使用多个先前的帧进行平滑处理）
    // filtered_frame = temporal_filter.process(filtered_frame);

    // // 从视差帧变换深度帧
    // rs2::disparity_transform depth_transform(false);
    // filtered_frame = depth_transform.process(filtered_frame);
    cv::Mat depth(cv::Size(Width, Height), CV_16UC1,
                  (void *)depth_frame.get_data(), cv::Mat::AUTO_STEP);
    cv::Mat display(cv::Size(Width, Height), CV_8UC3,
                    (void *)filtered_frame.get_data(), cv::Mat::AUTO_STEP);
    depth.copyTo(depth_data);
    cv::Mat color_diaplay = cv::Mat::zeros(cv::Size(Width, Height), CV_8UC3);
    cv::namedWindow("diaplay_color", CV_WINDOW_AUTOSIZE);
    cv::circle(color_diaplay, cv::Point(400, 300), 3, cv::Scalar(0, 0, 255), 3);
    cv::circle(display, cv::Point(400, 300), 3, cv::Scalar(0, 0, 255), 3);
    cv::circle(display, cv::Point(Width / 2, Height / 2), 3,
               cv::Scalar(0, 0, 255), 3);
    cv::Vec3f test;
    test[2] = depth.at<ushort>(cv::Point(400, 300));
    test[0] = 400;
    test[1] = 300;
    test = pixel_to_world(test);
    cv::Vec3f test_raw;
    test_raw[2] = depth.at<ushort>(cv::Point(Width / 2, Height / 2));
    test_raw[0] = Width / 2;
    test_raw[1] = Height / 2;
    test_raw = pixel_to_world(test_raw);
    cv::putText(display,
                "X:" + std::to_string(test[0]) + " Y:" +
                    std::to_string(test[1]) + " Z:" + std::to_string(test[2]),
                cv::Point(100, 200), 1, 1, cv::Scalar(0, 0, 255));
    cv::putText(display,
                "X_raw:" + std::to_string(test_raw[0]) +
                    " Y_raw:" + std::to_string(test_raw[1]) +
                    " Z_raw:" + std::to_string(test_raw[2]),
                cv::Point(100, 250), 1, 1, cv::Scalar(0, 0, 255));
    cv::putText(
        display,
        "X_raw_r:" + std::to_string(test_raw[0]) +
            " Y_raw_r:" + std::to_string(test_raw[1]) + " Z_raw_r:" +
            std::to_string(depth.at<ushort>(cv::Point(Width / 2, Height / 2))),
        cv::Point(100, 280), 1, 1, cv::Scalar(0, 0, 255));
    cv::putText(display,
                "X_raw_l:" + std::to_string(test_raw[0]) +
                    " Y_raw_l:" + std::to_string(test_raw[1]) + " Z_raw_l:" +
                    std::to_string(depth.at<ushort>(cv::Point(400, 300))),
                cv::Point(100, 300), 1, 1, cv::Scalar(0, 0, 255));
    cv::imshow("diaplay_color", color_diaplay);
    cv::waitKey(1);
    cv::imshow("diaplay", display);
    cv::waitKey(1);
    cv::imshow("roi_depth", depth(ROI));
    res.emplace_back(depth(ROI).clone());
  }

  //   cv::Mat display = depth.clone();
  //   display.convertTo(display, CV_8UC1, 255.0 / 7000.0);
  //   cv::imshow("depth_display", display);
  //   cv::waitKey(1);
  //   depth.copyTo(depth_data);
  //   depth.copyTo(depth_data);
  stop = clock();
  duration = static_cast<double>(stop - start) / CLOCKS_PER_SEC;
  //   std::cout << "consume time for Getdepth2calculate(): " << duration <<
  //   "second"
  // << std::endl;
  return res;
}

cv::Mat D435::show_depth(int row_start, int row_end, int col_start,
                         int col_end) {
  cv::Mat display_depth = depth_data.clone();
  for (int i = 0; i < display_depth.rows; i++) {
    for (int j = 0; j < display_depth.cols; j++) {
      if (i < row_start || i > row_end || j < col_start || j > col_end) {
        display_depth.at<ushort>(i, j) = 0;
      }
    }
  }

  cv::Mat res = display_depth.clone();
  display_depth.convertTo(display_depth, CV_8UC1, 255.0 / 10000);
  cv::imshow("diaplay_depth", display_depth);
  cv::waitKey(1);
  return res;
}

void D435::mask_depth(cv::Mat &image, int throld) {
  //这里也可以利用域值滤波
  int nr = image.rows;  // number of rows
  int nc = image.cols;  // number of columns
  for (int i = 0; i < nr; i++) {
    for (int j = 0; j < nc; j++) {
      // if (image.at<ushort>(i, j)>throld||image.at<ushort>(i,j)<12)

      if (image.at<ushort>(i, j) > throld || i > 400 || i < 20 || j < 200 ||
          j > 440 || image.at<ushort>(i, j) < 50) {
        image.at<ushort>(i, j) = 0;
      }
    }
  }

  //滤除地面，也可以考虑把边界滤除去
  // int deltap=20;
  // int heig=3000;
  // float z=1;
  // for(int i=0;i<nr;i++)
  // {
  //     for(int j=0;j<nc;j++)
  //     {
  //         if(heig-image.at<ushort>(i,j)*z<deltap||heig-image.at<ushort>(i,j)*z<0)
  //         {
  //             image.at<ushort>(i,j)=0;
  //         }
  //     }
  // }
}

void D435::find_obstacle(std::vector<cv::Mat> depth, int thresh, int max_thresh,
                         std::vector<int> areas) {
  //   cv::Mat dep;
  //   dep = depth.clone();
  //   cv::Mat threshold_output;
  std::vector<std::vector<cv::Point>> contours;
  cv::Mat drawing = cv::Mat::zeros(Height, Width, CV_8UC3);  // 用于画图显示

  std::vector<cv::Vec4i> hierarchy;
  cv::RNG rng(12345);
  // 阈值分割
  for (int i = 0; i < depth.size(); i++) {
    cv::threshold(depth[i], depth[i], thresh, 255, cv::THRESH_BINARY_INV);
  }
  //   cv::imshow("erzhihua", threshold_output);
  //   cv::waitKey(1);
  // mask_depth(src, threshold_output);
  /// 寻找轮廓

  /// 绘出轮廓及其凸包
  for (int h = 0; h < depth.size(); h++) {
    findContours(depth[h], contours, hierarchy,
                 CV_RETR_TREE,  // 找到第i副图像的轮廓
                 CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    /// 对每个轮廓计算其凸包
    std::vector<std::vector<cv::Point>> hull(contours.size());
    for (uint i = 0; i < contours.size(); i++) {
      convexHull(cv::Mat(contours[i]), hull[i], false);
    }

    for (int i = 0; i < contours.size(); i++) {
      if (contourArea(contours[i]) < areas[h] ||
          contourArea(contours[i]) >
              306000)  // 面积大于或小于area的凸包，可忽略
        continue;
      result.push_back(hull[i]);
      cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255),
                                    rng.uniform(0, 255));
      drawContours(drawing, contours, i, color, 1, 8, std::vector<cv::Vec4i>(),
                   0, cv::Point());
      drawContours(drawing, hull, i, color, 1, 8, std::vector<cv::Vec4i>(), 0,
                   cv::Point());
    }
    contours.clear();
  }
  cv::imshow("contours", drawing);
}

void D435::quit_black_block(cv::Mat &image) {
  int nr = image.rows;
  int nc = image.cols;

  for (int i = 0; i < nr; i++) {
    for (int j = 0; j < nc; j++) {
      if (image.at<uchar>(i, j) == 0) {
        image.at<uchar>(i, j) = 255;
      }
    }
  }
}

void D435::calculate_mindistance(float threshold_x, float threshold_y) {
  std::vector<cv::Rect> ve_rect;
  cv::Mat drawing = cv::Mat::zeros(cv::Size(Width, Height), CV_8UC3);

  if (result.empty()) {
    // 设置避障等级为0
    // min_distance = 3000;
    // std::cout << min_distance << std::endl;
    std::cout << "avoid Free" << std::endl;
  } else {
    for (int i = 0; i < result.size(); i++) {
      cv::Rect rect;
      rect = cv::boundingRect(result[i]);
      //   if (rect.area() > 304000) {continue;}
      ve_rect.emplace_back(rect);
    }

    // 去除重复框
    std::sort(ve_rect.begin(), ve_rect.end(), cmp);
    for (auto i = ve_rect.begin(); i < ve_rect.end(); i++) {
      for (auto j = i + 1; j < ve_rect.end(); j++) {
        if (isInside((*i), (*j))) {
          ve_rect.erase(j);
          j--;
        }
      }
    }
    /* 进行rect减半操作 */
    // for (auto i = ve_rect.begin(); i < ve_rect.end(); i++) {
    //   (*i) = (*i) + cv::Point((*i).width / 2, (*i).height / 2);
    //   (*i) = (*i) + cv::Size(-(*i).width / 2, -(*i).height / 2);
    // }
    std::vector<double> res;
    if (!ve_rect.empty()) {
      //   cv::Rect ROI(160, 0, 520, 480);
      //   cv::Mat ROI_depth = depth_data(ROI);
      for (int i = 0; i < ve_rect.size(); i++) {
        cv::Mat imageROI = depth_data(ve_rect[i]);
        int x_delta = ve_rect[i].x;
        int y_delta = ve_rect[i].y;
        //     std::cout << "iamge :" << ROI_depth.rows << " " << ROI_depth.cols
        //     << " "
        //               << ROI.x << " " << ROI.y << std::endl;
        //     // cv::imshow("ROI", imageROI);
        //     // cv::waitKey(1);
        //     // 过滤零点
        //     cv::Vec3f test;
        //     test[0] = 120 + ROI.x;
        //     test[1] = 240 + ROI.y;
        //     test[2] = ROI_depth.at<ushort>(cv::Point(120, 240));
        //     test = pixel_to_world(test);
        //     std::cout << "test[0]: " << test[0] << "test1: " << test[1] <<
        //     "test[2]"
        //               << test[2] << std::endl;

        cv::rectangle(drawing, ve_rect[i], cv::Scalar(0, 0, 255));
        for (int i = 0; i < imageROI.rows; i++) {
          for (int j = 0; j < imageROI.cols; j++) {
            float Z = static_cast<float>(imageROI.at<ushort>(i, j));
            float X =
                (static_cast<float>((j + x_delta) * Z) - depth_intrin.ppx * Z) /
                depth_intrin
                    .fx;  // 这是考虑的没有畸变的情况，如果有畸变那么就是另外的情况了
            float Y =
                (static_cast<float>((i + y_delta) * Z) - depth_intrin.ppy * Z) /
                depth_intrin.fy;
            // 进行相机坐标到车体坐标的转换
            Z = -std::sin(ration_angle) * Y + std::cos(ration_angle) * Z;
            Y = std::cos(ration_angle) * Y + std::sin(ration_angle) * Z;
            if (depth_data.at<ushort>(i, j) == 0 ||
                std::fabs(X) > threshold_x) {  // 这边可以再加一个Y的阈值
              depth_data.at<ushort>(i, j) = 4000;
            } else {
              depth_data.at<ushort>(i, j) = static_cast<ushort>(Z);
            }
          }
        }
        // std::cout << imageROI << std::endl;
        cv::imshow("rectangle", drawing);
        cv::waitKey(1);
        double min_dis;
        cv::Point min_point;
        std::vector<int> tmp;

        // for(int i=0;i<3;i++)
        //   std::cout << "imageROI" << imageROI << std::endl;
        while (tmp.size() <= 3) {
          cv::minMaxLoc(depth_data, &min_dis, NULL, &min_point, NULL);
          //  std::cout<<"min_dis_roi"<<min_dis<<std::endl;
          // 优化获取的是前面3个最小点的平均值
          if (tmp.empty()) {
            tmp.emplace_back(min_dis);
            depth_data.at<ushort>(min_point) = 4000;
          } else {
            if (std::fabs(min_dis - tmp.back()) < 40) {
              tmp.emplace_back(min_dis);
              depth_data.at<ushort>(min_point) = 4000;
            } else {
              // 是噪声点
              tmp.pop_back();
            }
          }
          // std::cout<<tmp.size()<<std::endl;
        }
        min_dis = 0;
        for (auto average : tmp) {
          min_dis += average;
        }
        res.emplace_back(min_dis / 4.0);
      }

      if (!res.empty()) {
        min_distance = *std::min_element(res.begin(), res.end());
        std::cout << "min_diatance" << min_distance << std::endl;
        if (min_distance < 1000) {
          std::cout << "stop avoid" << std::endl;
        } else if (min_distance < 1500) {
          std::cout << "avoid level 1" << std::endl;
        } else if (min_distance < 4000) {
          std::cout << "avoid level 2" << std::endl;
        } else {
          std::cout << "aviod FREE" << std::endl;
        }
      } else {
        std::cout << "avoid FREE" << std::endl;
      }
      std::cout << std::endl;
    } else {
      std::cout << "avoid FREE" << std::endl;
    }
  }
  result.clear();
}

cv::Vec3f D435::pixel_to_world(cv::Vec3f point) {
  cv::Vec3f result;
  result[0] =
      (point[0] * point[2] - depth_intrin.ppx * point[2]) / depth_intrin.fx;
  result[1] =
      (point[1] * point[2] - depth_intrin.ppy * point[2]) / depth_intrin.fy;
  result[2] = point[2];

  result[2] =
      -std::sin(ration_angle) * result[1] + std::cos(ration_angle) * result[2];
  result[1] =
      std::cos(ration_angle) * result[1] + std::sin(ration_angle) * result[2];

  return result;
}

void D435::caculate_thread4() {
  int res = 0;
  int count = 0;
  double max = 0;
  int error_thread = 2000;
  cv::Mat trd1(depth_data, cv::Range(79, 80));
  //   std::cout << trd1.rows << std::endl;
  //   std::cout << trd1.cols << std::endl;
  //   std::cout << "thread1" << trd1 << std::endl;
  //   for (int j = 0; j < Width; j++) {
  //     if (trd1.at<ushort>(0, j) == 0) {
  //       continue;
  //     } else {
  //       res += trd1.at<ushort>(0, j);
  //       count++;
  //     }
  //   }
  //   std::cout << "dls" << std::endl;
  //   max = 0;
  cv::minMaxLoc(trd1, NULL, &max, NULL, NULL);
  if (thread1 == 0) {
    if (max > 6000) {
      thread1 = 6000;
    } else if (max != 0) {
      thread1 = max;
      if (thread1 > 10) thread1 -= 10;  // 提前量
    }
  } else if (std::fabs(thread1 - max) > error_thread && max < 6000) {
  } else if (max > 6000) {
    thread1 = 6000;
  } else if (max != 0) {
    thread1 = max;
    if (thread1 > 10) thread1 -= 10;
  }  // 提前量
  std::cout << "thread1: " << thread1 << std::endl;
  //   res = 0;
  //   count = 0;
  max = 0;
  cv::Mat trd2(depth_data, cv::Range(159, 160));
  //   std::cout << "thread2" << trd2 << std::endl;
  //   for (int j = 0; j < Width; j++) {
  //     if (trd2.at<ushort>(0, j) == 0) {
  //       continue;
  //     } else {
  //       res += trd2.at<ushort>(0, j);
  //       count++;
  //     }
  //   }
  cv::minMaxLoc(trd2, NULL, &max, NULL, NULL);
  if (thread2 == 0) {
    if (max > 5500) {
      thread2 = 5500;
    } else if (max != 0) {
      thread2 = max;
      if (thread2 > 10) thread2 -= 10;  // 提前量
    }
  } else if (std::fabs(thread2 - max) > error_thread && max < 5500) {
  } else if (max > 5500) {
    thread2 = 5500;
  } else if (max != 0) {
    thread2 = max;
    if (thread2 > 10) thread2 -= 10;  // 提前量
  }
  std::cout << "thread2: " << thread2 << std::endl;
  //   res = 0;
  //   count = 0;
  max = 0;
  cv::Mat trd3(depth_data, cv::Range(239, 240));
  //   std::cout << "thread3" << trd3 << std::endl;
  //   for (int j = 0; j < Width; j++) {
  //     if (trd3.at<ushort>(0, j) == 0) {
  //       continue;
  //     } else {
  //       res += trd3.at<ushort>(0, j);
  //       count++;
  //     }
  //   }
  cv::minMaxLoc(trd3, NULL, &max, NULL, NULL);
  if (thread3 == 0) {
    if (max > 5000) {
      thread3 = 5000;
    } else if (max != 0) {
      thread3 = max;
      if (thread3 > 10) thread3 -= 10;  // 提前量
    }
  } else if (std::fabs(thread3 - max) > error_thread && max < 5000) {
  } else if (max > 5000) {
    thread3 = 5000;
  } else if (max != 0) {
    thread3 = max;
    if (thread3 > 10) thread3 -= 10;  // 提前量
  }
  std::cout << "thread3: " << thread3 << std::endl;
  //   res = 0;
  //   count = 0;
  max = 0;
  cv::Mat trd4(depth_data, cv::Range(319, 320));
  //   std::cout << "thread4" << trd4 << std::endl;
  //   for (int j = 0; j < Width; j++) {
  //     if (trd4.at<ushort>(0, j) == 0) {
  //       continue;
  //     } else {
  //       res += trd4.at<ushort>(0, j);
  //       count++;
  //     }
  //   }
  cv::minMaxLoc(trd4, NULL, &max, NULL, NULL);
  if (thread4 == 0) {
    if (max > 4500) {
      thread4 = 4500;
    } else if (max != 0) {
      thread4 = max;
      if (thread4 > 10) thread4 -= 10;  // 提前量
    }
  } else if (std::fabs(thread4 - max) > error_thread && max < 4500) {
  } else if (max > 4500) {
    thread4 = 4500;
  } else if (max != 0) {
    thread4 = max;
    if (thread4 > 10) thread4 -= 10;  // 提前量
  }
  std::cout << "thread4: " << thread4 << std::endl;
  //   res = 0;
  //   count = 0;
  max = 0;
  cv::Mat trd5(depth_data, cv::Range(399, 400));
  cv::minMaxLoc(trd5, NULL, &max, NULL, NULL);
  // std::cout << trd5 << std::endl;
  if (thread5 == 0) {
    if (max > 4000) {
      thread5 = 4000;
    } else if (max != 0) {
      thread5 = max;
      if (thread5 > 10) thread5 -= 10;  // 提前量
    }
  } else if (std::fabs(thread5 - max) > error_thread && max < 4000) {
  } else if (max > 4000) {
    thread5 = 4000;
  } else if (max != 0) {
    thread5 = max;
    if (thread5 > 10) thread5 -= 10;  // 提前量
  }
  std::cout << "thread5: " << thread5 << std::endl;

  max = 0;
  cv::Mat trd6(depth_data, cv::Range(478, 480));
  cv::minMaxLoc(trd6, NULL, &max, NULL, NULL);
  std::cout << max << std::endl;
  if (thread6 == 0) {
    if (max > 4000) {
      thread6 = 4000;
    } else if (max != 0) {
      thread6 = max;
      if (thread6 > 10) thread6 -= 10;  // 提前量
    }
  }
  //   else if (std::fabs(thread6 - max) > error_thread && max < 4000) {
  //   }
  else if (max > 4000) {
    thread6 = 4000;
  } else if (max != 0) {
    thread6 = max;
    if (thread6 > 10) thread6 -= 10;  // 提前量
  }
  std::cout << "thread6: " << thread6 << std::endl;
}
void D435::region_thread(cv::Mat &data) {
  int nr = data.rows;
  int nc = data.cols;
  int view_left = 100;
  int view_right = 540;
  int view_top = 20;
  for (int i = 0; i < view_top; i++) {
    for (int j = 0; j < nc; j++) {
      data.at<ushort>(i, j) = 255;
    }
  }
  for (int i = view_top; i < 80; i++) {
    for (int j = 0; j < nc; j++) {
      if (data.at<ushort>(i, j) > thread1 || data.at<ushort>(i, j) == 0 ||
          j < view_left || j > view_right) {
        data.at<ushort>(i, j) = thread1;
      }
      data.at<ushort>(i, j) = data.at<ushort>(i, j) * 255 / thread1;
    }
  }
  for (int i = 80; i < 160; i++) {
    for (int j = 0; j < nc; j++) {
      if (data.at<ushort>(i, j) > thread2 || data.at<ushort>(i, j) == 0 ||
          j < view_left || j > view_right) {
        data.at<ushort>(i, j) = thread2;
      }
      data.at<ushort>(i, j) = data.at<ushort>(i, j) * 255 / thread2;
    }
  }
  for (int i = 160; i < 240; i++) {
    for (int j = 0; j < nc; j++) {
      if (data.at<ushort>(i, j) > thread3 || data.at<ushort>(i, j) == 0 ||
          j < view_left || j > view_right) {
        data.at<ushort>(i, j) = thread3;
      }
      data.at<ushort>(i, j) = data.at<ushort>(i, j) * 255 / thread3;
    }
  }
  for (int i = 240; i < 320; i++) {
    for (int j = 0; j < nc; j++) {
      if (data.at<ushort>(i, j) > thread4 || data.at<ushort>(i, j) == 0 ||
          j < view_left || j > view_right) {
        data.at<ushort>(i, j) = thread4;
      }
      data.at<ushort>(i, j) = data.at<ushort>(i, j) * 255 / thread4;
    }
  }
  for (int i = 320; i < 400; i++) {
    for (int j = 0; j < nc; j++) {
      if (data.at<ushort>(i, j) > thread5 || data.at<ushort>(i, j) == 0 ||
          j < view_left || j > view_right) {
        data.at<ushort>(i, j) = thread5;
      }
      data.at<ushort>(i, j) = data.at<ushort>(i, j) * 255 / thread5;
    }
  }
  for (int i = 400; i < 480; i++) {
    for (int j = 0; j < nc; j++) {
      if (data.at<ushort>(i, j) > thread6 || data.at<ushort>(i, j) == 0 ||
          j < view_left || j > view_right) {
        data.at<ushort>(i, j) = thread6;
      }
      data.at<ushort>(i, j) = data.at<ushort>(i, j) * 255 / thread6;
    }
  }
}

void D435::matching() {
  cv::Mat match = depth_data.clone();
  for (int i = 0; i < match.rows; i++) {
    for (int j = 0; j < match.cols; j++) {
      if (i >= 0 && i < 80) {
        if (-match.at<ushort>(i, j) +
                (plan_arg.fir.a * j + (plan_arg.fir.b) * i + plan_arg.fir.c) <=
            40) {
          match.at<ushort>(i, j) = 0;
        } else {
          match.at<ushort>(i, j) = 255;
        }
      }

      if (i >= 80 && i < 160) {
        if (-match.at<ushort>(i, j) +
                (plan_arg.sec.a * j + (plan_arg.sec.b) * i + plan_arg.sec.c) <=
            45) {
          match.at<ushort>(i, j) = 0;
        } else {
          match.at<ushort>(i, j) = 255;
        }
      }

      if (i >= 160 && i < 240) {
        if (-match.at<ushort>(i, j) +
                (plan_arg.thrid.a * j + (plan_arg.thrid.b) * i +
                 plan_arg.thrid.c) <=
            1260) {
          match.at<ushort>(i, j) = 0;
        } else {
          match.at<ushort>(i, j) = 255;
        }
      }

      if (i >= 240 && i < 320) {
        if (-match.at<ushort>(i, j) +
                (plan_arg.four.a * j + (plan_arg.four.b) * i +
                 plan_arg.four.c) <=
            190) {
          match.at<ushort>(i, j) = 0;
        } else {
          match.at<ushort>(i, j) = 255;
        }
      }

      if (i >= 320 && i < 400) {
        if (-match.at<ushort>(i, j) +
                (plan_arg.five.a * j + (plan_arg.five.b) * i +
                 plan_arg.five.c) <=
            100) {
          match.at<ushort>(i, j) = 0;
        } else {
          match.at<ushort>(i, j) = 255;
        }
      }

      if (i >= 400 && i < 480) {
        if (-match.at<ushort>(i, j) +
                (plan_arg.six.a * j + (plan_arg.six.b) * i + plan_arg.six.c) <=
            80) {
          match.at<ushort>(i, j) = 0;
        } else {
          match.at<ushort>(i, j) = 255;
        }
      }
    }
  }
  match.convertTo(match, CV_8UC1, 1);
  cv::imshow("match", match);
  cv::waitKey(1);
}

bool D435::PlaneFitting(const std::vector<Vector3VP> &points_input,
                        double *center, double *normal) {
  int Num = points_input.size();
  std::vector<std::shared_ptr<GRANSAC::AbstractParameter>> CandPoints;
  CandPoints.resize(Num);
#pragma omp parallel for num_threads(6)
  for (int i = 0; i < Num; ++i) {
    Vector3VP p = points_input[i];
    std::shared_ptr<GRANSAC::AbstractParameter> CandPt =
        std::make_shared<Point3D>(p[0], p[1], p[2]);
    CandPoints[i] = CandPt;
  }

  GRANSAC::RANSAC<PlaneModel, 3> Estimator;
  Estimator.Initialize(0.1, 100);  // Threshold, iterations

  int64_t start = cv::getTickCount();
  Estimator.Estimate(CandPoints);
  int64_t end = cv::getTickCount();
  std::cout << "RANSAC took: "
            << GRANSAC::VPFloat(end - start) /
                   GRANSAC::VPFloat(cv::getTickFrequency()) * 1000.0
            << " ms." << std::endl;

  auto BestPlane = Estimator.GetBestModel();
  if (BestPlane == nullptr) {
    return false;
  }
  for (int i = 0; i < 3; i++) {
    center[i] = BestPlane->m_PointCenter[i];
  }
  for (int i = 0; i < 4; i++) {
    normal[i] = BestPlane->m_PlaneCoefs[i];
  }

  return true;
}
/* 补偿速度对图像的损失 */

void D435::image_translation(double delta_distance, cv::Mat iamge) {
  // 先由车体坐标转换到相机坐标
}

void D435::calibration() {
  std::vector<Vector3VP> point_cloud;
  std::vector<Vector3VP> point_cloud1;
  std::vector<Vector3VP> point_cloud2;
  std::vector<Vector3VP> point_cloud3;
  std::vector<Vector3VP> point_cloud4;
  std::vector<Vector3VP> point_cloud5;

  double *center = new double[3];
  double *coefs = new double[4];
  double *coefs1 = new double[4];
  double *coefs2 = new double[4];
  double *coefs3 = new double[4];
  double *coefs4 = new double[4];
  double *coefs5 = new double[4];
  int add_num = 80;

  for (double i = 0; i < add_num; i++) {
    for (double j = 0; j < depth_data.cols; j++) {
      if (depth_data.at<ushort>(i, j) == 0 ||
          depth_data.at<ushort>(i, j) > 65000) {
        continue;
      } else {
        Vector3VP Pt3d = {j, i,
                          static_cast<double>(depth_data.at<ushort>(i, j))};
        point_cloud.push_back(Pt3d);
        //   std::cout << static_cast<double>(depth_data.at<ushort>(i, j)) <<
        //   std::endl;
      }
    }
  }

  for (double i = add_num; i < add_num * 2; i++) {
    for (double j = 0; j < depth_data.cols; j++) {
      if (depth_data.at<ushort>(i, j) == 0 ||
          depth_data.at<ushort>(i, j) > 65000) {
        continue;
      } else {
        Vector3VP Pt3d = {j, i,
                          static_cast<double>(depth_data.at<ushort>(i, j))};
        point_cloud1.push_back(Pt3d);
        // std::cout << static_cast<double>(depth_data.at<ushort>(i, j))
        //           << std::endl;
      }
    }
  }

  for (double i = add_num * 2; i < add_num * 3; i++) {
    for (double j = 0; j < depth_data.cols; j++) {
      if (depth_data.at<ushort>(i, j) == 0 ||
          depth_data.at<ushort>(i, j) > 65000) {
        continue;
      } else {
        Vector3VP Pt3d = {j, i,
                          static_cast<double>(depth_data.at<ushort>(i, j))};
        point_cloud2.push_back(Pt3d);
        // std::cout << static_cast<double>(depth_data.at<ushort>(i, j))
        //           << std::endl;
      }
    }
  }

  for (double i = add_num * 3; i < add_num * 4; i++) {
    for (double j = 0; j < depth_data.cols; j++) {
      if (depth_data.at<ushort>(i, j) == 0 ||
          depth_data.at<ushort>(i, j) > 65000) {
        continue;
      } else {
        Vector3VP Pt3d = {j, i,
                          static_cast<double>(depth_data.at<ushort>(i, j))};
        point_cloud3.push_back(Pt3d);
        // std::cout << static_cast<double>(depth_data.at<ushort>(i, j))
        //           << std::endl;
      }
    }
  }

  for (double i = add_num * 4; i < add_num * 5; i++) {
    for (double j = 0; j < depth_data.cols; j++) {
      if (depth_data.at<ushort>(i, j) == 0 ||
          depth_data.at<ushort>(i, j) > 65000) {
        continue;
      } else {
        Vector3VP Pt3d = {j, i,
                          static_cast<double>(depth_data.at<ushort>(i, j))};
        point_cloud4.push_back(Pt3d);
        // std::cout << depth_data.at<ushort>(i, j) << " ";
        // std::cout << static_cast<double>(depth_data.at<ushort>(i, j))
        //           << std::endl;
      }
    }
  }

  for (double i = add_num * 5; i < add_num * 6; i++) {
    for (double j = 0; j < depth_data.cols; j++) {
      if (depth_data.at<ushort>(i, j) == 0 ||
          depth_data.at<ushort>(i, j) > 65000) {
        continue;
      } else {
        Vector3VP Pt3d = {j, i,
                          static_cast<double>(depth_data.at<ushort>(i, j))};
        point_cloud5.push_back(Pt3d);
        // std::cout << static_cast<double>(depth_data.at<ushort>(i, j))
        //           << std::endl;
      }
    }
  }

  //   Vector3VP Pt3d = {x, y, z};

  //   point_cloud.push_back(Pt3d);
  // }
  // }
  //   for (int i = 0; i < 300; i++) {
  //     for (int j = 0; j < 400; j++) {
  //       double x = double(i);
  //       double y = double(j);
  //       double z = a * x + b * y + d + rng.gaussian(w_sigma);

  //       Vector3VP Pt3d = {x, y, z};

  //       point_cloud.push_back(Pt3d);
  //     }
  //   }
  // perform Plane Fitting Algorithm
  PlaneFitting(point_cloud, center, coefs);
  for (int i = 0; i < 4; i++) {
    std::cout << coefs[i] << std::endl;
  }
  plan_arg.fir.a = coefs[0];
  plan_arg.fir.b = coefs[1];
  plan_arg.fir.c = coefs[3];
  std::cout << "plan_arg" << plan_arg.fir.a << " " << plan_arg.fir.b << " "
            << plan_arg.fir.c << std::endl;
  std::cout << std::endl;

  PlaneFitting(point_cloud1, center, coefs1);
  for (int i = 0; i < 4; i++) {
    std::cout << coefs1[i] << std::endl;
  }
  plan_arg.sec.a = coefs1[0];
  plan_arg.sec.b = coefs1[1];
  plan_arg.sec.c = coefs1[3];
  std::cout << std::endl;

  PlaneFitting(point_cloud2, center, coefs2);
  for (int i = 0; i < 4; i++) {
    std::cout << coefs2[i] << std::endl;
  }
  plan_arg.thrid.a = coefs2[0];
  plan_arg.thrid.b = coefs2[1];
  plan_arg.thrid.c = coefs2[3];
  std::cout << std::endl;

  PlaneFitting(point_cloud3, center, coefs3);
  for (int i = 0; i < 4; i++) {
    std::cout << coefs3[i] << std::endl;
  }
  plan_arg.four.a = coefs3[0];
  plan_arg.four.b = coefs3[1];
  plan_arg.four.c = coefs3[3];
  std::cout << std::endl;

  PlaneFitting(point_cloud4, center, coefs4);
  for (int i = 0; i < 4; i++) {
    std::cout << coefs4[i] << std::endl;
  }
  plan_arg.five.a = coefs4[0];
  plan_arg.five.b = coefs4[1];
  plan_arg.five.c = coefs4[3];
  std::cout << std::endl;
  PlaneFitting(point_cloud5, center, coefs5);
  for (int i = 0; i < 4; i++) {
    std::cout << coefs5[i] << std::endl;
  }
  plan_arg.six.a = coefs5[0];
  plan_arg.six.b = coefs5[1];
  plan_arg.six.c = coefs5[3];
  std::cout << std::endl;
}

void D435::handle_depth(std::vector<cv::Mat> data) {
  //   cv::Mat data;
  //   data = depth_data.clone();
  //   cv::medianBlur(data, data, 5);
  //   mask_depth(data, 3000);
  //   region_thread(data);
  //   data.convertTo(data, CV_8UC1, 1);
  //   cv::imshow("fenkuaizhuanhuan", data);
  //   cv::waitKey(1);
  //   std::cout << data << std::endl;
  //   quit_black_block(data);
  //   cv::imshow("depth_raw", data);

  //   cv::Mat element = cv::getStructuringElement(
  //       cv::MORPH_RECT, cv::Size(5, 5));  // 闭操作核的大小
  //   cv::morphologyEx(data, data, cv::MORPH_OPEN, element);  // 闭操作
  //   cv::Mat element1 = cv::getStructuringElement(
  //       cv::MORPH_RECT, cv::Size(7, 7));  // 膨胀操作核的大小
  //   cv::imshow("close", data);
  //   cv::erode(data, data, element1);
  //   cv::imshow("diate", data);
  //   cv::waitKey(1);
  //   std::cout << data << std::endl;
  clock_t start, stop;
  double duration;
  start = clock();
  find_obstacle(data, 170, 255, are_threshold);
  stop = clock();
  duration = static_cast<double>(stop - start) / CLOCKS_PER_SEC;
  //   std::cout << "consume time for find_obstacle(): " << duration <<
  //   "second"
  // << std::endl;
  start = clock();

  calculate_mindistance(450, 200);
  stop = clock();
  duration = static_cast<double>(stop - start) / CLOCKS_PER_SEC;
  //   std::cout << "consume time for calculate_minstance(): " << duration
  // << "second" << std::endl;
}

void D435::start_calibration() {
  calibration_data.open("calibration_data.csv", std::ios::out);
  if (calibration_data.is_open()) {
    std::cout << "calibration_data.csv file open sucess" << std::endl;
  } else {
    std::cout << "calibration_data.csv file open failed" << std::endl;
  }

  calibration_data << "1-a"
                   << ","
                   << "1-b"
                   << ","
                   << "1-c"
                   << ","
                   << "2-a"
                   << ","
                   << "2-b"
                   << ","
                   << "2-c"
                   << ","
                   << "3-a"
                   << ","
                   << "3-b"
                   << ","
                   << "3-c"
                   << ","
                   << "4-a"
                   << ","
                   << "4-b"
                   << ","
                   << "4-c"
                   << ","
                   << "5-a"
                   << ","
                   << "5-b"
                   << ","
                   << "5-c"
                   << ","
                   << "6-a"
                   << ","
                   << "6-b"
                   << ","
                   << "6-c" << std::endl;
  int i = 0;
  int row_start;
  int row_end;
  int col_start;
  int col_end;
  char in_cal = 'q';
  while (in_cal == 'q') {
    i = 0;
    std::cout << "please input row_start row_end col_start col_end **rows: "
                 "0-480** **cols: 0-640**"
              << std::endl;
    std::cin >> row_start;
    std::cin >> row_end;
    std::cin >> col_start;
    std::cin >> col_end;
    std::cout << "row_start:" << row_start << " "
              << "row_end:" << row_end << " "
              << "col_start:" << col_start << " "
              << "col_end:" << col_end << std::endl;
    std::cout << "please ensure args q or t  q: quit  t: true" << std::endl;
    while (i != 50) {
      i++;
      get_depth();
      depth_data = show_depth(row_start, row_end, col_start, col_end);
    }
    std::cin >> in_cal;
    if (in_cal == 't') {
      break;
    } else {
      in_cal = 'q';
    }
  }
  i = 0;
  while (i != 5) {
    char choose;
    std::cout << "plaese select t: true   q:quit" << std::endl;
    get_depth();
    depth_data = show_depth(row_start, row_end, col_start, col_end);
    std::cin >> choose;
    if (choose == 't') {
      std::string file_name("calibration");
      file_name = file_name + std::to_string(i + 1) + ".png";
      i++;
      calibration();
      cv::imwrite(file_name, depth_data);

      /*
            标定数据保存
   */

      calibration_data << plan_arg.fir.a << "," << plan_arg.fir.b << ","
                       << plan_arg.fir.c << ",";
      calibration_data << plan_arg.sec.a << "," << plan_arg.sec.b << ","
                       << plan_arg.sec.c << ",";
      calibration_data << plan_arg.thrid.a << "," << plan_arg.thrid.b << ","
                       << plan_arg.thrid.c << ",";
      calibration_data << plan_arg.four.a << "," << plan_arg.four.b << ","
                       << plan_arg.four.c << ",";
      calibration_data << plan_arg.five.a << "," << plan_arg.five.b << ","
                       << plan_arg.five.c << ",";
      calibration_data << plan_arg.six.a << "," << plan_arg.six.b << ","
                       << plan_arg.six.c << "," << std::endl;
      cv::waitKey(1000);
    } else if (choose == 'q') {
      continue;
    }
  }
  calibration_data.close();
}

/* 计算多项式参数 */

void D435::calculate_poly(cv::Mat mean_depth) {
  for (int i = 0; i < mean_depth.rows; i++) {
    std::vector<cv::Point> in_points;

    for (int j = 0; j < mean_depth.cols; j++) {
      in_points.push_back(cv::Point(j, mean_depth.at<ushort>(i, j)));
    }
    std::vector<double> res;
    res = polyfit(in_points, 5);
    poly.push_back(res);
    std::cout << "heni" << i << std::endl;
    std::cout << std::endl;
    in_points.clear();
    res.clear();
  }
}

std::vector<double> D435::polyfit(std::vector<cv::Point> &in_point, int n) {
  int size = in_point.size();
  //所求未知数个数
  int x_num = n + 1;
  // 构造矩阵U和Y
  cv::Mat mat_u(size, x_num, CV_64F);
  cv::Mat mat_y(size, 1, CV_64F);

  for (int i = 0; i < mat_u.rows; ++i)
    for (int j = 0; j < mat_u.cols; ++j) {
      mat_u.at<double>(i, j) = pow(in_point[i].x, j);
    }

  for (int i = 0; i < mat_y.rows; ++i) {
    mat_y.at<double>(i, 0) = in_point[i].y;
  }

  // 矩阵运算，获得系数矩阵K
  cv::Mat mat_k(x_num, 1, CV_64F);
  mat_k = (mat_u.t() * mat_u).inv() * mat_u.t() * mat_y;
  std::vector<double> res;
  for (int i = 0; i < mat_k.rows; i++) {
    std::cout << mat_k.at<float>(i, 0) << std::endl;
    res.push_back(static_cast<double>(mat_k.at<float>(i, 0)));
  }
  return res;

  //   return mat_k;
}

/* 获取三帧同时进行处理
可以试试四 五 六 帧的情况
*/
std::vector<cv::Mat> D435::Get3_depth(cv::Mat mean_depth_average,
                                      const std::vector<double> &threshold_data,
                                      int up_num, int nums, cv::Rect ROI_UP,
                                      cv::Rect ROI_DOWN) {
  clock_t start, stop;
  double duration;
  start = clock();
  std::vector<cv::Mat> three_map_up;
  std::vector<cv::Mat> three_map_down;

  std::vector<cv::Mat> deal_result;
  cv::Mat raw_deal_result = cv::Mat::zeros(Height, Width, CV_8UC1);

  three_map_up = get_depth2calculate(ROI_UP);
  three_map_down = get_depth2calculate(ROI_DOWN);

  //   for (int i = 0; i < three_map[0].rows; i++) {
  //     for (int j = 0; j < three_map[0].cols; j++) {
  //       if (static_cast<double>(mean_depth_average.at<double>(i, j)) -
  //               static_cast<double>(three_map[0].at<ushort>(i, j)) >
  //           threshold_data[i]) {
  //         raw_deal_result.at<char>(i, j) = 0;
  //       } else {
  //         raw_deal_result.at<char>(i, j) = 255;
  //       }
  //     }
  //   }

  //   cv::imshow("raw ", raw_deal_result);
  //   cv::waitKey(1);
  /* 把图片拼接起来 */

  for (int k = 0; k < up_num; k++) {
    cv::Mat result_data(Height, Width, CV_8UC1, cv::Scalar(255));

    thresholding(three_map_up, mean_depth_average(ROI_UP), threshold_data, k,
                 nums - 1, ROI_UP, result_data);

    thresholding(three_map_down, mean_depth_average(ROI_DOWN), threshold_data,
                 k, nums - 1, ROI_DOWN, result_data);
    deal_result.emplace_back(result_data.clone());
  }
  cv::imshow("after 0", deal_result[0]);
  cv::waitKey(1);
  cv::imshow("after 1", deal_result[1]);
  cv::waitKey(1);
  stop = clock();
  duration = static_cast<double>(stop - start) / CLOCKS_PER_SEC;
  //   std::cout << "consume time for Get3depth(): " << duration << "second"
  // << std::endl;
  start = clock();
  for (int h = 0; h < deal_result.size(); h++) {
    cv::Mat element = cv::getStructuringElement(
        cv::MORPH_RECT, cv::Size(3, 3));  // 闭操作核的大小
    cv::morphologyEx(deal_result[h], deal_result[h], cv::MORPH_CLOSE,
                     element);  // 闭操作
  }
  stop = clock();
  duration = static_cast<double>(stop - start) / CLOCKS_PER_SEC;
  //   std::cout << "consume time for close(): " << duration << "second"
  // << std::endl;
  cv::imshow("close 0", deal_result[0]);
  cv::waitKey(1);
  cv::imshow("close 1", deal_result[1]);
  cv::waitKey(1);
  //   cv::Mat element1 = cv::getStructuringElement(
  //       cv::MORPH_RECT, cv::Size(7, 7));  // 膨胀操作核的大小
  //   cv::imshow("close", data);
  //   cv::erode(data, data, element1);
  //   cv::imshow("diate", data);
  //   cv::waitKey(1);
  //   std::cout << data << std::endl;
  return deal_result;
}

/* 计算阈值参数 */
void D435::get_mean_depth() {
  cv::Mat_<double> mean_depth_average = cv::Mat_<double>::zeros(Height, Width);

  if (judge_file("mean_depth.png") && judge_file("threshold.csv")) {
    cv::Mat tmp = cv::imread("mean_depth.png", cv::IMREAD_ANYDEPTH);
    for (int i = 0; i < tmp.rows; i++) {
      for (int j = 0; j < tmp.cols; j++) {
        mean_depth_average.at<double>(i, j) =
            static_cast<double>(tmp.at<ushort>(i, j));
      }
    }
    std::cout << "mean_depth" << mean_depth_average << std::endl;
    std::ifstream inFile("threshold.csv", std::ios::in);
    std::string value;
    std::vector<double> data;
    getline(inFile, value);  // 读取整行进value中 读取第一行
    std::stringstream ss(value);
    std::string str;
    while (getline(ss, str, ',')) {       // 以逗号为分隔读取string
      data.push_back(atof(str.c_str()));  // string转为double
    }
    std::cout << "data" << std::endl;
    for (auto a : data) {
      std::cout << a << " ";
    }
    std::cout << std::endl;
    threshold_data = data;
  } else {
    /* 声明局部变量 */
    cv::Mat out(3000, 500, CV_8UC3, cv::Scalar::all(0));  // 用于绘图
    calibration_data.open("calibartion_data.csv",
                          std::ios::out | std::ios::app);  // 记录标定的数据
    if (calibration_data.is_open()) {
      std::cout << RED << "open sucess clibration_data" << RESET << std::endl;
    }
    std::fstream save_mean_depth;
    save_mean_depth.open("mean_depth_data.csv", std::ios::out | std::ios::app);
    if (save_mean_depth.is_open()) {
      std::cout << RED << "open sucess mean_depth_data" << RESET << std::endl;
    }

    //   cv::Mat_<double> mean_depth_average = cv::Mat_<double>::zeros(480,
    //   640);

    int flag_t = 0;

    for (int flag = 1; flag <= iter_times; flag++) {
      cv::Mat tmp_result(Height, Width, CV_16UC1);
      std::vector<cv::Mat> raw_datas;
      // std::cout << raw_datas.size() << std::endl;
      cv::Mat_<double> count = cv::Mat_<double>::zeros(Height, Width);
      cv::Mat_<double> result = cv::Mat_<double>::zeros(Height, Width);
      std::cout << "times" << flag << std::endl;
      cv::waitKey(50);
      cv::Rect rect1(left_edge, top_edge, right_edge - left_edge,
                     below_edge - top_edge);
      /* 开始取图片 */
      for (int k = 0; k < samples_nums_up; k++) {
        get_depth();
        /* 进行补全 ,去除零点*/
        for (int i = 0; i < depth_data.rows; i++) {
          for (int j = 0; j < depth_data.cols; j++) {
            if (depth_data.at<ushort>(i, j) > 7000) {
              depth_data.at<ushort>(i, j) = 7000;
              continue;
            }
            if (depth_data.at<ushort>(i, j) == 0) {
              int k = j + 1;
              int count = 0;
              while (1) {
                if (depth_data.at<ushort>(i, k % Width) != 0) {
                  depth_data.at<ushort>(i, j) =
                      depth_data.at<ushort>(i, k % Width);
                  break;
                }
                k++;
                count++;
                if (count > 900) {
                  break;
                }
              }
            }
          }
        }
        // std::cout << depth_data << std::endl;

        /* 修复反光 */
        for (int i = 0; i < depth_data.rows; i++) {
          for (int j = 1; j < depth_data.cols - 1; j++) {
            if (depth_data.at<ushort>(i, j + 1) - depth_data.at<ushort>(i, j) >
                500) {
              depth_data.at<ushort>(i, j + 1) = depth_data.at<ushort>(i, j);
            }
          }
        }
        // std::cout << depth_data << std::endl;

#ifdef DEBUG
        std::string depth_name("calibration_data");
        depth_name = "/home/zhongsy/Desktop/test/test_opencv/build/raw_data/" +
                     depth_name + std::to_string(k + 1) + ".png";
        cv::imwrite(depth_name, depth_data);
#endif

        cv::Mat raw_data_edge = depth_data(rect1);
        cv::Mat edge_data_display = raw_data_edge;
        edge_data_display.convertTo(edge_data_display, CV_8UC1,
                                    255.0 / 15000.0);
        cv::imshow("raw_rect_data", edge_data_display);
        cv::waitKey(10);
        raw_datas.push_back(raw_data_edge.clone());  //  存放感兴趣区域
        for (int i = 0; i < depth_data.rows; i++) {
          for (int j = 0; j < depth_data.cols; j++) {
            if (depth_data.at<ushort>(i, j) == 0) {
              continue;
            }
            if (depth_data.at<ushort>(i, j) > 10000) {
              continue;
            } else {
              count.at<double>(i, j) += 1;
              result.at<double>(i, j) +=
                  static_cast<double>(depth_data.at<ushort>(i, j));
            }
          }
        }

        cv::waitKey(15);
      }
      /* 结束取图 */

      for (int i = 0; i < result.rows; i++) {
        for (int j = 0; j < result.cols; j++) {
          if (count.at<double>(i, j) == 0) {
            result.at<double>(i, j) = 0;
            continue;
          }
          result.at<double>(i, j) = static_cast<double>(
              result.at<double>(i, j) / count.at<double>(i, j));
        }
      }

      //   for (int i = 0; i < result.rows; i++) {
      //     for (int j = 0; j < result.cols; j++) {
      //       if (result.at<double>(i, j) == 0) {
      //         int k = j + 1;
      //         int flag = 1;
      //         int count = 0;
      //         while (flag) {
      //           if (result.at<double>(i, k % Width) != 0) {
      //             flag = 0;
      //             result.at<double>(i, j) = result.at<double>(i, k %
      //             Width);
      //           }
      //           k++;
      //           count++;
      //           if (count > 500) {
      //             break;
      //           }
      //         }
      //       }
      //     }
      //   }

      /*
          保存每一次的均值
       */
      save_mean_depth << " maean_depth" << flag << std::endl;
      for (int i = 0; i < result.rows; i++) {
        for (int j = 0; j < result.cols; j++) {
          save_mean_depth << result.at<double>(i, j) << ",";
        }
        save_mean_depth << std::endl;
      }
      // mean_depth_average += result;  // 将平均值加起来
      if (flag_t == 0) {
        mean_depth_average = result;
        flag_t = 1;
      } else {
        for (int i = 0; i < mean_depth_average.rows; i++) {
          for (int j = 0; j < mean_depth_average.cols; j++) {
            mean_depth_average.at<double>(i, j) =
                std::max(mean_depth_average.at<double>(i, j),
                         result.at<double>(i, j));  // 背景取的是最远的
          }
        }
      }

      // cv::Mat tmp_result(480, 640, CV_16UC1);
      for (int i = 0; i < result.rows; i++) {
        for (int j = 0; j < result.cols; j++) {
          tmp_result.at<ushort>(i, j) =
              static_cast<ushort>(result.at<double>(i, j));
        }
      }
//   calculate_poly(result);
#ifdef DEBUG
      cv::Mat conv;
      conv.create(480, 640, CV_16UC1);
      conv = result;
      conv.convertTo(conv, CV_16UC1, 1);
      cv::imwrite("mean_depth.png", conv);
#endif

      //   cv::Rect rect(left_edge, top_edge, right_edge - left_edge,
      //                 below_edge - top_edge);
      cv::Mat edge_data = tmp_result(rect1);  // 感兴趣的均值

      if (threshold_data.empty()) {
        threshold_data =
            calculate_max_threshold(edge_data, raw_datas);  // 开始计算阈值
        calibration_data << "time: "
                         << "," << flag << ","
                         << "samples_num: "
                         << "," << flag * samples_nums_up << std::endl;
        for (auto value : threshold_data) {
          calibration_data << value << ",";
        }
        calibration_data << std::endl;
      } else {
        calibration_data << "time: "
                         << "," << flag << ","
                         << "samples_num: "
                         << "," << flag * samples_nums_up << std::endl;
        std::vector<double> threshold_data_tmp =
            calculate_max_threshold(edge_data, raw_datas);
        for (auto value : threshold_data_tmp) {
          calibration_data << value << ",";
        }
        calibration_data << std::endl;
        for (int l = 0; l < threshold_data.size(); l++) {
          threshold_data[l] = std::min(
              threshold_data[l], threshold_data_tmp[l]);  // 取的是最小的阈值
        }
      }
#ifdef DEBUG
      for (auto a : threshold_data) {
        std::cout << a << "  ";
      }
      std::cout << "threshold size: " << threshold_data.size() << std::endl;
      std::cout << std::endl;
#endif
      // 取四次阈值当中的最大值
      raw_datas.clear();
    }

#ifdef DEBUG
    for (int i = 0; i < threshold_data.size(); i++) {
      cv::Point2i drawing;
      drawing.x = static_cast<int>(i);
      drawing.y = static_cast<int>(threshold_data[i]);
      cv::circle(out, drawing, 5, cv::Scalar(255, 255, 255), CV_FILLED, CV_AA);
    }
    cv::imwrite("calibration.jpg", out);
#endif
    /*
      save calibration data
   */
    calibration_data.close();
    calibration_data.open("threshold.csv", std::ios::out | std::ios::trunc);
    // calibration_data << "min_threshold" << std::endl;
    for (int h = 0; h < threshold_data.size(); h++) {
      // threshold_data[h] = threshold_data[h] ;
      calibration_data << threshold_data[h] << ",";
    }
    calibration_data << std::endl;
    calibration_data.close();

    /*
       计算并保存平均深度图
    */

    //   mean_depth_average = mean_depth_average / iter_times;
    save_mean_depth << "mean_depth_average" << std::endl;
    for (int i = 0; i < mean_depth_average.rows; i++) {
      for (int j = 0; j < mean_depth_average.cols; j++) {
        save_mean_depth << mean_depth_average.at<double>(i, j) << ",";
      }
      save_mean_depth << std::endl;
    }
    // 无损保存深度图
    std::vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_STRATEGY_DEFAULT);
    compression_params.push_back(0);  // 无压缩png.
    cv::Mat conv = cv::Mat::zeros(Height, Width, CV_16UC1);
    for (int i = 0; i < mean_depth_average.rows; i++) {
      for (int j = 0; j < mean_depth_average.cols; j++) {
        conv.at<ushort>(i, j) =
            static_cast<ushort>(mean_depth_average.at<double>(i, j));
      }
    }
    imwrite("mean_depth.png", conv, compression_params);
    // 显示出来
    // cv::Mat test = cv::imread("mean_depth.png", cv::IMREAD_ANYDEPTH);
    // std::cout << test << std::endl;
  }

  std::vector<double> threshold_data_tmp = threshold_data;
  //   light_stream.resize(2);
  //   for (int i = 0; i < light_stream.size(); i++) {
  //     while (light_stream[i].size() <= 4) {
  //       get_depth();
  //       cv::Mat Display = depth_data.clone();
  //       Display = thresholding(Display.clone(), mean_depth_average,
  //                              threshold_data_tmp, i);
  //       light_stream[i].push_back(Display.clone());
  //       // std::cout << Di'splay << std::endl;
  //     }
  //   }
  //   std::cout << "size_init" << light_stream.size() << std::endl;
  while (1) {
    clockid_t start, stop;
    int nums = 3;
    double duration;
    if (_kbhit()) {
      std::cin.clear();
      std::cin.ignore();
      std::cout << "srart change args" << std::endl;
      std::cout << "please input numbers of threashold" << std::endl;
      double up_data_value = 0;
      for (int i = 0; i < up_to_nums.size(); i++) {
        std::cout << "raw_data: ";
        std::cout << up_to_nums[i] << std::endl;
        std::cin >> up_data_value;
        up_to_nums[i] = up_data_value;
        std::cout << "deal_after_data: " << std::endl;
        std::cout << up_to_nums[i] << " ";
      }

      std::cout << std::endl;

      std::cout << "please input ares_thread" << std::endl;
      for (int i = 0; i < are_threshold.size(); i++) {
        std::cout << "raw_ares_thread " << i << ": " << are_threshold[i]
                  << std::endl;
        std::cin >> are_threshold[i];
        std::cout << "are_threshold: " << i << ": " << are_threshold[i]
                  << std::endl;
      }

      std::cout << "if you want to change nums" << std::endl;
      std::cout << "please input t or f" << std::endl;
      char tmp;
      std::cin >> tmp;
      if (tmp == 't') {
        int num = 0;
        std::cout << "please in put a num" << std::endl;
        std::cin >> num;
        nums = num;
      }
      std::cin.clear();
      std::cin.ignore();
    }
    start = clock();
    ROI_UP = cv::Rect(300, 0, 200, 240);
    ROI_DOWN = cv::Rect(160, 240, 520, 240);
    cv::Mat display_rect1 = cv::Mat::zeros(Height, Width, CV_8UC3);

    cv::rectangle(display_rect1, ROI_UP, cv::Scalar(0, 0, 255));
    cv::rectangle(display_rect1, ROI_DOWN, cv::Scalar(0, 0, 255));

    cv::imshow("display_rect", display_rect1);
    handle_depth(Get3_depth(mean_depth_average, threshold_data_tmp, 2, nums,
                            ROI_UP, ROI_DOWN));
    stop = clock();
    duration = static_cast<double>(stop - start) / CLOCKS_PER_SEC;
    // std::cout << "consume time for handle: " << duration << "second"
    //   << std::endl;
  }
}

/* 函数作用为了把得到的图转换成二值图 */
void D435::thresholding2gray(cv::Mat data, cv::Mat mean_depth,
                             const std::vector<double> &thread_data, int h,
                             int nums, cv::Rect ROI, cv::Mat result_image) {
  for (int i = 0; i < mean_depth.rows; i++) {
    for (int j = 0; j < mean_depth.cols; j++) {
      if (data.at<ushort>(i, j) == 0) {
        result_image.at<uchar>(i + ROI.y, j + ROI.x) = 255;
      }
      if (static_cast<double>(mean_depth.at<double>(i, j)) -
              static_cast<double>(data.at<ushort>(i, j)) >
          thread_data[i + ROI.y] * up_to_nums[h]) {
                    result_image.at<uchar>(i + ROI.y, j + ROI.x) = 0;;

      } else {
        
      }
    }
  }
}

void D435::thresholding(const std::vector<cv::Mat> &data, cv::Mat mean_depth,
                        const std::vector<double> &thread_data, int h, int nums,
                        cv::Rect ROI, cv::Mat result) {
  //   std::cout << "mean_depth" << mean_depth <<"mean_depth_end"
  //   <<std::endl; std::cout << "raw_data" << data <<"raw_data_end"
  //   <<std::endl;
  clockid_t start, stop;
  double duration;
  start = clock();
  for (int i = 0; i < mean_depth.rows; i++) {
    for (int j = 0; j < mean_depth.cols; j++) {
      int count = 0;
      for (int k = 0; k < data.size(); k++) {
        if (data[k].at<ushort>(i, j) == 0) {
          break;
        }
        if (static_cast<double>(mean_depth.at<double>(i, j)) -
                static_cast<double>(data[k].at<ushort>(i, j)) >
            thread_data[i + ROI.y] * up_to_nums[h]) {
          count++;
        }
      }
      if (count > nums) {
        result.at<uchar>(i + ROI.y, j + ROI.x) = 0;
      } else {
        result.at<uchar>(i + ROI.y, j + ROI.x) = 255;
      }
    }
  }

  // if (static_cast<double>(mean_depth_average.at<ushort>(i, j)) -
  //         static_cast<double>(Display.at<ushort>(i, j)) <
  //     threshold_data[i] * 2.0 * std::pow(1.52, (479 - i) / 200))
  //   cv::Mat element1 = cv::getStructuringElement(cv::MORPH_RECT,
  //                                                cv::Size(7, 7));  //
  //                                                操作核的大小
  //   cv::morphologyEx(data, data, cv::MORPH_CLOSE,
  //                    element1);  // 开操作

  //   cv::Mat element2 = cv::getStructuringElement(cv::MORPH_RECT,
  //                                                cv::Size(4, 4));  //
  //                                                操作核的大小
  //   cv::dilate(data, data, element2);
  //   cv::medianBlur(data, data, 5);  // 中值滤波
  //   data.convertTo(data, CV_8UC1, 1);
  stop = clock();
  duration = static_cast<double>(stop - start) / CLOCKS_PER_SEC;
  //   std::cout << "consume time for thresholding(): " << duration <<
  //   "second"
  // << std::endl;
}

void D435::save_depth_image() {
  for (int i = 1; i <= 200; i++) {
    std::string depth_name("calibration_data");
    get_depth();
    depth_name = "/home/zhongsy/Desktop/test/test_opencv/build/raw_data/" +
                 depth_name + std::to_string(i) + ".png";
    cv::imwrite(depth_name, depth_data);
    std::cout << "save depthimage: " << i << std::endl;
    cv::waitKey(20);
  }
}

void D435::HandleFeedbackData() {
  //   while (1) {
  //     get_depth2calculate();
  //   }
  calibration_angle();
  get_mean_depth();
  //   start_calibration();
  //   save_depth_image();
  // while (1) {
  //   get_depth();
  //   matching();
  //     // separate_byte();
  //     // caculate_thread4();
  //   handle_depth();
  // }
}
