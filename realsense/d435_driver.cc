/*
 "Copyright [year] <Copyright Owner>"
*/
#include "d435_driver.h"

#include <opencv2/imgproc/types_c.h>

#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>

void D435::Init() {
  // rs2::context ctx;
  //获取设备列表
  dev_list = ctx.query_devices();
  if (dev_list.size() == 0) {
    // ROS_ERROR("D435 not detected.");
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
  cfg.enable_stream(RS2_STREAM_COLOR, Width, Height, RS2_FORMAT_RGB8, fps);
  cfg.enable_stream(RS2_STREAM_DEPTH, Width, Height, RS2_FORMAT_Z16, fps);
  cfg.enable_stream(RS2_STREAM_INFRARED, 1, Width, Height, RS2_FORMAT_Y8, fps);
  cfg.enable_stream(RS2_STREAM_INFRARED, 2, Width, Height, RS2_FORMAT_Y8, fps);
  pipe.start(cfg);
  std::cout << "device start" << std::endl;
  // AbstractDriver::SetState(DriverState::READY);
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
  frames = pipe.wait_for_frames();
  rs2::depth_frame depth_frame = frames.get_depth_frame();
  cv::Mat depth(cv::Size(Width, Height), CV_16UC1,
                (void *)depth_frame.get_data(), cv::Mat::AUTO_STEP);
  //   cv::imshow("depth", depth);
  //   cv::waitKey(1);
  //   cv::Mat display = depth.clone();
  //   display.convertTo(display, CV_8UC1, 255.0 / 7000.0, 0.0);
  //   cv::imshow("depth", display);
  //   cv::waitKey(1);
  depth.copyTo(depth_data);
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

void D435::find_obstacle(cv::Mat &depth, int thresh, int max_thresh, int area) {
  cv::Mat dep;
  dep = depth.clone();
  cv::Mat threshold_output;
  std::vector<std::vector<cv::Point>> contours;

  std::vector<cv::Vec4i> hierarchy;
  cv::RNG rng(12345);
  // 阈值分割
  threshold(dep, threshold_output, thresh, 255, cv::THRESH_BINARY_INV);
  cv::imshow("thread_later", threshold_output);
  cv::waitKey(1);
  // mask_depth(src, threshold_output);
  /// 寻找轮廓
  findContours(threshold_output, contours, hierarchy, CV_RETR_TREE,
               CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
  /// 对每个轮廓计算其凸包
  std::vector<std::vector<cv::Point>> hull(contours.size());
  for (uint i = 0; i < contours.size(); i++) {
    convexHull(cv::Mat(contours[i]), hull[i], false);
  }

  /// 绘出轮廓及其凸包
  cv::Mat drawing = cv::Mat::zeros(threshold_output.size(), CV_8UC3);
  for (int i = 0; i < contours.size(); i++) {
    if (contourArea(contours[i]) < area ||
        contourArea(contours[i]) > 306000)  // 面积大于或小于area的凸包，可忽略
      continue;
    result.push_back(hull[i]);
    cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255),
                                  rng.uniform(0, 255));
    drawContours(drawing, contours, i, color, 1, 8, std::vector<cv::Vec4i>(), 0,
                 cv::Point());
    drawContours(drawing, hull, i, color, 1, 8, std::vector<cv::Vec4i>(), 0,
                 cv::Point());
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

void D435::calculate_mindistance() {
  std::vector<cv::Rect> ve_rect;
  cv::Mat drawing = cv::Mat::zeros(cv::Size(640, 480), CV_8UC3);

  if (result.empty()) {
    // 设置避障等级为0
    // min_distance = 3000;
    // std::cout << min_distance << std::endl;
  } else {
    for (int i = 0; i < result.size(); i++) {
      cv::Rect rect;
      rect = cv::boundingRect(result[i]);
      //   if (rect.area() > 304000) {continue;}
      ve_rect.push_back(rect);
    }

    std::vector<double> res;
    if (!ve_rect.empty()) {
      for (int i = 0; i < ve_rect.size(); i++) {
        cv::Mat imageROI(depth_data, ve_rect[i]);
        // cv::imshow("ROI", imageROI);
        // cv::waitKey(1);
        // 过滤零点
        cv::rectangle(drawing, ve_rect[i], cv::Scalar(2, 0, 255));
        for (int i = 0; i < imageROI.rows; i++) {
          for (int j = 0; j < imageROI.cols; j++) {
            if (imageROI.at<ushort>(i, j) == 0) {
              imageROI.at<ushort>(i, j) = 3000;
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
        while (tmp.size() <= 3) {
          cv::minMaxLoc(imageROI, &min_dis, NULL, &min_point, NULL);
          //  std::cout<<"min_dis_roi"<<min_dis<<std::endl;
          // 优化获取的是前面3个最小点的平均值
          if (tmp.empty()) {
            tmp.push_back(min_dis);
            imageROI.at<ushort>(min_point) = 3000;
          } else {
            if (std::fabs(min_dis - tmp.back()) < 40) {
              tmp.push_back(min_dis);
              imageROI.at<ushort>(min_point) = 3000;
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
        res.push_back(min_dis / 4.0);
      }

      if (!res.empty()) {
        min_distance = *std::min_element(res.begin(), res.end());
        std::cout << "min_diatance" << min_distance << std::endl;
        if (min_distance < 500) {
          std::cout << "stop avoid" << std::endl;
        } else if (min_distance < 2000) {
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
    }
  }
  result.clear();
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

void D435::handle_depth() {
  cv::Mat data;
  data = depth_data.clone();
  cv::medianBlur(data, data, 5);
  //   mask_depth(data, 3000);
  region_thread(data);
  data.convertTo(data, CV_8UC1, 1);
  cv::imshow("fenkuaizhuanhuan", data);
  cv::waitKey(1);
  //   std::cout << data << std::endl;
  quit_black_block(data);
  cv::imshow("depth_raw", data);

  cv::Mat element = cv::getStructuringElement(
      cv::MORPH_RECT, cv::Size(5, 5));  // 闭操作核的大小
  cv::morphologyEx(data, data, cv::MORPH_OPEN, element);  // 闭操作
  cv::Mat element1 = cv::getStructuringElement(
      cv::MORPH_RECT, cv::Size(7, 7));  // 膨胀操作核的大小
  cv::imshow("close", data);
  cv::erode(data, data, element1);
  cv::imshow("diate", data);
  cv::waitKey(1);
  //   std::cout << data << std::endl;
  find_obstacle(data, 170, 255, 500);
  calculate_mindistance();
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

void D435::get_mean_depth() {
  cv::Mat_<double> count(480, 640);
  cv::Mat_<double> result(480, 640);
  for (int k = 0; k <= 200; k++) {
    std::string depth_name("calibration_data");
    depth_name = "/home/zhongsy/Desktop/test/test_opencv/build/raw_data/" +
                 depth_name + std::to_string(k + 1) + ".png";
    get_depth();
    cv::imwrite(depth_name, depth_data);
    for (int i = 0; i < depth_data.rows; i++) {
      for (int j = 0; j < depth_data.cols; j++) {
        if (depth_data.at<ushort>(i, j) == 0 ||
            depth_data.at<ushort>(i, j) > 10000) {
          continue;
        }
        count.at<double>(i, j)++;
        result.at<double>(i, j) +=
            static_cast<double>(depth_data.at<ushort>(i, j));
      }
    }
  }
  for (int i = 0; i < result.rows; i++) {
    for (int j = 0; j < result.cols; j++) {
      if (count.at<double>(i, j) == 0) {
        result.at<double>(i, j) = 0;
        continue;
      }
      result.at<double>(i, j) =
          result.at<double>(i, j) / count.at<double>(i, j);
    }
  }

  cv::Mat conv;
  conv.create(480, 640, CV_16UC1);
  conv = result;
  conv.convertTo(conv, CV_16UC1, 1);
  cv::imwrite("mean_depth.png", conv);
  //   std::cout << conv << std::endl;
  while (1) {
    get_depth();
    for (int i = 0; i < 80; i++) {
      for (int j = 0; j < depth_data.cols; j++) {
        if (result.at<double>(i, j) -
                static_cast<double>(depth_data.at<ushort>(i, j)) <
            1670) {
          depth_data.at<ushort>(i, j) = 255;
        } else {
          depth_data.at<ushort>(i, j) = 0;
        }
      }
    }

    for (int i = 80; i < 160; i++) {
      for (int j = 0; j < depth_data.cols; j++) {
        if (result.at<double>(i, j) -
                static_cast<double>(depth_data.at<ushort>(i, j)) <
            1570) {
          depth_data.at<ushort>(i, j) = 255;
        } else {
          depth_data.at<ushort>(i, j) = 0;
        }
      }
    }

    for (int i = 160; i < 240; i++) {
      for (int j = 0; j < depth_data.cols; j++) {
        if (result.at<double>(i, j) -
                static_cast<double>(depth_data.at<ushort>(i, j)) <
            370) {
          depth_data.at<ushort>(i, j) = 255;
        } else {
          depth_data.at<ushort>(i, j) = 0;
        }
      }
    }

    for (int i = 240; i < 320; i++) {
      for (int j = 0; j < depth_data.cols; j++) {
        if (result.at<double>(i, j) -
                static_cast<double>(depth_data.at<ushort>(i, j)) <
            100) {
          depth_data.at<ushort>(i, j) = 255;
        } else {
          depth_data.at<ushort>(i, j) = 0;
        }
      }
    }

    for (int i = 320; i < 400; i++) {
      for (int j = 0; j < depth_data.cols; j++) {
        if (result.at<double>(i, j) -
                static_cast<double>(depth_data.at<ushort>(i, j)) <
            40) {
          depth_data.at<ushort>(i, j) = 255;
        } else {
          depth_data.at<ushort>(i, j) = 0;
        }
      }
    }

    for (int i = 400; i < 480; i++) {
      for (int j = 0; j < depth_data.cols; j++) {
        if (result.at<double>(i, j) -
                static_cast<double>(depth_data.at<ushort>(i, j)) <
            45) {
          depth_data.at<ushort>(i, j) = 255;
        } else {
          depth_data.at<ushort>(i, j) = 0;
        }
      }
    }
    // std::cout << result << std::endl;
    cv::Mat element1 = cv::getStructuringElement(
        cv::MORPH_RECT, cv::Size(12, 12));  // 操作核的大小
    cv::morphologyEx(depth_data, depth_data, cv::MORPH_CLOSE,
                     element1);  // 开操作
    cv::threshold(depth_data, depth_data, 100, 255,
                  cv::THRESH_BINARY_INV);  // 黑白倒转
    cv::Mat element2 = cv::getStructuringElement(
        cv::MORPH_RECT, cv::Size(4, 4));  // 操作核的大小
    cv::dilate(depth_data, depth_data, element2);
    depth_data.convertTo(depth_data, CV_8UC1, 1);
    cv::imshow("depth_mean", depth_data);
    cv::waitKey(1);
  }
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
  get_mean_depth();
  // start_calibration();
  //   save_depth_image();
  // while (1) {
  //   get_depth();
  //   matching();
  //     // separate_byte();
  //     // caculate_thread4();
  //     // handle_depth();
  // }
}
