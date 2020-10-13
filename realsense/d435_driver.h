
#ifndef _D435_DRIVER_H_
#define _D435_DRIVER_H_

#include <deque>
#include <fstream>
#include <librealsense2/rs.hpp>
#include <list>
#include <memory>
#include <opencv2/opencv.hpp>
#include <thread>
#include <vector>

#include "GRANSAC.hpp"
#include "PlaneModel.hpp"

const float pi = 3.1415926;
// #define DEBUG ;

/* 划定标定范围 */
namespace {
int left_edge = 200;
int right_edge = 510;
int top_edge = 0;
int below_edge = 480;
int flag_threads = 0;
int iter_times = 2;         // 迭代次数
int samples_nums_up = 100;  // 计算阈值样本增量
}  // namespace
namespace {
const int Width = 848;
const int Height = 480;
const int fps = 90;  // 帧数最大能支持90

}  // namespace

struct arg_plane {
  double a;
  double b;
  double c;
};

struct argument {
  arg_plane fir;
  arg_plane sec;
  arg_plane thrid;
  arg_plane four;
  arg_plane five;
  arg_plane six;
};

class D435 {
 public:
  D435(){};
  ~D435(){};

  // 实现AbstractDriver的接口
  void Init();
  void GetData(void *data);
  std::vector<cv::Mat> Get3_depth(cv::Mat mean_depth_average,
                                  const std::vector<double> &threshold_data,
                                  int up_num, int nums, cv::Rect ROI);
  std::vector<cv::Mat> get_depth2calculate(cv::Rect ROI);
  void separate_byte();
  void matching();
  void save_depth_image();
  void get_mean_depth();  // 获取平均地面值
  bool PlaneFitting(const std::vector<Vector3VP> &points_input, double *center,
                    double *normal);
  void calibration();
  cv::Mat show_depth(int row_start, int row_end, int col_start, int col_end);
  void start_calibration();
  void calculate_poly(cv::Mat mean_depth);
  cv::Mat thresholding(const std::vector<cv::Mat> &data, cv::Mat mean_depth,
                       const std::vector<double> &thread_data, int h, int nums);
  void calibration_angle();

 private:
  //自定义接口
  void HandleFeedbackData();
  void get_depth();
  std::vector<double> polyfit(std::vector<cv::Point> &in_point,
                              int n);  // 计算多项式参数函数
  void handle_depth(std::vector<cv::Mat> data);
  void quit_black_block(cv::Mat &image);
  void mask_depth(cv::Mat &image, int throld = 3000);
  void find_obstacle(std::vector<cv::Mat> depth, int thresh, int max_thresh,
                     std::vector<int> areas);
  void calculate_mindistance(float threshold_x, float threshold_y);
  void region_thread(cv::Mat &data);
  void caculate_thread4();
  cv::Vec3f pixel_to_world(cv::Vec3f point);
  std::shared_ptr<std::thread> run_executor_;
  rs2::context ctx;
  rs2::frameset frames;
  rs2::pipeline pipe;
  rs2::spatial_filter spatial_filter;
  rs2::threshold_filter thd_filter;
  rs2::hole_filling_filter hole_filter;
  rs2::decimation_filter decimation_filter;
  rs2::temporal_filter temporal_filter;
  rs2::colorizer colorizered;
  rs2::device_list dev_list;
  rs2::device dev;
  cv::Mat depth_data;
  double min_distance;
  std::vector<std::vector<cv::Point>> result;  //存放凸包
  double thread1;
  double thread2;
  double thread3;
  double thread4;
  double thread5;
  double thread6;
  argument plan_arg;
  std::fstream calibration_data;
  std::vector<std::vector<double>> poly;
  std::vector<double> threshold_data;
  std::vector<std::deque<cv::Mat>> light_stream;  // 存放用来平均的图片
  std::vector<int> are_threshold;
  std::vector<double> up_to_nums;
  rs2::stream_profile dprofile;  // 用来存放深度相机参数
  rs2_intrinsics depth_intrin;
  float ration_angle;
  cv::Rect ROI;
};
#endif  // REALSENSE_D435_DRIVER_H_
