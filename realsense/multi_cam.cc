#include <iostream>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

int main() {
  std::vector<rs2::pipeline> pipelines;
  rs2::context ctx;
  rs2::colorizer colorizered;

  for (auto &&dev : ctx.query_devices()) {
    rs2::pipeline pipe(ctx);
    rs2::config cfg;
    cfg.enable_device(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    pipe.start(cfg);
    pipelines.emplace_back(pipe);
  }
  while (1) {
    std::cout << pipelines.size() << std::endl;

    rs2::frameset frame = pipelines.front().wait_for_frames();
    rs2::frame deal_frame = frame.get_depth_frame();
    auto show_frame = colorizered.process(deal_frame);
    cv::Mat colorized_mat(cv::Size(640, 480), CV_8UC3,
                          (void *)show_frame.get_data(), cv::Mat::AUTO_STEP);
    cv::imshow("coloried", colorized_mat);
    cv::waitKey(2);
    rs2::frameset frame_2 = pipelines[1].wait_for_frames();
    rs2::frame deal_frame_2 = frame_2.get_depth_frame();
    deal_frame_2 = deal_frame_2.apply_filter(colorizered);
    cv::Mat colorized_mat_2(cv::Size(640, 480), CV_8UC3,
                            (void *)deal_frame_2.get_data(),
                            cv::Mat::AUTO_STEP);
    cv::imshow("coloried_2", colorized_mat_2);
    cv::waitKey(2);
  }
}