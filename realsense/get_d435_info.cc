#include <iostream>
#include <librealsense2/rs.hpp>  // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>

int main(int argc, char* argv[]) try {
  rs2::log_to_console(RS2_LOG_SEVERITY_ERROR);

  /// Create librealsense context for managing devices
  rs2::context ctx;
  auto devs = ctx.query_devices();  ///获取设备列表
  int device_num = devs.size();
  std::cout << "device num: " << device_num << std::endl;  ///设备数量

  ///我只连了一个设备，因此我查看第0个设备的信息
  /// 当无设备连接时此处抛出rs2::error异常
  rs2::device dev = devs[0];
  ///设备编号，每个设备都有一个不同的编号,
  ///如果连接了多个设备，便可根据此编号找到你希望启动的设备
  char serial_number[100] = {0};
  strcpy(serial_number, dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
  printf("serial_number: %s\n", serial_number);

  ///设置从设备管道获取的深度图和彩色图的配置对象
  rs2::config cfg;
  ///配置彩色图像流：分辨率640*480，图像格式：BGR， 帧率：30帧/秒
  ///默认配置任意一个设备，若要配置指定的设备可以根据设备在设备列表里的序列号进行制定:
  /// int indx = 0; ///表示第0个设备
  /// cfg.enable_stream(RS2_STREAM_COLOR,indx, 640, 480, RS2_FORMAT_BGR8, 30);
  cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
  ///配置深度图像流：分辨率640*480，图像格式：Z16， 帧率：30帧/秒
  cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);

  ///生成Realsense管道，用来封装实际的相机设备
  rs2::pipeline pipe;
  pipe.start(cfg);  ///根据给定的配置启动相机管道

  rs2::frameset data;
  data = pipe.wait_for_frames();  ///等待一帧数据，默认等待5s

  rs2::depth_frame depth = data.get_depth_frame();  ///获取深度图像数据
  rs2::video_frame color = data.get_color_frame();  ///获取彩色图像数据
  rs2::stream_profile dprofile = depth.get_profile();
  rs2::stream_profile cprofile = color.get_profile();

  ///获取彩色相机内参
  rs2::video_stream_profile cvsprofile(cprofile);
  rs2_intrinsics color_intrin = cvsprofile.get_intrinsics();
  std::cout << "\ncolor intrinsics: ";
  std::cout << color_intrin.width << "  " << color_intrin.height << "  ";
  std::cout << color_intrin.ppx << "  " << color_intrin.ppy << "  ";
  std::cout << color_intrin.fx << "  " << color_intrin.fy << std::endl;
  std::cout << "coeffs: ";
  for (auto value : color_intrin.coeffs) std::cout << value << "  ";
  std::cout << std::endl;
  std::cout << "distortion model: " << color_intrin.model
            << std::endl;  ///畸变模型

  ///获取深度相机内参
  rs2::video_stream_profile dvsprofile(dprofile);
  rs2_intrinsics depth_intrin = dvsprofile.get_intrinsics();
  std::cout << "\ndepth intrinsics: ";
  std::cout << depth_intrin.width << "  " << depth_intrin.height << "  ";
  std::cout << depth_intrin.ppx << "  " << depth_intrin.ppy << "  ";
  std::cout << depth_intrin.fx << "  " << depth_intrin.fy << std::endl;
  std::cout << "coeffs: ";
  for (auto value : depth_intrin.coeffs) std::cout << value << "  ";
  std::cout << std::endl;
  std::cout << "distortion model: " << depth_intrin.model
            << std::endl;  ///畸变模型

  ///获取深度相机相对于彩色相机的外参，即变换矩阵: P_color = R * P_depth + T
  rs2_extrinsics extrin = dprofile.get_extrinsics_to(cprofile);
  std::cout << "\nextrinsics of depth camera to color camera: \nrotaion: "
            << std::endl;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      float value = extrin.rotation[3 * i + j];
      std::cout << value << "  ";
    }
    std::cout << std::endl;
  }
  std::cout << std::endl;

  std::cout << "translation: ";
  for (auto value : extrin.translation) std::cout << value << "  ";
  std::cout << std::endl;

  while (1) {
    ///等待一帧数据，默认等待5s
    data = pipe.wait_for_frames();

    rs2::depth_frame depth = data.get_depth_frame();  ///获取深度图像数据
    rs2::video_frame color = data.get_color_frame();  ///获取彩色图像数据
    int color_width = color.get_width();
    int color_height = color.get_height();

    int depth_width = depth.get_width();
    int depth_height = depth.get_height();

    if (!color || !depth) break;  ///如果获取不到数据则退出

    ///将彩色图像和深度图像转换为Opencv格式
    cv::Mat image(cv::Size(color_width, color_height), CV_8UC3,
                  (void*)color.get_data(), cv::Mat::AUTO_STEP);
    cv::Mat depthmat(cv::Size(depth_width, depth_height), CV_16U,
                     (void*)depth.get_data(), cv::Mat::AUTO_STEP);

    ///显示
    cv::imshow("image", image);
    cv::imshow("depth", depthmat);
    cv::waitKey(1);
  }
  return EXIT_SUCCESS;
} catch (const rs2::error& e) {
  ///捕获相机设备的异常
  std::cerr << "RealSense error calling " << e.get_failed_function() << "("
            << e.get_failed_args() << "):\n    " << e.what() << std::endl;
  return EXIT_FAILURE;
} catch (const std::exception& e) {
  std::cerr << "Other error : " << e.what() << std::endl;
  return EXIT_FAILURE;
}
