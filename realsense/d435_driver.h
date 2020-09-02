#ifndef _D435_DRIVER_H_
#define _D435_DRIVER_H_

#include<librealsense2/rs.hpp>
#include<opencv2/opencv.hpp>
#include<vector>
#include<thread>


namespace {
    const int Width=640;
    const int Height=480;
    const int fps=30; 

}//namespace D435

class D435 
{
public:
    
    D435(){};
    ~D435(){};

    // 实现AbstractDriver的接口
    void Init();
    void GetData(void *data) ;
private:
    //自定义接口
    void HandleFeedbackData();
    void get_depth();
    void handle_depth();
    void quit_black_block(cv::Mat& image);
    void mask_depth(cv::Mat &image, int throld=3000);
    void find_obstacle(cv::Mat &depth, int thresh = 200, int max_thresh = 255, int area = 500);
    void calculate_mindistance();

    std::shared_ptr<std::thread> run_executor_;
    rs2::context ctx;
    rs2::frameset frames;
    rs2::pipeline pipe;
    rs2::spatial_filter spat_filter;
    rs2::threshold_filter thd_filter;
    rs2::hole_filling_filter hole_filter;
    rs2::colorizer colorizered; 
    rs2::device_list dev_list;  
    rs2::device dev;
    cv::Mat depth_data; 
    double min_distance;  
    std::vector<std::vector<cv::Point> > result; //存放凸包


};










#endif