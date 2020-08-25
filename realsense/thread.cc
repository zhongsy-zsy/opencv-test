#include<librealsense2/rs.hpp>
#include<opencv2/opencv.hpp>
#include"realsense.h"
#include<thread>
#include<functional>

int main()
{
    realsense dev_1;
    dev_1.init();
    cv::Mat filtered;
    std::shared_ptr<std::thread> run_executor_= std::make_shared<std::thread>(
      std::bind(&realsense::get_depth, dev_1));

    while(1)
    {
        // dev_1.imshow_colorized_depth();
        // cv::waitKey(100);
    }
    
}