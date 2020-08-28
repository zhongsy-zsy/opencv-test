#include <librealsense2/rs.hpp>

// include OpenCV header file
#include <opencv2/opencv.hpp>
#include<time.h>
#include<iostream>

using namespace std;
using namespace cv;

#define width 848
#define height 480 
#define fps 30


void norm_image(cv::Mat& src)
{
    double max_val,min_val;
    int nr=src.rows;
    int nc=src.cols;
    cv::minMaxLoc(src,&min_val,&max_val,NULL,NULL);
    for(int i=0;i<nr;i++)
    {
        for(int j=0;j<nc;j++)
        {
            src.at<uchar>(i,j)=((src.at<uchar>(i,j)-min_val)/(max_val-min_val))*255;

        }
    }
    //return src;
}

//处理一下没有信息的点
void quit_black_block(cv::Mat& image)
{
    int nr=image.rows;
    int nc=image.cols;

    for(int i=0;i<nr;i++)
    {
        for(int j=0;j<nc;j++)
        {
            if(image.at<uchar>(i,j)==0)
            {
                image.at<uchar>(i,j)=255;
            }
        }
    }
}

void mask_depth(Mat &image,Mat& twith,int throld=1000)
{
//这里也可以利用域值滤波
int nr = image.rows; // number of rows 
int nc = image.cols; // number of columns 
for (int i = 0; i<nr; i++)
{
 
    for (int j = 0; j<nc; j++) 
    {
        // if(j<10||j>800||i<10||i>460)
        // {
        //     image.at<ushort>(i,j)=0;
        //     continue;
        // }
        if (image.at<ushort>(i, j)>throld||image.at<ushort>(i,j)<12)
        image.at<ushort>(i, j) = 0;
    }
}

//滤出地面，也可以把边界滤除去
int deltap=20;
int heig=4000;
float z=1;
for(int i=0;i<nr;i++)
{
    for(int j=0;j<nc;j++)
    {
        if(heig-image.at<ushort>(i,j)*z<deltap||heig-image.at<ushort>(i,j)*z<0)
        {
            image.at<ushort>(i,j)=0;
        }
    }
}
}


int main(int argc, char** argv) try
{
    // judge whether devices is exist or not 
	rs2::context ctx;
	auto list = ctx.query_devices(); // Get a snapshot of currently connected devices
	if (list.size() == 0) 
		throw std::runtime_error("No device detected. Is it plugged in?");
	rs2::device dev = list.front();
    
    //
    rs2::frameset frames;
    //Contruct a pipeline which abstracts the device
    rs2::pipeline pipe;//创建一个通信管道//https://baike.so.com/doc/1559953-1649001.html pipeline的解释
    //Create a configuration for configuring the pipeline with a non default profile
	rs2::config cfg;//创建一个以非默认配置的配置用来配置管道
    //Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, fps);//向配置添加所需的流
    cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16,fps);
    cfg.enable_stream(RS2_STREAM_INFRARED, 1, width, height, RS2_FORMAT_Y8, fps);
	cfg.enable_stream(RS2_STREAM_INFRARED, 2, width, height, RS2_FORMAT_Y8, fps);
    cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
    cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
   
    // get depth scale 
    // float depth_scale = get_depth_scale(profile.get_device());

    // start stream 
    pipe.start(cfg);//指示管道使用所请求的配置启动流
    rs2_error *e=0;
    clock_t start,stop;
    double duration;
    
    while(1)
    {
        start=clock();
        frames = pipe.wait_for_frames();//等待所有配置的流生成框架

        // Align to depth 
        // rs2::align align_to_depth(RS2_STREAM_DEPTH);
        // frames = align_to_depth.process(frames);
    
        // // Get imu data
        // if (rs2::motion_frame accel_frame = frames.first_or_default(RS2_STREAM_ACCEL))
        // {
        //     rs2_vector accel_sample = accel_frame.get_motion_data();
        //     std::cout << "Accel:" << accel_sample.x << ", " << accel_sample.y << ", " << accel_sample.z << std::endl;
        // }
        // if (rs2::motion_frame gyro_frame = frames.first_or_default(RS2_STREAM_GYRO))
        // {
        //     rs2_vector gyro_sample = gyro_frame.get_motion_data();
        //     std::cout << "Gyro:" << gyro_sample.x << ", " << gyro_sample.y << ", " << gyro_sample.z << std::endl;
        // }
        
        //Get each frame
        rs2::frame color_frame = frames.get_color_frame();
        rs2::depth_frame depth_frame = frames.get_depth_frame();
        // rs2::video_frame ir_frame_left = frames.get_infrared_frame(1);
        // rs2::video_frame ir_frame_right = frames.get_infrared_frame(2);
        
        
        // Creating OpenCV Matrix from a color image
        Mat color(Size(width, height), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
        Mat depth(Size(width,height), CV_16UC1, (void*)depth_frame.get_data(), Mat::AUTO_STEP);
        cv::Mat depth_copy;
        mask_depth(depth,depth,4000);
        // norm_image(depth);
        depth.convertTo(depth,CV_8UC1,0.63);
        imshow("depth",depth);
        waitKey(1);
        // std::cout<<depth.rows<<"  "<<depth.cols<<"  "<<depth.channels()<<endl;
        depth.copyTo(depth_copy);

        quit_black_block(depth_copy); 
        
        imshow("quit_black",depth_copy);
    
        // waitKey(1);
        // cvtColor(depth_copy,depth_copy,CV_GRAY2RGB);
        // pyrMeanShiftFiltering(depth_copy,depth_copy,50,50,3);
        // imshow("meahshift",depth_copy);
        // waitKey(1);
    }
    return 0;
}

// error

catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}



