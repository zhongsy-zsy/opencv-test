#ifndef _REALSENSE_H_
#define _REALSENSE_H_

#include<librealsense2/rs.hpp>
#include<opencv2/opencv.hpp>
#include<iostream>

#define width 848
#define height 480 
#define fps 30

class realsense
{
private:
    /* data */
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

public:
    realsense();
    ~realsense();
    void init();
    void get_depth();
    void imshow_colorized_depth();
    void get_data(void *data);
   
};

void realsense::get_data(void *data)
{
    auto cdata=static_cast<cv::Mat *>(data);
    *cdata=depth_data;
}


realsense::realsense():spat_filter(0.4f,4.0f,2.0f,0),thd_filter(0.15f,4.0f),hole_filter(0),colorizered(0)
{
   
}

realsense::~realsense()
{
}

void realsense::get_depth()
{
    while(1)
    {
         frames=pipe.wait_for_frames();
         rs2::depth_frame depth_frame=frames.get_depth_frame();
         cv::Mat depth(cv::Size(width,height),CV_16UC1,(void*)depth_frame.get_data(),cv::Mat::AUTO_STEP);
         cv::imshow("depth",depth);
         cv::waitKey(1);
         depth.copyTo(depth_data);    
    }
}

void realsense::init()
{

    //rs2::context ctx;
    //获取设备列表
    dev_list=ctx.query_devices();
    if(dev_list.size()==0)
    throw std::runtime_error("No device detected.");

    dev=dev_list.front();
    rs2::pipeline pipe1(ctx);
    pipe=pipe1;
    //rs2::decimation_filter dec_filter;
    // rs2::spatial_filter spat_filter(0.4f,4.0f,2.0f,0);
    // rs2::threshold_filter thd_filter(0.15f,4.0f);
    // rs2::hole_filling_filter hole_filter(0);
    // rs2::colorizer colorizered;

    //dec_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE,3);
    // spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA,0.4f);
    // spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA,4.0f);
    
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR,width,height,RS2_FORMAT_RGB8,fps);
    cfg.enable_stream(RS2_STREAM_DEPTH,width,height,RS2_FORMAT_Z16,fps);
    cfg.enable_stream(RS2_STREAM_INFRARED, 1, width, height, RS2_FORMAT_Y8, fps);
	cfg.enable_stream(RS2_STREAM_INFRARED, 2, width, height, RS2_FORMAT_Y8, fps);
    pipe.start(cfg);
    std::cout<<"device start"<<std::endl;
}

void realsense::imshow_colorized_depth()
{
    frames=pipe.wait_for_frames();
    rs2::frame coloried=frames.get_depth_frame().apply_filter(colorizered);
    cv::Mat color(cv::Size(width,height),CV_8UC3,(void*)coloried.get_data(),cv::Mat::AUTO_STEP);
    cv::imshow("coloried",color);
    cv::waitKey(1);
    
    
}



#endif