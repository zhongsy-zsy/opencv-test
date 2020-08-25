#ifndef _REALSENSE_H_
#define _REALSENSE_H_

#include<librealsense2/rs.hpp>

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

public:
    realsense();
    ~realsense();
    void init();
};

realsense::realsense()
{
    rs2::spatial_filter spat_filter(0.4f,4.0f,2.0f,0);
    rs2::threshold_filter thd_filter(0.15f,4.0f);
    rs2::hole_filling_filter hole_filter(0);
    rs2::colorizer colorizered(0);
}

realsense::~realsense()
{
}

void realsense::init()
{

    rs2::context ctx;
    //获取设备列表
    dev_list=ctx.query_devices();
    if(dev_list.size()==0)
    throw std::runtime_error("No device detected.");

    rs2::device dev=list.front();
    rs2::frameset frames;
    rs2::pipeline pipe(ctx);

    //rs2::decimation_filter dec_filter;
    rs2::spatial_filter spat_filter(0.4f,4.0f,2.0f,0);
    rs2::threshold_filter thd_filter(0.15f,4.0f);
    rs2::hole_filling_filter hole_filter(0);
    rs2::colorizer colorizered;

    //dec_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE,3);
    // spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA,0.4f);
    // spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA,4.0f);
    
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR,width,height,RS2_FORMAT_RGB8,fps);
    cfg.enable_stream(RS2_STREAM_DEPTH,width,height,RS2_FORMAT_Z16,fps);
    cfg.enable_stream(RS2_STREAM_INFRARED, 1, width, height, RS2_FORMAT_Y8, fps);
	cfg.enable_stream(RS2_STREAM_INFRARED, 2, width, height, RS2_FORMAT_Y8, fps);
    pipe.start(cfg);
}






#endif