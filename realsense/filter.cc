#include<opencv2/opencv.hpp>
#include<librealsense2/rs.hpp>

using namespace std;
using namespace cv;
#define width 640
#define height 480 
#define fps 30

int main()
{
    rs2::context ctx;
    //获取设备列表
    auto list=ctx.query_devices();
    if(list.size()==0)
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

    Mat latest;
    int flag=0;

    while(true)
    {
        frames=pipe.wait_for_frames();

        rs2::depth_frame depth_frame=frames.get_depth_frame();
        Mat depth(Size(width,height),CV_16UC1,(void*)depth_frame.get_data(),Mat::AUTO_STEP);
        imshow("depth",depth);
        waitKey(1);
        Mat out;
        
        if(flag==0) 
        {
        latest=depth;
        flag=1;
        continue;
        }
        //两张图片的差
        Mat diff=depth-latest;
        std::cout<<"head"<<endl<<diff<<std::endl<<std::endl<<"tail"<<endl;

        latest=depth;
        depth.convertTo(depth,CV_8UC1);
        applyColorMap(depth,out,COLORMAP_JET);
         imshow("depthcolor",out);
         waitKey(1);

        diff.convertTo(diff,CV_8UC1);
        applyColorMap(diff,diff,cv::COLORMAP_JET);
        imshow("diff",diff);
        waitKey(1);
        //滤波处理
        rs2::frame filtered=depth_frame;
        //filtered=dec_filter.process(filtered);
        filtered=spat_filter.process(filtered);
        filtered=thd_filter.process(filtered);
        filtered=hole_filter.process(filtered);
        Mat fitered(Size(width,height),CV_16UC1,(void*)filtered.get_data(),Mat::AUTO_STEP);
        // imshow("filtered",fitered);
        // waitKey(1);
        fitered.convertTo(fitered,CV_8UC1);
        applyColorMap(fitered,fitered,cv::COLORMAP_JET);
        imshow("filtercolor",fitered);

        // Mat draw=depth.clone();
        // Mat draw1=Mat(draw.rows,draw.cols,CV_8UC3);
        // cvtColor(draw,draw1,CV_GRAY2BGR);
        // cv::circle(draw1,Point(width/2,height/2),3,cv::Scalar(0,0,255));
        //cv::Mat depth_copy=depth.clone();
        // cv::Mat out_z;
        //Mat tmp;
        // depth.convertTo(tmp,CV_8UC1);
        // applyColorMap(tmp,depth,cv::COLORMAP_HOT);
        // imshow("raw",depth);
        // waitKey(1);
        // depth.convertTo(depth,CV_8UC1);
        // cv::medianBlur(depth,out_z,3);
        // imshow("medium",out_z);
        // waitKey(1);
        // Mat out_s;
        // //depth_copy.convertTo(depth_copy,CV_8UC1);
        // bilateralFilter(depth,out_s,25,25*2,25/2);
        // imshow("shuangbian",out_s);
        // waitKey(1);
        // //分割
        // Mat out_meanshift;
        // pyrMeanShiftFiltering(out_z,out_meanshift,50,50,2);
        // imshow("mena_shift",out_meanshift);
        // waitKey(1);

        // //背景差分
        // Mat out_diff;
        // absdiff(depth,out_z,out_diff);
        // imshow("diff",out_diff);
        // waitKey(1);

        // float center=depth_frame.get_distance(width/2,height/2);
        // cout<<"center"<<center<<endl;
    }   
}