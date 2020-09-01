#include<opencv2/opencv.hpp>
#include<librealsense2/rs.hpp>
#include<fstream>

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
    rs2::hole_filling_filter hole_filter(2);
    rs2::colorizer colorizered;

    //dec_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE,3);
    // spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA,0.4f);
    // spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA,4.0f);
    
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR,width,height,RS2_FORMAT_RGB8,fps);
    cfg.enable_stream(RS2_STREAM_DEPTH,width,height,RS2_FORMAT_Z16,fps);
   
    
    pipe.start(cfg);

    Mat latest;
    int flag=0;
    std::fstream file;
    std::fstream result;

    std::fstream deal_data;
    std::fstream deal_result;

    file.open("raw_data.txt",ios::out|ios::trunc);
    if(file.is_open()){
        cout<<"open sucess"<<endl;
    }
    else{
        cout<<"open failed"<<endl;
    }

    result.open("out_result.txt",ios::out|ios::trunc);
    if(result.is_open()){
        cout<<"open sucess"<<endl;
    }
    else{
        cout<<"open failed"<<endl;
    }

    deal_data.open("out_deal_data.txt",ios::out|ios::trunc);
    if(deal_data.is_open()){
        cout<<"open sucess"<<endl;
    }
    else{
        cout<<"open failed"<<endl;
    }

    deal_result.open("out_deal_result.txt",ios::out|ios::trunc);
    if(deal_result.is_open()){
        cout<<"open sucess"<<endl;
    }
    else{
        cout<<"open failed"<<endl;
    }
    int i=1;
    while(true)
    {
        frames=pipe.wait_for_frames();

        rs2::depth_frame depth_frame=frames.get_depth_frame();
        Mat depth(Size(width,height),CV_16UC1,(void*)depth_frame.get_data(),Mat::AUTO_STEP);

        
        // 去除零点干扰
         for(int i=0;i<depth.rows;i++)
        {
            for(int j=0;j<depth.cols;j++)
            {
                if(depth.at<ushort>(i,j)==0)
                depth.at<ushort>(i,j)=960;

            }
        }

        imshow("depth",depth);
        cout<<depth;
        waitKey(1);
        Mat out;
        depth.copyTo(out);
        
        // if(flag==0) 
        // {
        // latest=depth;
        // flag=1;
        // continue;
        // }
        file<<"head"<<i<<": "<<endl<<depth<<"tail     "<<endl;
        cv::Mat mean,stddev;
        cv::meanStdDev(depth,mean,stddev);
        result<<"head"<<i<<" "<<endl<<"mean: "<<mean<<endl<<"stddev: "<<stddev<<endl<<"tail"<<endl;

        Mat_<short> new1=out;        
        for(int i=0;i<new1.rows;i++)
        {
            for(int j=0;j<new1.cols;j++)
            {
                    new1.at<short>(i,j)-=988;
               
            }
        }
        // cout<<new1;
        deal_data<<"head"<<i<<": "<<endl<<new1<<"tail     "<<endl;

        cv::Mat mean1,stddev1;
        cv::meanStdDev(new1,mean1,stddev1);
        deal_result<<"head"<<i<<" "<<endl<<"mean: "<<mean1<<endl<<"stddev: "<<stddev1<<endl<<"tail"<<endl;
        
        
        i++;




        cv::waitKey(2000);

    }   
}