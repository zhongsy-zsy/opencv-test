#include"d435_driver.h"


void D435::Init(){

    //rs2::context ctx;
    //获取设备列表
    dev_list=ctx.query_devices();
    if(dev_list.size()==0)
    {
        // ROS_ERROR("D435 not detected.");
        
    }
    dev=dev_list.front();
    rs2::pipeline pipe1(ctx);
    pipe=pipe1;
    //启动滤波器
    //rs2::decimation_filter dec_filter;
    // rs2::spatial_filter spat_filter(0.4f,4.0f,2.0f,0);
    // rs2::threshold_filter thd_filter(0.15f,4.0f);
    // rs2::hole_filling_filter hole_filter(0);
    // rs2::colorizer colorizered;
    //dec_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE,3);
    // spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA,0.4f);
    // spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA,4.0f);
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR,Width,Height,RS2_FORMAT_RGB8,fps);
    cfg.enable_stream(RS2_STREAM_DEPTH,Width,Height,RS2_FORMAT_Z16,fps);
    cfg.enable_stream(RS2_STREAM_INFRARED, 1, Width, Height, RS2_FORMAT_Y8, fps);
	cfg.enable_stream(RS2_STREAM_INFRARED, 2, Width, Height, RS2_FORMAT_Y8, fps);
    pipe.start(cfg);
    std::cout<<"device start"<<std::endl;
    // AbstractDriver::SetState(DriverState::READY);
    run_executor_=std::make_shared<std::thread>(
        std::bind(&D435::HandleFeedbackData,this));
}

void D435::GetData(void* data)
{
    auto cdata=static_cast<cv::Mat *>(data);
    *cdata=depth_data;
}

void D435::get_depth()
{
     frames=pipe.wait_for_frames();
     rs2::depth_frame depth_frame=frames.get_depth_frame();
     cv::Mat depth(cv::Size(Width,Height),CV_16UC1,(void*)depth_frame.get_data(),cv::Mat::AUTO_STEP);
     cv::imshow("depth",depth);
     cv::waitKey(1);
     depth.copyTo(depth_data);    
}

void D435::mask_depth(cv::Mat &image, int throld)
{
    //这里也可以利用域值滤波
    int nr = image.rows; // number of rows 
    int nc = image.cols; // number of columns 
    for (int i = 0; i<nr; i++)
    {
 
        for (int j = 0; j<nc; j++) 
        {
            // if (image.at<ushort>(i, j)>throld||image.at<ushort>(i,j)<12)
          
            if (image.at<ushort>(i, j) > throld || i > 455 || image.at<ushort>(i, j) < 50){
                image.at<ushort>(i, j) = 0;
            }
        }
        
    }
    

    //滤除地面，也可以考虑把边界滤除去
    // int deltap=20;
    // int heig=3000;
    // float z=1;
    // for(int i=0;i<nr;i++)
    // {
    //     for(int j=0;j<nc;j++)
    //     {
    //         if(heig-image.at<ushort>(i,j)*z<deltap||heig-image.at<ushort>(i,j)*z<0)
    //         {
    //             image.at<ushort>(i,j)=0;
    //         }
    //     }
    // }
}

void  D435::find_obstacle(cv::Mat &depth, int thresh, int max_thresh, int area)
{
    cv::Mat dep;
    dep=depth.clone();
    cv::Mat threshold_output;
    std::vector<std::vector<cv::Point> > contours;

    std::vector<cv::Vec4i> hierarchy;
    cv::RNG rng(12345);
    /// 阈值分割
    threshold(dep, threshold_output, thresh, 255, CV_THRESH_BINARY);
    //mask_depth(src, threshold_output);
    /// 寻找轮廓
    findContours(threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));  
    /// 对每个轮廓计算其凸包
    std::vector<std::vector<cv::Point> >hull(contours.size());
    for (int i = 0; i < contours.size(); i++)
    {
    convexHull(cv::Mat(contours[i]), hull[i], false);
    
    }

    
    /// 绘出轮廓及其凸包
    cv::Mat drawing = cv::Mat::zeros(threshold_output.size(), CV_8UC3);
    for (int i = 0; i< contours.size(); i++)
    {
    if (contourArea(contours[i]) < area) //面积小于area的凸包，可忽略
    continue;
    result.push_back(hull[i]);
    cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
    drawContours(drawing, contours, i, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
    drawContours(drawing, hull, i, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());

    }
    cv::imshow("contours", drawing);

   

}

void D435::quit_black_block(cv::Mat& image)
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

void D435::calculate_mindistance()
{
    std::vector<cv::Rect> ve_rect;

    if(result.empty())
    {
        // 设置避障等级为0
    }
    else
    {
        for(int i=0;i<result.size();i++)
        {
            cv::Rect rect;
            rect=cv::boundingRect(result[i]);
            ve_rect.push_back(rect);
        }
    
        std::vector<double> res;
        if(!ve_rect.empty())
        {
            for(int i=0;i<ve_rect.size();i++)
            {
                cv::Mat imageROI(depth_data,ve_rect[i]);
                // 过滤零点
                for(int i=0;i<imageROI.rows;i++)
                {
                    for(int j=0;j<imageROI.cols;j++)
                    {
                        if(imageROI.at<ushort>(i,j)==0)
                        {
                            imageROI.at<ushort>(i,j)=3000;
                        }
                    }
                }
                double min_dis;
                cv::Point min_point;
                std::vector<int> tmp;

                // for(int i=0;i<3;i++)
                while(tmp.size()<=3)
                {
                    cv::minMaxLoc(imageROI,&min_dis,NULL,&min_point,NULL);
                    // std::cout<<min_dis<<std::endl;
                    // 优化获取的是前面3个最小点的平均值
                    if(tmp.size()==0){
                        tmp.push_back(min_dis);
                        imageROI.at<ushort>(min_point)=3000;
                    }
                    else{
                        if(std::fabs(min_dis-tmp.back())<100)
                        {
                            tmp.push_back(min_dis);
                            imageROI.at<ushort>(min_point)=3000;
                        }
                        else{
                            // 是噪声点
                            tmp.pop_back();
                        }
                    }
                    // std::cout<<tmp.size()<<std::endl;
                }
                min_dis=0;
                for(auto average:tmp)
                {
                    min_dis+=average;
                }
                res.push_back(min_dis/3.0);       
            }

        
        if(!res.empty())

        {

            min_distance=*std::min_element(res.begin(),res.end());
            std::cout<<min_distance<<std::endl;
        }

        }
    }
    result.clear();
    
    
}

void D435::handle_depth()
{
    cv::Mat data;
    data=depth_data.clone();
    mask_depth(data,3000);
    data.convertTo(data,CV_8UC1,0.085);
    cv::imshow("depth_raw",data);
    quit_black_block(data);
    cv::Mat element=cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,3));//闭操作核的大小
    cv::morphologyEx(data,data,cv::MORPH_CLOSE,element);//闭操作
     cv::imshow("close",data);
    find_obstacle(data,147,255,500);
    calculate_mindistance();

}

void D435::HandleFeedbackData()
{
    while(1)
    {
        get_depth();
        handle_depth();

    }
}



