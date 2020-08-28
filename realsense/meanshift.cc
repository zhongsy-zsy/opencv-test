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

void caculate(cv::Mat depth,cv::Rect rect,vector<double> &res)
{
    int thread=50;
   //设置感兴趣区域
   cv::Mat imageROI(depth,rect);
   //方法一，把零点过滤
   for(int i=0;i<imageROI.rows;i++)
   {
       for(int j=0;j<imageROI.cols;j++)
       {
           if(imageROI.at<ushort>(i,j)==0) imageROI.at<ushort>(i,j)=2000;
       }
   }   
   //cout<<imageROI; 
   imshow("ROI",imageROI);
   waitKey(1);
   double  mindiatance;
   //直接输出最小距离
   minMaxLoc(imageROI,&mindiatance,NULL,NULL,NULL);
   //std::cout<<"最小距离"<<mindiatance<<std::endl; //因为感兴趣区域内会存在零点所以需要弄掉
   res.push_back(mindiatance);

}

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

void quit_landnoise(cv::Mat &image)
{
//滤出地面，也可以把边界滤除去
int nr=image.rows;
int nc=image.cols;
int deltap=50;
int heig=2000;
float z=1;
for(int i=0;i<nr;i++)
    {
    for(int j=0;j<nc;j++)
        {
            if(heig-image.at<ushort>(i,j)*z<deltap)
            {   
                image.at<ushort>(i,j)=0;
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
        if (image.at<ushort>(i, j)>throld||image.at<ushort>(i,j)<15)
        image.at<ushort>(i, j) = 0;
    }
}

    quit_landnoise(image);
}




vector<vector<Point> > find_obstacle(Mat &dep, int thresh = 20, int max_thresh = 255, int area = 500)
{

    Mat threshold_output;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    RNG rng(12345);
    cout<<dep<<endl;

    /// 阈值分割
    // quit_black_block(dep);
    threshold(dep, threshold_output, thresh, 255, CV_THRESH_BINARY);
    imshow("fenge",threshold_output);
    //mask_depth(src, threshold_output);
    /// 寻找轮廓
    findContours(threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
    
    /// 对每个轮廓计算其凸包
    vector<vector<Point> >hull(contours.size());
    vector<vector<Point> > result;
    for (int i = 0; i < contours.size(); i++)
    {
    convexHull(Mat(contours[i]), hull[i], false);
    
    }

    
    /// 绘出轮廓及其凸包
    Mat drawing = Mat::zeros(threshold_output.size(), CV_8UC3);
    for (int i = 0; i< contours.size(); i++)
    {
        if (contourArea(contours[i]) < area) //面积小于area的凸包，可忽略
        continue;
        result.push_back(hull[i]);
        Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        drawContours(drawing, contours, i, color, 1, 8, vector<Vec4i>(), 0, Point());
        drawContours(drawing, hull, i, color, 1, 8, vector<Vec4i>(), 0, Point());


    }
        imshow("contours", drawing);

        return result;
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
    // cfg.enable_stream(RS2_STREAM_INFRARED, 1, width, height, RS2_FORMAT_Y8, fps);
	// cfg.enable_stream(RS2_STREAM_INFRARED, 2, width, height, RS2_FORMAT_Y8, fps);
    // cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
    // cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
   
    // get depth scale 
    // float depth_scale = get_depth_scale(profile.get_device());

    // start stream 
    pipe.start(cfg);//指示管道使用所请求的配置启动流
    rs2_error *e=0;
    clock_t start,stop;
    double duration;
    
    while(1)
    {
        frames = pipe.wait_for_frames();//等待所有配置的流生成框架
        rs2::frame color_frame = frames.get_color_frame();
        rs2::depth_frame depth_frame = frames.get_depth_frame();

        
        
        // Creating OpenCV Matrix from a color image
        Mat color(Size(width, height), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
        Mat depth_raw(Size(width,height), CV_16UC1, (void*)depth_frame.get_data(), Mat::AUTO_STEP);
        cv::Mat depth_copy;
        cv::Mat depth;
        imshow("depth_raw",depth_raw);
        waitKey(1);
        depth_raw.copyTo(depth);
        mask_depth(depth,depth,2000);
        // norm_image(depth);
        depth.convertTo(depth,CV_8UC1,0.1275);
        
        // std::cout<<depth.rows<<"  "<<depth.cols<<"  "<<depth.channels()<<endl;
         depth.copyTo(depth_copy);
        
        quit_black_block(depth_copy); 
        
        imshow("quit_black",depth_copy);

        Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));//核的大小可适当调整
        //Mat out;
        //进行闭操作
        morphologyEx(depth_copy, depth_copy, MORPH_CLOSE, element);
        //dilate(dhc, out, element);

        // 显示效果图
        imshow("bicaozuo", depth_copy);
        vector<cv::Rect> ve_rect;
        vector<vector<Point> > result;
        result = find_obstacle(depth_copy, 147, 255, 500);
        if(result.size()<1) 
        {   
        
        }
        else
        {
            //这边也可以试一下minarearect
            int i=1;
            for(int i=0;i<result.size();i++)
            {
                cv::Rect rect;
                rect=boundingRect(result[i]);
                ve_rect.push_back(rect);
                Mat drawing = Mat::zeros(480,848, CV_8UC3);
                cv::rectangle(drawing,rect.tl(),rect.br(),Scalar(0,0,255));
                imshow("kuang",drawing);
                waitKey(1);
    
            }
        }
        //cout<<depth;
        vector<double> res;
        if(ve_rect.size()>0)
        {
         for(int i=0;i<ve_rect.size();i++)
         {
            cout<<"object  "<<i<<":"<<ve_rect[i].x<<","<<ve_rect[i].y<<endl;
            caculate(depth_raw,ve_rect[i],res);
         }

        
        }
         if(res.size()>0)
         {
          double res1=*min_element(res.begin(),res.end());
          std::cout<<"min"<<res1 <<endl;
         }
    

            // 阈值分割
            // cv::Mat iamge_mask;
            // threshold(depth_copy,iamge_mask,200,255,CV_THRESH_BINARY);
            // cv::imshow("mask",iamge_mask);

            // quit_black_block(depth_copy); 

            // imshow("quit_black",depth_copy);

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



