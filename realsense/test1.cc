#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include<librealsense2/rs.hpp>
#include "opencv2/imgproc/imgproc.hpp"
 
using namespace std;
using namespace cv;
#define width 848
#define height 480 
#define fps 30


void mask_depth(Mat &image,Mat& twith,int throld=1000)
{
int nr = image.rows; // number of rows 
int nc = image.cols; // number of columns 
for (int i = 0; i<nr; i++)
{
 
for (int j = 0; j<nc; j++) {
if (image.at<ushort>(i, j)>throld)
image.at<ushort>(i, j) = 0;
}
}
 
}
vector<vector<Point> > find_obstacle(Mat &depth, int thresh = 20, int max_thresh = 255, int area = 500)
{
Mat dep;
depth.copyTo(dep);
mask_depth(depth, dep, 1000);
dep.convertTo(dep, CV_8UC1, 1.0 / 16);
//imshow("color", color);
imshow("depth", dep);
Mat element = getStructuringElement(MORPH_RECT, Size(15, 15));//核的大小可适当调整
Mat out;
//进行开操作
morphologyEx(dep, out, MORPH_OPEN, element);
//dilate(dhc, out, element);
 
//显示效果图
imshow("kaicaozuo", out);
Mat src_copy = dep.clone();
Mat threshold_output;
vector<vector<Point> > contours;
vector<Vec4i> hierarchy;
RNG rng(12345);
/// 对图像进行二值化
threshold(dep, threshold_output, thresh, 255, CV_THRESH_BINARY);
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
if (contourArea(contours[i]) < area)//面积小于area的凸包，可忽略
continue;
result.push_back(hull[i]);
Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
drawContours(drawing, contours, i, color, 1, 8, vector<Vec4i>(), 0, Point());
drawContours(drawing, hull, i, color, 1, 8, vector<Vec4i>(), 0, Point());
}
imshow("contours", drawing);
return result;
}


int main(int argc, char* argv[])
{
// Mat dhc;
// Mat dep;
rs2_error *e=0;
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
    // cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
    // cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
   Mat latest;
   int flag=0;
    // get depth scale 
    // float depth_scale = get_depth_scale(profile.get_device());

    // start stream 
    pipe.start(cfg);//指示管道使用所请求的配置启动流
 
while (true)
{
    frames = pipe.wait_for_frames();
//Get RGB-D Images

rs2::frame color_frame = frames.get_color_frame();
rs2::depth_frame depth_frame = frames.get_depth_frame();
Mat color(Size(width, height), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
Mat depth(Size(width,height), CV_16U, (void*)depth_frame.get_data(), Mat::AUTO_STEP);

// if(flag==0) 
// {
// latest=depth;
// flag=1;
// continue;
// }

// //两张图片的差
// Mat diff=depth-latest;
// std::cout<<diff<<std::endl;
// latest=depth;
// imshow("diff",diff);
// waitKey(1);
imshow("depth1",depth);
waitKey(1);


//双边滤波
// Mat out,dst,test;
// depth.convertTo(dst,CV_8U);
// GaussianBlur(dst,test,Size(5,5),0,0);
// imshow("shuangbian",test);
// waitKey(1);
// test.convertTo(out,CV_16U);


vector<vector<Point> > result;
result = find_obstacle(depth, 20, 255, 500);
imshow("color",color);
waitKey(1);
imshow("depth",depth);
waitKey(1);

// if (cvWaitKey(1) == 27)
// break;
}
 
}