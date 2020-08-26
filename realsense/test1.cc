#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include<librealsense2/rs.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include<math.h>
 
using namespace std;
using namespace cv;
#define Width 848
#define Height 480 
#define fps 30

double caculate(cv::Mat depth,cv::Rect rect)
{
    vector<cv::Point> point;

}


//获得单个点经过旋转后所在精确坐标
Point2f GetPointAfterRotate(Point2f inputpoint,Point2f center,double angle){
    Point2d preturn;
    preturn.x = (inputpoint.x - center.x)*cos(-angle) - (inputpoint.y - center.y)*sin(-angle)+center.x;
    preturn.y = (inputpoint.x - center.x)*sin(-angle) + (inputpoint.y - center.y)*cos(-angle)+center.y;
    return preturn;
}
Point GetPointAfterRotate(Point inputpoint,Point center,double angle){
    Point preturn;
    preturn.x = (inputpoint.x - center.x)*cos(-1*angle) - (inputpoint.y - center.y)*sin(-1*angle)+center.x;
    preturn.y = (inputpoint.x - center.x)*sin(-1*angle) + (inputpoint.y - center.y)*cos(-1*angle)+center.y;
    return preturn;
}
double getOrientation(vector<Point> &pts, Point2f& pos,Mat& img)
{
    //Construct a buffer used by the pca analysis
    Mat data_pts = Mat(pts.size(), 2, CV_64FC1);
    for (int i = 0; i < data_pts.rows; ++i)
    {
        data_pts.at<double>(i, 0) = pts[i].x;
        data_pts.at<double>(i, 1) = pts[i].y;
    }
    //Perform PCA analysis
    PCA pca_analysis(data_pts, Mat(), CV_PCA_DATA_AS_ROW);
    //Store the position of the object
    pos = Point2f(pca_analysis.mean.at<double>(0, 0),
        pca_analysis.mean.at<double>(0, 1));
    //Store the eigenvalues and eigenvectors
    vector<Point2d> eigen_vecs(2);
    vector<double> eigen_val(2);
    for (int i = 0; i < 2; ++i)
    {
        eigen_vecs[i] = Point2d(pca_analysis.eigenvectors.at<double>(i, 0),
            pca_analysis.eigenvectors.at<double>(i, 1));
        eigen_val[i] = pca_analysis.eigenvalues.at<double>(i,0);
    }
    // Draw the principal components
    //在轮廓/图像中点绘制小圆
    //circle(img, pos, 3, CV_RGB(255, 0, 255), 2);
    //计算出直线，在主要方向上绘制直线
    //line(img, pos, pos + 0.02 * Point2f(eigen_vecs[0].x * eigen_val[0], eigen_vecs[0].y * eigen_val[0]) , CV_RGB(255, 255, 0));
    //line(img, pos, pos + 0.02 * Point2f(eigen_vecs[1].x * eigen_val[1], eigen_vecs[1].y * eigen_val[1]) , CV_RGB(0, 255, 255));
    return atan2(eigen_vecs[0].y, eigen_vecs[0].x);
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
int heig=2000;
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
vector<vector<Point> > find_obstacle(Mat &depth, int thresh = 20, int max_thresh = 255, int area = 500)
{
Mat dep;
depth.copyTo(dep);
mask_depth(depth,depth);
mask_depth(dep, depth, 1000);
//cout<<dep<<endl;
dep.convertTo(dep, CV_8UC1,1.0/1);
// cout<<dep<<endl;
//imshow("color", color);
imshow("depth", dep);
//cv::medianBlur(dep,dep,9);
Mat element = getStructuringElement(MORPH_RECT, Size(10, 10));//核的大小可适当调整
//Mat out;
//进行开操作
morphologyEx(dep, dep, MORPH_OPEN, element);
//dilate(dhc, out, element);
 
//显示效果图
imshow("kaicaozuo", dep);
Mat src_copy = dep.clone();
Mat threshold_output;
vector<vector<Point> > contours;
vector<Vec4i> hierarchy;
RNG rng(12345);

/// 阈值分割
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
if (contourArea(contours[i]) < area) //面积小于area的凸包，可忽略
continue;
result.push_back(hull[i]);
Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
drawContours(drawing, contours, i, color, 1, 8, vector<Vec4i>(), 0, Point());
drawContours(drawing, hull, i, color, 1, 8, vector<Vec4i>(), 0, Point());


//获得轮廓的角度
        Point2f* pos = new Point2f();
        double dOrient =  getOrientation(contours[i], *pos,drawing);
        //转换轮廓,并获得极值
        for (size_t j = 0;j<contours[i].size();j++)
            contours[i][j] = GetPointAfterRotate(contours[i][j],(Point)*pos,dOrient);
        Rect rect = boundingRect(contours[i]);//轮廓最小外接矩形
        RotatedRect rotateRect = RotatedRect((Point2f)rect.tl(),Point2f(rect.br().x,rect.tl().y),(Point2f)rect.br());
        //将角度转换回去并绘图
        Point2f rect_points[4];
        rotateRect.points( rect_points ); 
        for (size_t j = 0;j<4;j++)
            rect_points[j] = GetPointAfterRotate((Point)rect_points[j],(Point)*pos,-dOrient);
        for( size_t j = 0; j < 4; j++ )
            line( drawing, rect_points[j], rect_points[(j+1)%4],Scalar(255,255,0),2);
        //得出结果    
        char cbuf[255];
        double fshort = std::min(rect.width,rect.height);
        double flong  = std::max(rect.width,rect.height);
        cout<<"第%d个轮廓,长度%.2f,宽度%.2f像素"<<i<<flong<<fshort;

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
    cfg.enable_stream(RS2_STREAM_COLOR, Width, Height, RS2_FORMAT_BGR8, fps);//向配置添加所需的流
    cfg.enable_stream(RS2_STREAM_DEPTH, Width, Height, RS2_FORMAT_Z16,fps);
    cfg.enable_stream(RS2_STREAM_INFRARED, 1, Width, Height, RS2_FORMAT_Y8, fps);
	cfg.enable_stream(RS2_STREAM_INFRARED, 2, Width, Height, RS2_FORMAT_Y8, fps);
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
Mat color(Size(Width, Height), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
Mat depth(Size(Width,Height), CV_16U, (void*)depth_frame.get_data(), Mat::AUTO_STEP);

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

vector<cv::Rect> ve_rect;
vector<vector<Point> > result;
result = find_obstacle(depth, 30, 255, 500);
if(result.size()<1) 
{

}
else
{
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
for(int i=0;i<ve_rect.size();i++)
{
    cout<<"object  "<<i<<":"<<ve_rect[i].x<<","<<ve_rect[i].y<<endl;
    double distance=caculate(depth,ve_rect[i]);
}
imshow("color",color);
waitKey(1);
imshow("depth",depth);
waitKey(1);

// if (cvWaitKey(1) == 27)
// break;
}
 
}