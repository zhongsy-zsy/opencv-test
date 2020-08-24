<span style="font-size:14px;">#include <stdlib.h>  
#include <iostream>  
#include <string>  
 
#include <XnCppWrapper.h>  
#include <opencv2/opencv.hpp>  
 
using namespace std;  
using namespace xn;  
using namespace cv;  
 
//---------------------------------【全局变量声明】---------------------------------//
Mat cvBGRImage,       blur_c_out, GaussianBlur_c_out, MedianBlur_c_out, bilateralFilter_c_out; 
Mat cvDepthImage8UC1, blur_g_out, GaussianBlur_g_out, MedianBlur_g_out, bilateralFilter_g_out;  
 
int g_nMeanBlurValue=10;           //均值滤波内核值
int g_nGaussianBlurValue=6;	   //高斯滤波内核值
int g_nMedianBlurValue=10;	   //中值滤波参数值
int g_nBilateralFilterValue=10;	   //双边滤波参数值
//---------------------------------------------------------------------------------//
 
//---------------------------------【全局函数声明】---------------------------------//
static void on_MeanBlur_c(int, void *);			//均值滤波器(彩色)
static void on_GaussianBlur_c(int, void *);		//高斯滤波器(彩色)
static void on_MedianBlur_c(int, void *);		//中值滤波器(彩色)
static void on_BilateralFilter_c(int, void *);	        //双边滤波器(彩色)
 
static void on_MeanBlur_g(int, void *);			//均值滤波器(深度)
static void on_GaussianBlur_g(int, void *);		//高斯滤波器(深度)
static void on_MedianBlur_g(int, void *);		//中值滤波器(深度)
static void on_BilateralFilter_g(int, void *);	        //双边滤波器(深度)
void ShowHelpText();
//---------------------------------------------------------------------------------//
 
void CheckOpenNIError(XnStatus eResult,string sStatus)	//检测错误并返回错误的函数  
{  
	if(eResult != XN_STATUS_OK)  
		cout << sStatus << "Error: " << xnGetStatusString(eResult) << endl;
}  
 
int main()  
{  
	//system("color 4F");	//cmd窗口颜色  
 
	ShowHelpText();	
 
	XnStatus eResult = XN_STATUS_OK;  
	ImageMetaData imageMD;  
	DepthMetaData depthMD;  
 
	Context mContext;	//初始化上下文对象  
	eResult = mContext.Init();  
	CheckOpenNIError(eResult, "Initialize context");  
 
	ImageGenerator mImageGenerator;		//创建彩色图像生产节点  
	eResult = mImageGenerator.Create(mContext);  
	CheckOpenNIError(eResult, "Create image generator");  
 
	DepthGenerator mDepthGenerator;		//创建深度图像生产节点  
	eResult = mDepthGenerator.Create(mContext);  
	CheckOpenNIError(eResult, "Create depth generator");  
 
	XnMapOutputMode mapMode;	//设置图像分辨率  
 	mapMode.nXRes = 640;  
 	mapMode.nYRes = 480; 
	mapMode.nFPS  = 30;  
	eResult = mImageGenerator.SetMapOutputMode(mapMode);  
	eResult = mDepthGenerator.SetMapOutputMode(mapMode);  
 
	mDepthGenerator.GetAlternativeViewPointCap().SetViewPoint(mImageGenerator);//深度图像视角转换成彩色图象视角  
 
	eResult = mContext.StartGeneratingAll();	//开始产生数据  
 
	char key;
	while((key != 27) && !(eResult = mContext.WaitNoneUpdateAll()))  
	{  
		//-------------------------------------------【【【彩色图像处理部分】】】--------------------------------------------//
		//====================【<0>原图-彩色】====================//
		mImageGenerator.GetMetaData(imageMD);  
		Mat cvRGBImage(imageMD.FullYRes(), imageMD.FullXRes(), CV_8UC3, (XnUInt8*) imageMD.Data());
		cvtColor(cvRGBImage, cvBGRImage, CV_RGB2BGR);//整幅图像颜色转换,因为opencv中采用BGR方式排列  
		namedWindow("<0>【原图-彩色】");  
		imshow("<0>【原图-彩色】", cvBGRImage);  
		//=======================================================//
 
		//====================【<1>均值滤波】====================//
		blur_c_out = cvBGRImage.clone();
		namedWindow("<1>均值滤波【效果图-彩色】"); 
		createTrackbar("内核值：", "<1>均值滤波【效果图-彩色】",&g_nMeanBlurValue, 50,on_MeanBlur_c );
		on_MeanBlur_c(g_nMeanBlurValue,0);
		//======================================================//
 
		//====================【<2>高斯滤波】====================//
		GaussianBlur_c_out = cvBGRImage.clone();
		namedWindow("<2>高斯滤波【效果图-彩色】");  
		createTrackbar("内核值：", "<2>高斯滤波【效果图-彩色】",&g_nGaussianBlurValue, 50,on_GaussianBlur_c );
		on_GaussianBlur_c(g_nGaussianBlurValue,0);
		//======================================================//
 
		//====================【<3>中值滤波】====================//
		MedianBlur_c_out = cvBGRImage.clone();
		namedWindow("<3>中值滤波【效果图-彩色】");  
		createTrackbar("参数值：", "<3>中值滤波【效果图-彩色】",&g_nMedianBlurValue, 50,on_MedianBlur_c );
		on_MedianBlur_c(g_nMedianBlurValue,0);
		//======================================================//
 
		//====================【<4>双边滤波】====================//
		bilateralFilter_c_out = cvBGRImage.clone();
		namedWindow("<4>双边滤波【效果图-彩色】"); 
		createTrackbar("参数值：", "<4>双边滤波【效果图-彩色】",&g_nBilateralFilterValue, 50,on_BilateralFilter_c);
		on_BilateralFilter_c(g_nBilateralFilterValue,0);
		//======================================================//
		//---------------------------------------------------------------------------------------------------------//
 
 
		//-------------------------------------------【【【深度图像处理部分】】】--------------------------------------------//
		//====================【<0>原图-深度】====================//
		mDepthGenerator.GetMetaData(depthMD);  
		Mat cvDepthImage16UC1(depthMD.FullYRes(), depthMD.FullXRes(), CV_16UC1, (XnUInt16*) depthMD.Data());    
		cvDepthImage16UC1.convertTo(cvDepthImage8UC1, CV_8UC1, 255.0/(depthMD.ZRes()));	//格式转换  
		namedWindow("<0>【原图-深度】");  
		imshow("<0>【原图-深度】", cvDepthImage8UC1);
		//======================================================//
 
		//====================【<1>均值滤波】====================//
		blur_g_out = cvDepthImage8UC1.clone();
		namedWindow("<1>均值滤波【效果图-深度】"); 
		createTrackbar("内核值：", "<1>均值滤波【效果图-深度】",&g_nMeanBlurValue, 50,on_MeanBlur_g );
		on_MeanBlur_g(g_nMeanBlurValue,0);
		//======================================================//
		
		//====================【<2>高斯滤波】====================//
		GaussianBlur_g_out = cvDepthImage8UC1.clone();
		namedWindow("<2>高斯滤波【效果图-深度】");  
		createTrackbar("内核值：", "<2>高斯滤波【效果图-深度】",&g_nGaussianBlurValue, 50,on_GaussianBlur_g );
		on_GaussianBlur_g(g_nGaussianBlurValue,0);
		//======================================================//
 
		//====================【<3>中值滤波】====================//
		MedianBlur_g_out = cvDepthImage8UC1.clone();
		namedWindow("<3>中值滤波【效果图-深度】"); 
		createTrackbar("参数值：", "<3>中值滤波【效果图-深度】",&g_nMedianBlurValue, 50,on_MedianBlur_g );
		on_MedianBlur_g(g_nMedianBlurValue,0);
		//======================================================//
 
		//====================【<4>双边滤波】====================//
		bilateralFilter_g_out = cvDepthImage8UC1.clone();
		namedWindow("<4>双边滤波【效果图-深度】");  
		createTrackbar("参数值：", "<4>双边滤波【效果图-深度】",&g_nBilateralFilterValue, 50,on_BilateralFilter_g);
		on_BilateralFilter_g(g_nBilateralFilterValue,0);
		//======================================================//
		//----------------------------------------------------------------------------------------------------------//
 
		key = waitKey(20);  
	}  
 
	mContext.StopGeneratingAll();	//停止产生数据  
	mContext.Shutdown();  
	return 0;  
} 
 
//-----------------------------【on_MeanBlur( )函数】------------------------------------
//		均值滤波操作的回调函数
//----------------------------------------------------------------------------------------------
static void on_MeanBlur_c(int, void *)
{
	blur( cvBGRImage, blur_c_out, Size( g_nMeanBlurValue+1, g_nMeanBlurValue+1), Point(-1,-1));
	imshow("<1>均值滤波【效果图-彩色】", blur_c_out);
}
static void on_MeanBlur_g(int, void *)
{
	blur( cvDepthImage8UC1, blur_g_out, Size( g_nMeanBlurValue+1, g_nMeanBlurValue+1), Point(-1,-1));
	imshow("<1>均值滤波【效果图-深度】", blur_g_out);
}
 
//-----------------------------【on_GaussianBlur( )函数】------------------------------------
//		高斯滤波操作的回调函数
//-----------------------------------------------------------------------------------------------
static void on_GaussianBlur_c(int, void *)
{
	GaussianBlur( cvBGRImage, GaussianBlur_c_out, Size( g_nGaussianBlurValue*2+1, g_nGaussianBlurValue*2+1 ), 0, 0);
	imshow("<2>高斯滤波【效果图-彩色】", GaussianBlur_c_out);
}
static void on_GaussianBlur_g(int, void *)
{
	GaussianBlur( cvDepthImage8UC1, GaussianBlur_g_out, Size( g_nGaussianBlurValue*2+1, g_nGaussianBlurValue*2+1 ), 0, 0);
	imshow("<2>高斯滤波【效果图-深度】", GaussianBlur_g_out);
}
 
 
//-----------------------------【on_MedianBlur( )函数】------------------------------------
//		中值滤波操作的回调函数
//-----------------------------------------------------------------------------------------------
static void on_MedianBlur_c(int, void *)
{
	medianBlur ( cvBGRImage, MedianBlur_c_out, g_nMedianBlurValue*2+1 );
	imshow("<3>中值滤波【效果图-彩色】", MedianBlur_c_out);
}
static void on_MedianBlur_g(int, void *)
{
	medianBlur ( cvDepthImage8UC1, MedianBlur_g_out, g_nMedianBlurValue*2+1 );
	imshow("<3>中值滤波【效果图-深度】", MedianBlur_g_out);
}
 
 
//-----------------------------【on_BilateralFilter( )函数】------------------------------------
//		双边滤波操作的回调函数
//-----------------------------------------------------------------------------------------------
static void on_BilateralFilter_c(int, void *)
{
	bilateralFilter ( cvBGRImage, bilateralFilter_c_out, g_nBilateralFilterValue, g_nBilateralFilterValue*2, g_nBilateralFilterValue/2 );
	imshow("<4>双边滤波【效果图-彩色】", bilateralFilter_c_out);
}
static void on_BilateralFilter_g(int, void *)
{
	bilateralFilter ( cvDepthImage8UC1, bilateralFilter_g_out, g_nBilateralFilterValue, g_nBilateralFilterValue*2, g_nBilateralFilterValue/2 );
	imshow("<4>双边滤波【效果图-深度】", bilateralFilter_g_out);
}
 
//-----------------------------------【ShowHelpText( )函数】-----------------------------
//		输出一些帮助信息
//----------------------------------------------------------------------------------------------
void ShowHelpText()
{
	//输出各滤波器的初始内核值
	printf("\n\n\t\t\t均值滤波器的初始内核值：10\n");
	printf("\n\n\t\t\t高斯滤波器的初始内核值：6\n");
	printf("\n\n\t\t\t中值滤波器的初始参数值：10\n");
	printf("\n\n\t\t\t双边滤波器的初始参数值：10\n");
	printf("\n\n  ----------------------------------------------------------------------------\n");
}</span>
