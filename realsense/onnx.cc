#include <opencv2/dnn/dnn.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

int main() {
  std::string Modelfile = "/home/zhongsy/yolov5/yolov5s.onnx";
  std::string ImageName = "/home/zhongsy/Downloads/q2.jpg";

  cv::dnn::Net net = cv::dnn::readNetFromONNX(Modelfile);

  cv::Mat image = cv::imread(ImageName);

  cv::Mat inputblob = cv::dnn::blobFromImage(image, 1.0, cv::Size(640, 640),
                                             cv::Scalar(), false, false);

  cv::cvtColor(image, image, cv::COLOR_BGR2RGB);

  net.setInput(inputblob);

  std::vector<cv::Mat> outs;
  std::vector<std::string> outs_name;
//   net.forward(outs, outs_name);
  outs = net.forward();
}
