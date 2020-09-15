/*
 "Copyright [year] <Copyright Owner>"
*/

#ifndef REALSENSE_COMMON_OP_H_
#define REALSENSE_COMMON_OP_H_

#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>

std::vector<double> calculate_threshold(cv::Mat mean_depth,
                                        const std::vector<cv::Mat>& raw_data);

#endif