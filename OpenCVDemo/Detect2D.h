#pragma once

#include <opencv2\opencv.hpp>

void detect2D(const cv::Mat &input, const cv::Mat &original, cv::Mat &output, int ratio);
