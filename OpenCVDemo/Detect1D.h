#pragma once

#include <opencv2\opencv.hpp>

void detect1D(const cv::Mat &input, const cv::Mat &original, cv::Mat &output, int scale);
