#pragma once

#include <opencv2\opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

void performanceStart(void);
void performanceStop(String tag);
void getOctagonKernel(Mat &kernel, int size);
void sharpen1(Mat &input, Mat &output);
void sharpen2(Mat &input, Mat &output);
