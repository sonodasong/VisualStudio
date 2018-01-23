#pragma once

#include <opencv2\opencv.hpp>
#include <iostream>
#include "Macros.h"

using namespace std;
using namespace cv;

double detect1D(Mat &input, Mat &draw, Mat &output, int ratio);
void detect1DRect(Mat &input, Mat &draw, Mat &output, int ratio);
