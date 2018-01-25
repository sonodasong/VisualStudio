#pragma once

#include <opencv2\opencv.hpp>
#include <iostream>
#include "Macros.h"
#include "Util.h"

using namespace std;
using namespace cv;

void detect2D(Mat &input, Mat &output, Mat &draw, int ratio);
