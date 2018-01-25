#pragma once

#include <opencv2\opencv.hpp>
#include <iostream>
#include "Macros.h"

using namespace std;
using namespace cv;

double detect1D(Mat &input, Mat &output, Mat &draw, int scale);
double detect1DTrapezoid(Mat &input, Mat &output, Mat &draw, int scale);
