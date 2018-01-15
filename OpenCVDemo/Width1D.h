#pragma once

#include <opencv2\opencv.hpp>
#include <iostream>
#include "Macros.h"

using namespace std;
using namespace cv;

int getWidth1DLineHist(Mat &input, int y, int coefficient, String window);
int getWidth1D(Mat &input, int step, int coefficient);
