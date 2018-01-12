#pragma once

#include <opencv2\opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

int getBarcodeWidth(Mat &input, int step, int num);
int getBarcodeWidthLine(Mat &input, int y, String window);
