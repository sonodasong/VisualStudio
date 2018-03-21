#include "stdafx.h"

#include <opencv2\opencv.hpp>
#include <iostream>
#include "Macros.h"
#include "Util.h"
#include "Detect1D.h"
#include "Detect2D.h"

using namespace std;
using namespace cv;

void xySobel(const Mat &src, Mat &dst)
{
	Mat x, y;
	Sobel(src, x, CV_16S, 1, 0, -1);
	Sobel(src, y, CV_16S, 0, 1, -1);
	x = abs(x);
	y = abs(y);
	subtract(x, y, dst);
	dst.convertTo(dst, CV_8U);
	threshold(dst, dst, 254, 255, THRESH_BINARY);
}

int main(int argc, char** argv) {
	Mat camBgr, camCompress, camGray, camSharp, camAt, camMoph;
	Mat kd1, ko1, ke1;

	VideoCapture cap;
	cap.open(0);

	Mat out1D = Mat::zeros(Size(BAR_WIDTH, BAR_HEIGHT), CV_8U);
	Mat out2D = Mat::zeros(Size(BAR_WIDTH, BAR_HEIGHT), CV_8U);

	namedWindow("camBgr", WINDOW_AUTOSIZE);
	namedWindow("camGray", WINDOW_AUTOSIZE);
	namedWindow("camAt", WINDOW_AUTOSIZE);
	namedWindow("camMoph", WINDOW_AUTOSIZE);
	namedWindow("out1D", WINDOW_AUTOSIZE);
	namedWindow("out2D", WINDOW_AUTOSIZE);

	int scale = 2;
	createTrackbar("scale", "camBgr", &scale, 6, NULL);
	int atOffset = 12;
	createTrackbar("atOffset", "camBgr", &atOffset, 50, NULL);
	int d1 = 5;
	createTrackbar("d1", "camBgr", &d1, 20, NULL);
	int o1 = 5;
	createTrackbar("o1", "camBgr", &o1, 3, NULL);

	while (1) {
		// color
#ifdef ANALOG_CAMERA
		cap >> camBgr;
		camBgr = camBgr(Rect(0, 0, 720, 480));
		resize(camBgr, camBgr, Size(WIDTH, HEIGHT));
		resize(camBgr, camCompress, Size(WIDTH / scale, HEIGHT / scale));
#else
		cap >> camBgr;
		resize(camBgr, camCompress, Size(WIDTH / scale, HEIGHT / scale));
#endif
		// gray
		cvtColor(camCompress, camGray, COLOR_BGR2GRAY);
		//sharpen1(camGray, camSharp);

		//PERFORMANCE_START();
		//xySobel(camGray, camAt);
		adaptiveThreshold(camGray, camAt, 255, CV_ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, 3, atOffset);
		//PERFORMANCE_STOP("adaptive threshold");

		//PERFORMANCE_START();
		detect2D(camAt, camBgr, out2D, scale);
		//PERFORMANCE_STOP("detect 2D");

		kd1 = getStructuringElement(MORPH_RECT, Size(d1 + 1, 1));
		//PERFORMANCE_START();
		morphologyEx(camAt, camMoph, MORPH_DILATE, kd1);
		//PERFORMANCE_STOP("moph dilate");

		ko1 = getStructuringElement(MORPH_RECT, Size(1, o1 + 1));
		//ko1 = getStructuringElement(MORPH_ELLIPSE, Size(o1 + 1, o1 + 1));
		//PERFORMANCE_START();
		morphologyEx(camMoph, camMoph, MORPH_OPEN, ko1);
		//PERFORMANCE_STOP("moph open");

		//PERFORMANCE_START();
		detect1D(camMoph, camBgr, out1D, scale);
		//PERFORMANCE_STOP("detect 1D");


		imshow("camBgr", camBgr);
		imshow("camGray", camGray);
		imshow("camAt", camAt);
		imshow("camMoph", camMoph);
		imshow("out1D", out1D);
		imshow("out2D", out2D);

		if (waitKey(1) == 27) {
			break;
		}
	}

	destroyWindow("camBgr");
	destroyWindow("camGray");
	destroyWindow("camAt");
	destroyWindow("camMoph");
	destroyWindow("out1D");
	destroyWindow("out2D");
	return 0;
}
