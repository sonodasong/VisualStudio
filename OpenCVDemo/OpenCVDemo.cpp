#include "stdafx.h"

#include <opencv2\opencv.hpp>
#include <iostream>
#include "Macros.h"
#include "Util.h"
#include "Detect2D.h"

using namespace std;
using namespace cv;

int main(int argc, char** argv) {
	Mat camBgr, camCompress, camGray, camSharp, camAt, camMoph;
	Mat kd1, ko1, ke1;

	VideoCapture cap;
	cap.open(0);

	Mat out1D = Mat::zeros(Size(BAR_2D_WIDTH, BAR_2D_HEIGHT), CV_8U);

	namedWindow("camBgr", WINDOW_AUTOSIZE);
	namedWindow("camGray", WINDOW_AUTOSIZE);
	namedWindow("camAt", WINDOW_AUTOSIZE);
	namedWindow("camMoph", WINDOW_AUTOSIZE);
	namedWindow("out1D", WINDOW_AUTOSIZE);

	int scale = 2;
	createTrackbar("scale", "camBgr", &scale, 6, NULL);
	int atOffset = 15;
	createTrackbar("atOffset", "camBgr", &atOffset, 50, NULL);
	int d1 = 0;
	createTrackbar("d1", "camBgr", &d1, 20, NULL);
	int m1 = 0;
	createTrackbar("m1", "camBgr", &m1, 3, NULL);
	int o1 = 0;
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
		sharpen1(camGray, camSharp);

		//PERFORMANCE_START();
		adaptiveThreshold(camSharp, camAt, 255, CV_ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, 3, atOffset);
		//PERFORMANCE_STOP("adaptive threshold");

		kd1 = getStructuringElement(MORPH_RECT, Size(d1 + 1, d1 + 1));
		//PERFORMANCE_START();
		morphologyEx(camAt, camMoph, MORPH_DILATE, kd1);
		//PERFORMANCE_STOP("moph dilate");

		PERFORMANCE_START();
		medianBlur(camMoph, camMoph, m1 * 2 + 1);
		PERFORMANCE_STOP("median");

		ko1 = getStructuringElement(MORPH_RECT, Size(o1 + 1, o1 + 1));
		//ko1 = getStructuringElement(MORPH_ELLIPSE, Size(o1 + 1, o1 + 1));
		//PERFORMANCE_START();
		morphologyEx(camMoph, camMoph, MORPH_OPEN, ko1);
		//PERFORMANCE_STOP("moph open");

		PERFORMANCE_START();
		detect2D(camMoph, out1D, camBgr, scale);
		PERFORMANCE_STOP("detect 2D");


		imshow("camBgr", camBgr);
		imshow("camGray", camGray);
		imshow("camAt", camAt);
		imshow("camMoph", camMoph);
		imshow("out1D", out1D);

		if (waitKey(1) == 27) {
			break;
		}
	}

	destroyWindow("camBgr");
	destroyWindow("camGray");
	destroyWindow("camAt");
	destroyWindow("camMoph");
	destroyWindow("out1D");
	return 0;
}
