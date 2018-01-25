#include "stdafx.h"

#include <opencv2\opencv.hpp>
#include <iostream>
#include "Macros.h"
#include "Util.h"
#include "Detect1D.h"

using namespace std;
using namespace cv;

static int ratioStateMachine(double area);
static int ratioStateMachine1(double area);

int main(int argc, char** argv) {
	Mat camBgr, camCompress, camGray, camAt, camMoph;
	Mat kd1, ko1, ke1;
	int scale = SCALE_MIN;

	VideoCapture cap;
	cap.open(0);

	Mat out1D = Mat::zeros(Size(BAR_1D_WIDTH, BAR_1D_HEIGHT), CV_8U);

	namedWindow("camBgr", WINDOW_AUTOSIZE);
	namedWindow("camGray", WINDOW_AUTOSIZE);
	namedWindow("camAt", WINDOW_AUTOSIZE);
	namedWindow("camMoph", WINDOW_AUTOSIZE);
	namedWindow("out1D", WINDOW_AUTOSIZE);

	int atOffset = 3;
	createTrackbar("atOffset", "camBgr", &atOffset, 15, NULL);

	int d1 = 7;
	createTrackbar("d1", "camBgr", &d1, 20, NULL);
	int o1 = 15;
	createTrackbar("o1", "camBgr", &o1, 60, NULL);

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

		//PERFORMANCE_START();
		adaptiveThreshold(camGray, camAt, 255, CV_ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, 3, atOffset);
		//PERFORMANCE_STOP("adaptive threshold");

		kd1 = getStructuringElement(MORPH_RECT, Size(d1, 1));
		//PERFORMANCE_START();
		morphologyEx(camAt, camMoph, MORPH_DILATE, kd1);
		//PERFORMANCE_STOP("moph dilate");

		//ko1 = getStructuringElement(MORPH_RECT, Size(o1, o1));
		ko1 = getStructuringElement(MORPH_ELLIPSE, Size(o1, o1));
		//PERFORMANCE_START();
		morphologyEx(camMoph, camMoph, MORPH_OPEN, ko1);
		//PERFORMANCE_STOP("moph open");

		PERFORMANCE_START();
		//scale = ratioStateMachine(detect1D(camMoph, out1D, camBgr, scale));
		scale = ratioStateMachine(detect1DTrapezoid(camMoph, out1D, camBgr, scale));
		PERFORMANCE_STOP("detect");


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

static int ratioStateMachine(double area)
{
	// 0: unlock; 1: search; 2: lock
	static int lock = 0;
	static int ratio = SCALE_MIN;
	static int ratioBest = -1;
	static double areaBest = 0;

	if (lock == 0) {
		if (area < 0) {
			if (ratio == SCALE_MAX) {
				ratio = SCALE_MIN;
			} else {
				ratio++;
			}
		} else {
			lock = 1;
			if (ratio == SCALE_MIN) {
				ratioBest = ratio;
				areaBest = area;
				ratio++;
			} else {
				ratioBest = -1;
				areaBest = 0;
				ratio = SCALE_MIN;
			}
		}
	} else if (lock == 1) {
		if (area > areaBest * UPDATE_FACTOR) {
			areaBest = area;
			ratioBest = ratio;
		}
		if (ratio == SCALE_MAX) {
			if (ratioBest == -1) {
				lock = 0;
				ratio = SCALE_MIN;
			} else {
				lock = 2;
				ratio = ratioBest;
			}
		} else {
			ratio++;
		}
	} else if (lock == 2) {
		if (area < 0) {
			lock = 0;
			ratioBest = -1;
			areaBest = 0;
			ratio = SCALE_MIN;
		}
	}
	return ratio;
}

static int ratioStateMachine1(double area)
{
	// 0: unlock; 1: search; 2: lock
	static int lock = 0;
	static int ratio = SCALE_MAX;
	static int ratioBest = -1;
	static double areaBest = 0;

	if (lock == 0) {
		if (area < 0) {
			if (ratio == SCALE_MIN) {
				ratio = SCALE_MAX;
			} else {
				ratio--;
			}
		} else {
			lock = 1;
			if (ratio == SCALE_MAX) {
				ratioBest = ratio;
				areaBest = area;
				ratio--;
			} else {
				ratioBest = -1;
				areaBest = 0;
				ratio = SCALE_MAX;
			}
		}
	} else if (lock == 1) {
		if (area > areaBest / UPDATE_FACTOR) {
			areaBest = area;
			ratioBest = ratio;
		}
		if (ratio == SCALE_MIN) {
			if (ratioBest == -1) {
				lock = 0;
				ratio = SCALE_MAX;
			} else {
				lock = 2;
				ratio = ratioBest;
			}
		} else {
			ratio--;
		}
	} else if (lock == 2) {
		if (area < 0) {
			lock = 0;
			ratioBest = -1;
			areaBest = 0;
			ratio = SCALE_MAX;
		}
	}
	return ratio;
}
