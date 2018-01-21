#include "stdafx.h"

#include <opencv2\opencv.hpp>
#include <iostream>
#include "Macros.h"
#include "Util.h"
#include "Width1D.h"
#include "Detect1D.h"

using namespace std;
using namespace cv;

static bool mouseClick = false;
static int mouseY = 0;

static void mouseCb(int event, int x, int y, int flags, void* userdata);

int main(int argc, char** argv) {
	Mat camBgr, camCompress, camGray, camAt, camMoph;
	Mat kd1, ko1, ke1;
	int width;

	VideoCapture cap;
	cap.open(0);

	Mat widthHist = Mat::zeros(Size(100, 200), CV_8U);
	Mat out1D = Mat::zeros(Size(BAR_1D_WIDTH, BAR_1D_HEIGHT), CV_8U);

	namedWindow("camBgr", WINDOW_AUTOSIZE);
	namedWindow("camGray", WINDOW_AUTOSIZE);
	//namedWindow("camSharp", WINDOW_AUTOSIZE);
	namedWindow("camAt", WINDOW_AUTOSIZE);
	namedWindow("camMoph", WINDOW_AUTOSIZE);
	namedWindow("widthHist", WINDOW_AUTOSIZE);
	namedWindow("out1D", WINDOW_AUTOSIZE);

	int atOffset = 3;
	createTrackbar("atOffset", "camBgr", &atOffset, 15, NULL);

	int coefficient = 21;
	createTrackbar("coefficient", "camBgr", &coefficient, 36, NULL);
	int d1 = 7;
	createTrackbar("d1", "camBgr", &d1, 20, NULL);
	int median = 0;
	createTrackbar("median", "camBgr", &median, 5, NULL);
	int o1 = 15;
	createTrackbar("o1", "camBgr", &o1, 60, NULL);
	int e1 = 1;
	createTrackbar("e1", "camBgr", &e1, 20, NULL);

	setMouseCallback("camAt", mouseCb, NULL);

	while (1) {
		// color
		cap >> camBgr;
		camBgr = camBgr(Rect(0, 0, 720, 480));
		resize(camBgr, camBgr, Size(600, 480));
		resize(camBgr, camCompress, Size(WIDTH, HEIGHT));
		// gray
		cvtColor(camCompress, camGray, COLOR_BGR2GRAY);

		PERFORMANCE_START();
		adaptiveThreshold(camGray, camAt, 255, CV_ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, 3, atOffset);
		PERFORMANCE_STOP("adaptive threshold");
		/*
		if (mouseClick) {
			mouseClick = false;
			cout << "line width max: " << getWidth1DLineHist(camAt, widthHist, mouseY, coefficient) << endl;
		}
		PERFORMANCE_START();
		width = getWidth1D(camAt, 4, coefficient);
		PERFORMANCE_STOP("width");
		cout << "window width max: " << width << endl; //
		
		d1 = width + 2;
		setTrackbarPos("d1", "camMoph", d1);
		o1 = 3 * (width + 1);
		setTrackbarPos("o1", "camMoph", o1);
		*/
		kd1 = getStructuringElement(MORPH_RECT, Size(d1, 1));
		PERFORMANCE_START();
		morphologyEx(camAt, camMoph, MORPH_DILATE, kd1);
		PERFORMANCE_STOP("moph dilate");

		//medianBlur(camMoph, camMoph, median * 2 + 1);

		//ko1 = getStructuringElement(MORPH_RECT, Size(o1, o1));
		ko1 = getStructuringElement(MORPH_ELLIPSE, Size(o1, o1));
		PERFORMANCE_START();
		morphologyEx(camMoph, camMoph, MORPH_OPEN, ko1);
		PERFORMANCE_STOP("moph open");

		//ke1 = getStructuringElement(MORPH_RECT, Size(e1 + 1, e1 + 1));
		//morphologyEx(camMoph, camMoph, MORPH_ERODE, ke1);

		//line(camAt, Point(0, mouseY), Point(camAt.cols - 1, mouseY), 128);

		PERFORMANCE_START();
		detect1D(camMoph, camBgr, out1D);
		PERFORMANCE_STOP("detect");

		imshow("camBgr", camBgr);
		imshow("camGray", camGray);
		imshow("camAt", camAt);
		imshow("camMoph", camMoph);
		imshow("widthHist", widthHist);
		imshow("out1D", out1D);

		if (waitKey(1) == 27) {
			break;
		}
	}

	destroyWindow("camBgr");
	destroyWindow("camGray");
	destroyWindow("camAt");
	destroyWindow("camMoph");
	destroyWindow("widthHist");
	destroyWindow("out1D");
	return 0;
}

static void mouseCb(int event, int x, int y, int flags, void* userdata)
{
	if (event == EVENT_LBUTTONDOWN) {
		mouseClick = true;
		mouseY = y;
	}
}
