#include "stdafx.h"

#include <opencv2\opencv.hpp>
#include <iostream>
#include "Macros.h"
#include "Performance.h"
#include "Width1D.h"

using namespace std;
using namespace cv;

static bool mouseClick = false;
static int mouseY = 0;

static void mouseCb(int event, int x, int y, int flags, void* userdata);

int main(int argc, char** argv) {
	Mat camBgr, camGray, camBlur, camSharp, camAt, camMoph;
	Mat kd1, ke1, kd2;
	int width;

	VideoCapture cap;
	cap.open(0);

	namedWindow("camBgr", WINDOW_AUTOSIZE);
	namedWindow("camGray", WINDOW_AUTOSIZE);
	namedWindow("camSharp", WINDOW_AUTOSIZE);
	namedWindow("camAt", WINDOW_AUTOSIZE);
	namedWindow("camMoph", WINDOW_AUTOSIZE);
	namedWindow("widthHist", WINDOW_AUTOSIZE);

	int blurSize = 1;
	createTrackbar("blurSize", "camGray", &blurSize, 3, NULL);

	int atOffset = 2;
	createTrackbar("atOffset", "camAt", &atOffset, 100, NULL);
	int atSize = 0;
	createTrackbar("atSize", "camAt", &atSize, 3, NULL);

	int coefficient = 21;
	createTrackbar("coefficient", "camMoph", &coefficient, 36, NULL);
	int d1 = 0;
	createTrackbar("d1", "camMoph", &d1, 20, NULL);
	int m1 = 1;
	createTrackbar("m1", "camMoph", &m1, 5, NULL);
	int e1 = 0;
	createTrackbar("e1", "camMoph", &e1, 50, NULL);

	setMouseCallback("camAt", mouseCb, NULL);

	while (1) {
		// color
		cap >> camBgr;
		// gray
		resize(camBgr, camBgr, Size(WIDTH, HEIGHT));
		cvtColor(camBgr, camGray, COLOR_BGR2GRAY);
		blur(camGray, camGray, Size(blurSize * 2 + 1, blurSize * 2 + 1));
		// sharp
		GaussianBlur(camGray, camBlur, Size(0, 0), 3);
		addWeighted(camGray, 1.5, camBlur, -0.5, 0, camSharp);

		PERFORMANCE_START();
		adaptiveThreshold(camGray, camAt, 255, CV_ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, atSize * 2 + 3, atOffset);
		camAt = Scalar::all(255) - camAt;
		PERFORMANCE_STOP("adaptive threshold");

		if (mouseClick) {
			mouseClick = false;
			cout << "line width max: " << getWidth1DLineHist(camAt, mouseY, coefficient, "widthHist") << endl;
		}
		PERFORMANCE_START();
		width = getWidth1D(camAt, 4, coefficient);
		PERFORMANCE_STOP("width");
		//cout << "window width max: " << width << endl; //
		
		d1 = width + 1;
		setTrackbarPos("d1", "camMoph", d1);
		e1 = 3 * d1;
		setTrackbarPos("e1", "camMoph", e1);

		kd1 = getStructuringElement(MORPH_RECT, Size(d1 + 1, 1));
		PERFORMANCE_START();
		morphologyEx(camAt, camMoph, MORPH_DILATE, kd1);
		PERFORMANCE_STOP("moph");

		medianBlur(camMoph, camMoph, m1 * 2 + 1);

		ke1 = getStructuringElement(MORPH_RECT, Size(e1 + 1, e1 + 1));
		morphologyEx(camMoph, camMoph, MORPH_OPEN, ke1);

		line(camAt, Point(0, mouseY), Point(camAt.cols - 1, mouseY), 128);

		imshow("camBgr", camBgr);
		imshow("camGray", camGray);
		imshow("camSharp", camSharp);
		imshow("camAt", camAt);
		imshow("camMoph", camMoph);

		if (waitKey(1) == 27) {
			break;
		}
	}

	destroyWindow("camBgr");
	destroyWindow("camGray");
	destroyWindow("camSharp");
	destroyWindow("camAt");
	destroyWindow("camMoph");
	destroyWindow("widthHist");
	return 0;
}

static void mouseCb(int event, int x, int y, int flags, void* userdata)
{
	if (event == EVENT_LBUTTONDOWN) {
		mouseClick = true;
		mouseY = y;
	}
}
