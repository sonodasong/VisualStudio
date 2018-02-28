#include "stdafx.h"

#include "Macros.h"
#include "Detect.h"
#include "Detect1D.h"

using namespace std;
using namespace cv;

static int _scale;

static int findMaxContour(const vector< vector<Point> > &contours);
static int getQuadrilateralStart(const vector<Point> &quadrilateral);
static void getBarcode1D(const Mat &input, Mat &output, const vector<Point> &quadrilateral, int quadrilateralStart);

void detect1D(const Mat &input, const Mat &original, Mat &output, int scale)
{
	vector< vector<Point> > contours;
	_scale = scale;
	findContours(input, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	int maxContour = findMaxContour(contours);
	if (maxContour == -1) return;
	const vector<Point> &minQuadrilateral = getMinQuadrilateral(contours[maxContour]);
	if (minQuadrilateral.size() != 4) return;
	getBarcode1D(original, output, minQuadrilateral, getQuadrilateralStart(minQuadrilateral));
}

static int findMaxContour(const vector< vector<Point> > &contours)
{
	double area = 0;
	int maxContour = -1;
	for (int i = 0; i < contours.size(); i++) {
		double temp = contourArea(contours[i]);
		if (temp > area) {
			area = temp;
			maxContour = i;
		}
	}
	return maxContour;
}

static int getQuadrilateralStart(const vector<Point> &quadrilateral)
{
	float leftMost[2] = { WIDTH, WIDTH };
	int index[2];
	for (int i = 0; i < 4; i++) {
		float x = quadrilateral[i].x;
		if (x < leftMost[0]) {
			leftMost[1] = leftMost[0];
			index[1] = index[0];
			leftMost[0] = x;
			index[0] = i;
		}
		else if (x < leftMost[1]) {
			leftMost[1] = x;
			index[1] = i;
		}
	}
	return quadrilateral[index[0]].y < quadrilateral[index[1]].y ? index[0] : index[1];
}

static void getBarcode1D(const Mat &input, Mat &output, const vector<Point> &quadrilateral, int quadrilateralStart)
{
	vector<Point2f> object(4);
	vector<Point2f> scene(4);
	scene[0] = Point2f(0, 0);
	scene[1] = Point2f(BAR_WIDTH, 0);
	scene[2] = Point2f(BAR_WIDTH, BAR_HEIGHT);
	scene[3] = Point2f(0, BAR_HEIGHT);
	object[0] = quadrilateral[quadrilateralStart] * _scale;
	object[1] = quadrilateral[(quadrilateralStart + 1) % 4] * _scale;
	object[2] = quadrilateral[(quadrilateralStart + 2) % 4] * _scale;
	object[3] = quadrilateral[(quadrilateralStart + 3) % 4] * _scale;
	Mat tran = getPerspectiveTransform(object, scene);
	warpPerspective(input, output, tran, Size(BAR_WIDTH, BAR_HEIGHT));
}
