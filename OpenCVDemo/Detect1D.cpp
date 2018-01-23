#include "stdafx.h"

#include "Detect1D.h"

static vector<vector<Point>> contours;
static int maxContourIndex;
static Point2f minRect[4];
static Point2f minRectMid[4];
static vector<int> originHull;
static vector<int> reduceHull;
static Point2f minQuadrilateral[4];
static int quadrilateralStart;

static double findMaxContour(void);
static void getMinRectMid(void);
static void reduceConvexHull(void);
static bool getMinQuadrilateral(void);
static void getQuadrilateralStart(Point2f *quadrilateral);
static double getQuadrilateralArea(Point2f *quadrilateral);
static void getBarcode1D(Mat &input, Mat &output, Point2f *quadrilateral, int ratio);
static void drawMinRect(Mat &draw, int ratio);
static void drawMinRectMid(Mat &output);
static void drawOriginHull(Mat &draw, int ratio);
static void drawReduceHull(Mat &output);
static void drawMinQuadrilateral(Mat &draw, int ratio);

double detect1D(Mat &input, Mat &draw, Mat &output, int ratio)
{
	vector<Vec4i> hierarchy;
	double area1, area2;
	findContours(input, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	area1 = findMaxContour();
	if (maxContourIndex == -1) return -1;
	//drawContours(draw, contours, maxContourIndex, Scalar(0, 0, 255), 1, 8, hierarchy, 0);
	minAreaRect(Mat(contours[maxContourIndex])).points(minRect);
	getMinRectMid();
	convexHull(Mat(contours[maxContourIndex]), originHull, false); // false is clockwise, why?
	reduceConvexHull();
	if (!getMinQuadrilateral()) return -1;
	area2 = getQuadrilateralArea(minQuadrilateral);
	//if (area1 / area2 < AREA_FACTOR) return -1;
	area1 *= ratio * ratio;
	//if (area1 < MIN_AREA) return -1;
	getQuadrilateralStart(minQuadrilateral);
	getBarcode1D(draw, output, minQuadrilateral, ratio);

	//drawMinRect(draw);
	//drawMinRectMid(draw);
	drawOriginHull(draw, ratio);
	//drawReduceHull(draw);
	drawMinQuadrilateral(draw, ratio);
	cout << area1<< endl;
	return area1;
}

void detect1DRect(Mat &input, Mat &draw, Mat &output, int ratio)
{
	vector<Vec4i> hierarchy;
	findContours(input, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	findMaxContour();
	if (maxContourIndex == -1) return;
	//drawContours(draw, contours, maxContourIndex, Scalar(0, 0, 255), 1, 8, hierarchy, 0);
	minAreaRect(Mat(contours[maxContourIndex])).points(minRect);
	getQuadrilateralStart(minRect);
	getBarcode1D(draw, output, minRect, ratio);

	drawMinRect(draw, ratio);
}

static double findMaxContour(void)
{
	double area = 0;
	double temp = 0;
	maxContourIndex = -1;
	for (int i = 0; i < contours.size(); i++) {
		temp = contourArea(contours[i]);
		if (temp > area) {
			area = temp;
			maxContourIndex = i;
		}
	}
	return area;
}

static void getMinRectMid(void)
{
	for (int i = 0; i < 4; i++) {
		minRectMid[i].x = (minRect[i].x + minRect[(i + 1) % 4].x) / 2;
		minRectMid[i].y = (minRect[i].y + minRect[(i + 1) % 4].y) / 2;
	}
}

static float length(Point p1, Point p2)
{
	float x = p1.x - p2.x;
	float y = p1.y - p2.y;
	return sqrtf(x * x + y * y);
}

static float dot(Point p1, Point p2, Point p3, float len1, float len2)
{
	float result = (p1.x - p2.x) * (p3.x - p2.x) + (p1.y - p2.y) * (p3.y - p2.y);
	return result / len1 / len2;
}

static void reduceConvexHull(void)
{
	Point prev, cur, next;
	float lenPrev, lenCur;
	int size = originHull.size();
	prev = contours[maxContourIndex][originHull[size - 1]];
	cur = contours[maxContourIndex][originHull[0]];
	lenPrev = length(prev, cur);
	reduceHull.clear();
	for (int i = 0; i < originHull.size(); i++) {
		next = contours[maxContourIndex][originHull[(i + 1) % size]];
		lenCur = length(cur, next);
		if (dot(prev, cur, next, lenPrev, lenCur) > cosf(HULL_REDUCE_ANGLE * CV_PI / 180)) {
			reduceHull.push_back(originHull[i]);
		}
		prev = cur;
		cur = next;
		lenPrev = lenCur;
	}
}

static bool validateEdge(int reduceHullIndex, int midIndex)
{
	Point2f p1 = contours[maxContourIndex][reduceHull[reduceHullIndex]];
	Point2f p2 = contours[maxContourIndex][reduceHull[(reduceHullIndex + 1) % reduceHull.size()]];
	Point2f m1 = minRectMid[midIndex];
	Point2f m2 = minRectMid[(midIndex + 2) % 4];
	float det = (m1.x * m2.y) - (m1.y * m2.x);
	if (fabsf(det) < FLT_EPSILON) {
		float k = m1.y / m1.x;
		return (p1.y - p1.x * k) * (p2.y - p2.x * k) < 0 ? true : false;
	} else {
		float a = ((m2.y - m1.y) * p1.x - (m2.x - m1.x) * p1.y) - det;
		float b = ((m2.y - m1.y) * p2.x - (m2.x - m1.x) * p2.y) - det;
		return a * b < 0 ? true : false;
	}
}

static Point2f getIntersection(int p, int q)
{
	Point2f p1 = contours[maxContourIndex][reduceHull[p]];
	Point2f p2 = contours[maxContourIndex][reduceHull[(p + 1) % reduceHull.size()]];
	Point2f q1 = contours[maxContourIndex][reduceHull[q]];
	Point2f q2 = contours[maxContourIndex][reduceHull[(q + 1) % reduceHull.size()]];
	float mat[2][2];
	mat[0][0] = p1.y - p2.y;
	mat[0][1] = p2.x - p1.x;
	mat[1][0] = q1.y - q2.y;
	mat[1][1] = q2.x - q1.x;
	float a = p2.x * p1.y - p1.x * p2.y;
	float b = q2.x * q1.y - q1.x * q2.y;
	float det = mat[0][0] * mat[1][1] - mat[0][1] * mat[1][0];
	float x0 = (mat[1][1] * a - mat[0][1] * b) / det;
	float y0 = (mat[0][0] * b - mat[1][0] * a) / det;
	return Point2f(x0, y0);
}

static bool getMinQuadrilateral(void)
{
	vector<int> validEdge;
	for (int i = 0; i < reduceHull.size(); i++) {
		if (validateEdge(i, 0) || validateEdge(i, 1)) {
			validEdge.push_back(i);
		}
	}
	if (validEdge.size() != 4) return false;
	for (int i = 0; i < 4; i++) {
		minQuadrilateral[i] = getIntersection(validEdge[i], validEdge[(i + 1) % 4]);
	}
	return true;
}

static void getQuadrilateralStart(Point2f *quadrilateral)
{
	float leftMost[2] = {WIDTH, WIDTH};
	int index[2];
	for (int i = 0; i < 4; i++) {
		float x = quadrilateral[i].x;
		if (x < leftMost[0]) {
			leftMost[1] = leftMost[0];
			index[1] = index[0];
			leftMost[0] = x;
			index[0] = i;
		} else if (x < leftMost[1]) {
			leftMost[1] = x;
			index[1] = i;
		}
	}
	quadrilateralStart = quadrilateral[index[0]].y < quadrilateral[index[1]].y ? index[0] : index[1];
}

static double getQuadrilateralArea(Point2f *quadrilateral)
{
	vector<Point> contour;
	for (int i = 0; i < 4; i++) {
		contour.push_back(Point(quadrilateral[i].x, quadrilateral[i].y));
	}
	return contourArea(contour);
}

static void getBarcode1D(Mat &input, Mat &output, Point2f *quadrilateral, int ratio)
{
	vector<Point2f> object(4);
	vector<Point2f> scene(4);
	scene[0] = Point2f(0, 0);
	scene[1] = Point2f(BAR_1D_WIDTH, 0);
	scene[2] = Point2f(BAR_1D_WIDTH, BAR_1D_HEIGHT);
	scene[3] = Point2f(0, BAR_1D_HEIGHT);
	object[0] = quadrilateral[quadrilateralStart] * ratio;
	object[1] = quadrilateral[(quadrilateralStart + 1) % 4] * ratio;
	object[2] = quadrilateral[(quadrilateralStart + 2) % 4] * ratio;
	object[3] = quadrilateral[(quadrilateralStart + 3) % 4] * ratio;
	Mat tran = getPerspectiveTransform(object, scene);
	warpPerspective(input, output, tran, Size(BAR_1D_WIDTH, BAR_1D_HEIGHT));
}

static void drawMinRect(Mat &draw, int ratio)
{
	for (int i = 0; i < 4; i++) {
		line(draw, minRect[i] * ratio, minRect[(i + 1) % 4] * ratio, Scalar(0, 0, 255), 1, LINE_AA);
	}
}

static void drawMinRectMid(Mat &draw)
{
	line(draw, minRectMid[0], minRectMid[2], Scalar(0, 0, 255), 1, LINE_AA);
	line(draw, minRectMid[1], minRectMid[3], Scalar(0, 0, 255), 1, LINE_AA);
}

static void drawOriginHull(Mat &draw, int ratio)
{
	Point prev, cur;
	int size = originHull.size();
	prev = contours[maxContourIndex][originHull[size - 1]] * ratio;
	for (int i = 0; i < size; i++) {
		cur = contours[maxContourIndex][originHull[i]] * ratio;
		line(draw, prev, cur, Scalar(255, 0, 0), 1, LINE_AA);
		rectangle(draw, Point(cur.x - 1, cur.y -1), Point(cur.x + 2, cur.y + 2), Scalar(255, 0, 0), FILLED, LINE_AA);
		prev = cur;
	}
}

static void drawReduceHull(Mat &draw)
{
	Point prev, cur;
	int size = reduceHull.size();
	prev = contours[maxContourIndex][reduceHull[size - 1]];
	for (int i = 0; i < size; i++) {
		cur = contours[maxContourIndex][reduceHull[i]];
		line(draw, prev, cur, Scalar(255, 0, 255), 1, LINE_AA);
		circle(draw, cur, 2, Scalar(255, 0, 255), FILLED, LINE_AA);
		prev = cur;
	}
}

static void drawMinQuadrilateral(Mat &draw, int ratio)
{
	for (int i = 0; i < 4; i++) {
		line(draw, minQuadrilateral[i] * ratio, minQuadrilateral[(i + 1) % 4] * ratio, Scalar(0, 255, 0), 1, LINE_AA);
	}
}
