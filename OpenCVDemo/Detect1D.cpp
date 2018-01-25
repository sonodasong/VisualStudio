#include "stdafx.h"

#include "Detect1D.h"

static Mat _draw;
static int _scale;
static vector<vector<Point>> contours;

static int findMaxContour(double &area);
static vector<Point> getMinQuadrilateral(int index);
static int getQuadrilateralStart(vector<Point> &quadrilateral);
static vector<Point> getMinTrapezoid(vector<Point> &minQuadrilateral, int start, int type);
static void getBarcode1D(Mat &input, Mat &output, vector<Point> &quadrilateral, int quadrilateralStart);
static void drawQuadrilateral(vector<Point> &quadrilateral, Scalar color);
static void drawMinRectMid(vector<Point> &minRectMid);
static void drawConvexHull(int index, vector<int> &hull, Scalar color);

double detect1D(Mat &input, Mat &output, Mat &draw, int scale)
{
	vector<Vec4i> hierarchy;
	double barcodeArea;
	_draw = draw;
	_scale = scale;
	findContours(input, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	int maxContour = findMaxContour(barcodeArea);
	if (maxContour == -1) return -1;
	//drawContours(draw, contours, maxContour, RED, 1, 8, hierarchy, 0); //
	vector<Point> &minQuadrilateral = getMinQuadrilateral(maxContour);
	if (minQuadrilateral.size() != 4) return -1;
	if (barcodeArea / contourArea(minQuadrilateral) < AREA_RATIO_THRESHOLD) return -1;
	barcodeArea *= scale * scale;
	if (barcodeArea < MIN_AREA) return -1;
	getBarcode1D(draw, output, minQuadrilateral, getQuadrilateralStart(minQuadrilateral));
	drawQuadrilateral(minQuadrilateral, GREEN);
	cout << barcodeArea << endl; //
	return barcodeArea;
}

double detect1DTrapezoid(Mat &input, Mat &output, Mat &draw, int scale)
{
	vector<Vec4i> hierarchy;
	double barcodeArea;
	_draw = draw;
	_scale = scale;
	findContours(input, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	int maxContour = findMaxContour(barcodeArea);
	if (maxContour == -1) return -1;
	//drawContours(draw, contours, maxContour, RED, 1, 8, hierarchy, 0); //
	vector<Point> &minQuadrilateral = getMinQuadrilateral(maxContour);
	if (minQuadrilateral.size() != 4) return -1;
	if (barcodeArea / contourArea(minQuadrilateral) < AREA_RATIO_THRESHOLD) return -1;
	barcodeArea *= scale * scale;
	if (barcodeArea < MIN_AREA) return -1;
	vector<Point> &minTrapezoid = getMinTrapezoid(minQuadrilateral, getQuadrilateralStart(minQuadrilateral), 0);
	getBarcode1D(draw, output, minTrapezoid, 0);
	drawQuadrilateral(minTrapezoid, BLUE);
	cout << barcodeArea << endl; //
	return barcodeArea;
}

static int findMaxContour(double &area)
{
	int maxContour = -1;
	area = 0;
	for (int i = 0; i < contours.size(); i++) {
		double temp = contourArea(contours[i]);
		if (temp > area) {
			area = temp;
			maxContour = i;
		}
	}
	return maxContour;
}

static vector<Point> getMinRect(int index)
{
	vector<Point> minRect;
	Point2f temp[4];
	minAreaRect(Mat(contours[index])).points(temp);
	for (int i = 0; i < 4; i++) {
		minRect.push_back(Point(temp[i].x, temp[i].y));
	}
	return minRect;
}

static vector<Point> getMinRectMid(vector<Point> &minRect)
{
	vector<Point> minRectMid;
	for (int i = 0; i < 4; i++) {
		int next = (i + 1) % 4;
		minRectMid.push_back(Point((minRect[i].x + minRect[next].x) / 2, (minRect[i].y + minRect[next].y) / 2));
	}
	return minRectMid;
}

static int len2(Point p1, Point p2)
{
	int x = p1.x - p2.x;
	int y = p1.y - p2.y;
	return x * x + y * y;
}

static int dot(Point p1, Point p2, Point p3, Point p4)
{
	return (p2.x - p1.x) * (p4.x - p3.x) + (p2.y - p1.y) * (p4.y - p3.y);
}

static vector<int> reduceConvexHull(int index, vector<int> &originHull)
{
	vector<int> reduceHull;
	int size = originHull.size();
	Point prev = contours[index][originHull[size - 1]];
	Point cur = contours[index][originHull[0]];
	Point next;
	int lenPrev = len2(prev, cur);
	int lenCur;
	for (int i = 0; i < originHull.size(); i++) {
		next = contours[index][originHull[(i + 1) % size]];
		lenCur = len2(cur, next);
		int _dot = dot(cur, prev, cur, next);
		if (_dot * _dot < lenPrev * lenCur * COS_170 * COS_170) {
			reduceHull.push_back(originHull[i]);
		}
		prev = cur;
		cur = next;
		lenPrev = lenCur;
	}
	return reduceHull;
}

static bool validateEdge(int index, vector<int> &hull, int hullIndex, vector<Point> &mid, int midIndex, int det)
{
	Point p1 = contours[index][hull[hullIndex]];
	Point p2 = contours[index][hull[(hullIndex + 1) % hull.size()]];
	Point m1 = mid[midIndex];
	Point m2 = mid[midIndex + 2];
	int sign1, sign2;
	if (det == 0) {
		sign1 = p1.y * m1.x - p1.x * m1.y;
		sign2 = p2.y * m1.x - p2.x * m1.y;
	}
	else {
		sign1 = ((m2.y - m1.y) * p1.x - (m2.x - m1.x) * p1.y) - det;
		sign2 = ((m2.y - m1.y) * p2.x - (m2.x - m1.x) * p2.y) - det;
	}
	return (sign1 < 0 && sign2 >= 0) || (sign2 < 0 && sign1 >= 0) ? true : false;
}

// e[0] * x + e[1] * y + e[2] = 0;
static vector<int> getLineEquation(Point p1, Point p2)
{
	vector<int> e;
	e.push_back(p1.y - p2.y);
	e.push_back(p2.x - p1.x);
	e.push_back(p1.x * p2.y - p2.x * p1.y);
	return e;
}

static vector<int> getParallelLineEquation(vector<int> &line, Point p)
{
	vector<int> e;
	e.push_back(line[0]);
	e.push_back(line[1]);
	e.push_back(-line[0] * p.x - line[1] * p.y);
	return e;
}

static Point getIntersection(vector<int> &e1, vector<int> &e2)
{
	int det = e1[0] * e2[1] - e1[1] * e2[0];
	if (det == 0) return Point(-1, -1);
	int x = (e1[1] * e2[2] - e2[1] * e1[2]) / det;
	int y = (e2[0] * e1[2] - e1[0] * e2[2]) / det;
	return Point(x, y);
}

static vector<Point> getMinQuadrilateral(int index)
{
	vector<Point> minQuadrilateral;
	vector<int> originHull;
	vector<int> validEdge;
	int i;
	int activeMid;
	vector<Point> minRectMid = getMinRectMid(getMinRect(index));
	convexHull(Mat(contours[index]), originHull, false); // false is clockwise, why?
	vector<int> &reduceHull = reduceConvexHull(index, originHull);
	//vector<int> &reduceHull = originHull;
	if (reduceHull.size() < 4) return minQuadrilateral;
	//drawConvexHull(index, reduceHull, BLUE); //
	int size = reduceHull.size();
	int det[2] = {
		minRectMid[0].x * minRectMid[2].y - minRectMid[2].x * minRectMid[0].y,
		minRectMid[1].x * minRectMid[3].y - minRectMid[3].x * minRectMid[1].y
	};
	for (i = 0; i < size; i++) {
		if (validateEdge(index, reduceHull, i, minRectMid, 0, det[0])) {
			validEdge.push_back(i);
			activeMid = 1;
			break;
		}
		else if (validateEdge(index, reduceHull, i, minRectMid, 1, det[1])) {
			validEdge.push_back(i);
			activeMid = 0;
			break;
		}
	}
	for (i++; i < size; i++) {
		if (validateEdge(index, reduceHull, i, minRectMid, activeMid, det[activeMid])) {
			validEdge.push_back(i);
			activeMid = 1 - activeMid;
		}
	}
	if (validEdge.size() != 4) return minQuadrilateral;
	for (i = 0; i < 4; i++) {
		vector<int> &e1 = getLineEquation(contours[index][reduceHull[validEdge[i]]], contours[index][reduceHull[(validEdge[i] + 1) % size]]);
		vector<int> &e2 = getLineEquation(contours[index][reduceHull[validEdge[(i + 1) % 4]]], contours[index][reduceHull[(validEdge[(i + 1) % 4] + 1) % size]]);
		Point temp = getIntersection(e1, e2);
		if (temp.x < 0) return minQuadrilateral;
		minQuadrilateral.push_back(temp);
	}
	return minQuadrilateral;
}

static int getQuadrilateralStart(vector<Point> &quadrilateral)
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
	return quadrilateral[index[0]].y < quadrilateral[index[1]].y ? index[0] : index[1];
}

// 0: min; 1: mid; 2: max
static vector<Point> getMinTrapezoid(vector<Point> &minQuadrilateral, int start, int type)
{
	vector<Point> minTrapezoid;
	vector<vector<int>> e;
	vector<int> &base = getLineEquation(minQuadrilateral[start], minQuadrilateral[(start + 1) % 4]);
	e.push_back(getParallelLineEquation(base, minQuadrilateral[(start + 2) % 4]));
	e.push_back(getParallelLineEquation(base, (minQuadrilateral[(start + 2) % 4] + minQuadrilateral[(start + 3) % 4]) / 2));
	e.push_back(getParallelLineEquation(base, minQuadrilateral[(start + 3) % 4]));
	if (abs(e[0][2] - base[2]) > abs(e[2][2] - base[2])) {
		type = 2 - type;
	}
	minTrapezoid.push_back(minQuadrilateral[start]);
	minTrapezoid.push_back(minQuadrilateral[(start + 1) % 4]);
	minTrapezoid.push_back(getIntersection(getLineEquation(minQuadrilateral[(start + 1) % 4], minQuadrilateral[(start + 2) % 4]), e[type]));
	minTrapezoid.push_back(getIntersection(getLineEquation(minQuadrilateral[start], minQuadrilateral[(start + 3) % 4]), e[type]));
	
	circle(_draw, minTrapezoid[2] * _scale, 2, RED, FILLED, LINE_AA);
	circle(_draw, minTrapezoid[3] * _scale, 2, RED, FILLED, LINE_AA);

	return minTrapezoid;
}

static void getBarcode1D(Mat &input, Mat &output, vector<Point> &quadrilateral, int quadrilateralStart)
{
	vector<Point2f> object(4);
	vector<Point2f> scene(4);
	scene[0] = Point2f(0, 0);
	scene[1] = Point2f(BAR_1D_WIDTH, 0);
	scene[2] = Point2f(BAR_1D_WIDTH, BAR_1D_HEIGHT);
	scene[3] = Point2f(0, BAR_1D_HEIGHT);
	object[0] = quadrilateral[quadrilateralStart] * _scale;
	object[1] = quadrilateral[(quadrilateralStart + 1) % 4] * _scale;
	object[2] = quadrilateral[(quadrilateralStart + 2) % 4] * _scale;
	object[3] = quadrilateral[(quadrilateralStart + 3) % 4] * _scale;
	Mat tran = getPerspectiveTransform(object, scene);
	warpPerspective(input, output, tran, Size(BAR_1D_WIDTH, BAR_1D_HEIGHT));
}

static void drawQuadrilateral(vector<Point> &quadrilateral, Scalar color)
{
	for (int i = 0; i < 4; i++) {
		line(_draw, quadrilateral[i] * _scale, quadrilateral[(i + 1) % 4] * _scale, color, 1, LINE_AA);
	}
}

static void drawMinRectMid(vector<Point> &minRectMid)
{
	line(_draw, minRectMid[0], minRectMid[2], Scalar(0, 0, 255), 1, LINE_AA);
	line(_draw, minRectMid[1], minRectMid[3], Scalar(0, 0, 255), 1, LINE_AA);
}

static void drawConvexHull(int index, vector<int> &hull, Scalar color)
{
	int size = hull.size();
	Point prev = contours[index][hull[size - 1]] * _scale;
	Point cur;
	for (int i = 0; i < size; i++) {
		cur = contours[index][hull[i]] * _scale;
		line(_draw, prev, cur, color, 1, LINE_AA);
		circle(_draw, cur, 2, color, FILLED, LINE_AA);
		prev = cur;
	}
}
