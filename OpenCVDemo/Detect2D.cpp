#include "stdafx.h"

#include "Detect2D.h"

static Mat _draw;
static int _scale;
static vector<vector<Point>> contours;
static vector<double> sortedArea;
static vector<int> sortedIndex;
static vector<vector<Point>> marker;
static Point markerCenter[3];
static int sequence[3];
static Point lowLeft, highLeft, highRight, lowRight;

static void detectMarker(void);
static void getBarcode2D(Mat &input, Mat &output);
static void drawRect(Point2f* rect);
static void drawConvexHull(int index, vector<int> hull);
static void drawQuadrilateral(vector<Point> quadrilateral, Scalar color);
static void printArea(void);

void detect2D(Mat &input, Mat &output, Mat &draw, int scale)
{
	vector<Vec4i> hierarchy;
	_draw = draw;
	_scale = scale;
	findContours(input, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	//cout << contours.size() << endl; //
	detectMarker();
	getBarcode2D(draw, output);
}

static void getSortedArea(void)
{
	vector<double>::iterator itArea;
	vector<int>::iterator itIndex;
	int count = 0;
	sortedArea.clear();
	sortedArea.resize(AREA_RANGE, -1);
	sortedIndex.resize(AREA_RANGE);
	for (int i = 0; i < contours.size(); i++) {
		double area = contourArea(contours[i]);
		for (int j = 0; j < AREA_RANGE; j++) {
			itArea = sortedArea.begin() + j;
			itIndex = sortedIndex.begin() + j;
			if (area > sortedArea[j]) {
				sortedArea.insert(itArea, area);
				sortedArea.erase(sortedArea.end() - 1);
				sortedIndex.insert(itIndex, i);
				sortedIndex.erase(sortedIndex.end() - 1);
				count++;
				break;
			}
		}
	}
	if (count < 16) {
		sortedArea.resize(count);
		sortedIndex.resize(count);
	}
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
	Point prev, cur, next;
	float lenPrev, lenCur;
	int _dot;
	int size = originHull.size();
	prev = contours[index][originHull[size - 1]];
	cur = contours[index][originHull[0]];
	lenPrev = len2(prev, cur);
	for (int i = 0; i < originHull.size(); i++) {
		next = contours[index][originHull[(i + 1) % size]];
		lenCur = len2(cur, next);
		_dot = dot(cur, prev, cur, next);
		if (_dot * _dot < lenPrev * lenCur * COS_170 * COS_170) {
			reduceHull.push_back(originHull[i]);
		}
		prev = cur;
		cur = next;
		lenPrev = lenCur;
	}
	return reduceHull;
}

static bool validateEdge(int index, vector<int> &hull, int hullIndex, Point* mid, int midIndex, int det)
{
	Point p1 = contours[index][hull[hullIndex]];
	Point p2 = contours[index][hull[(hullIndex + 1) % hull.size()]];
	Point m1 = mid[midIndex];
	Point m2 = mid[midIndex + 2];
	int sign1, sign2;
	if (det == 0) {
		sign1 = p1.y * m1.x - p1.x * m1.y;
		sign2 = p2.y * m1.x - p2.x * m1.y;
	} else {
		sign1 = ((m2.y - m1.y) * p1.x - (m2.x - m1.x) * p1.y) - det;
		sign2 = ((m2.y - m1.y) * p2.x - (m2.x - m1.x) * p2.y) - det;
	}
	return (sign1 < 0 && sign2 >= 0) || (sign2 < 0 && sign1 >= 0) ? true : false;
}

static Point getIntersection(Point p1, Point p2, Point q1, Point q2)
{
	int mat[2][2];
	mat[0][0] = p1.y - p2.y;
	mat[0][1] = p2.x - p1.x;
	mat[1][0] = q1.y - q2.y;
	mat[1][1] = q2.x - q1.x;
	int det = mat[0][0] * mat[1][1] - mat[0][1] * mat[1][0];
	if (det == 0) return Point(-1, -1);
	int a = p2.x * p1.y - p1.x * p2.y;
	int b = q2.x * q1.y - q1.x * q2.y;
	int x0 = (mat[1][1] * a - mat[0][1] * b) / det;
	int y0 = (mat[0][0] * b - mat[1][0] * a) / det;
	return Point(x0, y0);
}

static vector<Point> getMinQuadrilateral(int index)
{
	vector<Point> minQuadrilateral;
	Point2f minRect[4];
	Point minRectMid[4];
	vector<int> originHull;
	vector<int> validEdge;
	int det[2];
	int activeMid;
	int size;
	int i;
	minAreaRect(Mat(contours[index])).points(minRect);
	for (i = 0; i < 4; i++) {
		minRectMid[i].x = (minRect[i].x + minRect[(i + 1) % 4].x) / 2;
		minRectMid[i].y = (minRect[i].y + minRect[(i + 1) % 4].y) / 2;
	}
	convexHull(Mat(contours[index]), originHull, false); // false is clockwise, why?
	//vector<int> &reduceHull = reduceConvexHull(index, originHull);
	//if (reduceHull.size() < 4) return minQuadrilateral;
	size = originHull.size();
	det[0] = minRectMid[0].x * minRectMid[2].y - minRectMid[2].x * minRectMid[0].y;
	det[1] = minRectMid[1].x * minRectMid[3].y - minRectMid[3].x * minRectMid[1].y;
	for (i = 0; i < size; i++) {
		if (validateEdge(index, originHull, i, minRectMid, 0, det[0])) {
			validEdge.push_back(i);
			activeMid = 1;
			break;
		} else if (validateEdge(index, originHull, i, minRectMid, 1, det[1])) {
			validEdge.push_back(i);
			activeMid = 0;
			break;
		}
	}
	for (i++; i < size; i++) {
		if (validateEdge(index, originHull, i, minRectMid, activeMid, det[activeMid])) {
			validEdge.push_back(i);
			activeMid = 1 - activeMid;
		}
	}
	if (validEdge.size() != 4) return minQuadrilateral;
	for (int i = 0; i < 4; i++) {
		Point p1 = contours[index][originHull[validEdge[i]]];
		Point p2 = contours[index][originHull[(validEdge[i] + 1) % size]];
		Point q1 = contours[index][originHull[validEdge[(i + 1) % 4]]];
		Point q2 = contours[index][originHull[(validEdge[(i + 1) % 4] + 1) % size]];
		Point temp = getIntersection(p1, p2, q1, q2);
		if (temp.x < 0) return minQuadrilateral;
		minQuadrilateral.push_back(temp);
	}
	return minQuadrilateral;
}

static void getCenter(void)
{
	for (int i = 0; i < 3; i++) {
		markerCenter[i].x = (marker[i][0].x + marker[i][1].x + marker[i][2].x + marker[i][3].x) / 4;
		markerCenter[i].y = (marker[i][0].y + marker[i][1].y + marker[i][2].y + marker[i][3].y) / 4;
	}
}

static void getSequence(void)
{
	int min = WIDTH * WIDTH;
	int index = -1;
	for (int i = 0; i < 3; i++) {
		int temp = dot(markerCenter[i], markerCenter[(i + 1) % 3], markerCenter[i], markerCenter[(i + 2) % 3]);
		if (temp < min) {
			min = temp;
			index = i;
		}
	}
	Point low = markerCenter[(index + 1) % 3] - markerCenter[index];
	Point right = markerCenter[(index + 2) % 3] - markerCenter[index];
	if (low.x * right.y - low.y * right.x < 0) {
		sequence[0] = (index + 1) % 3;
		sequence[2] = (index + 2) % 3;
	} else {
		sequence[0] = (index + 2) % 3;
		sequence[2] = (index + 1) % 3;
	}
	sequence[1] = index;
}

static int getLowLeft(void)
{
	int max = -WIDTH * WIDTH;
	int index = -1;
	for (int i = 0; i < 4; i++) {
		int temp = dot(markerCenter[sequence[0]], markerCenter[sequence[1]], marker[sequence[0]][i], marker[sequence[0]][(i + 1) % 4]);
		if (temp > max) {
			max = temp;
			index = i;
		}
	}
	lowLeft = marker[sequence[0]][index];
	return index;
}

static int getHighRight(void)
{
	int max = -WIDTH * WIDTH;
	int index = -1;
	for (int i = 0; i < 4; i++) {
		int temp = dot(markerCenter[sequence[1]], markerCenter[sequence[2]], marker[sequence[2]][(i + 3) % 4], marker[sequence[2]][i]);
		if (temp > max) {
			max = temp;
			index = i;
		}
	}
	highRight = marker[sequence[2]][index];
	return index;
}

static void getHighLeft(void)
{
	int max = -WIDTH * WIDTH;
	int index = -1;
	for (int i = 0; i < 4; i++) {
		int temp = dot(markerCenter[sequence[0]], markerCenter[sequence[1]], marker[sequence[1]][(i + 3) % 4], marker[sequence[1]][i]);
		if (temp > max) {
			max = temp;
			index = i;
		}
	}
	highLeft = marker[sequence[1]][index];
}

static void getLowRight(int ll, int hr)
{
	lowRight = getIntersection(marker[sequence[0]][ll], marker[sequence[0]][(ll + 3) % 4], marker[sequence[2]][hr], marker[sequence[2]][(hr + 1) % 4]);
}

static void detectMarker(void)
{
	marker.clear();
	getSortedArea();
	for (int i = 0; i < sortedIndex.size(); i++) {
		vector<Point> &temp = getMinQuadrilateral(sortedIndex[i]);
		if (temp.size() != 4) continue;
		if (sortedArea[i] / contourArea(temp) > AREA_RATIO) {
			marker.push_back(temp);
		}
	}
	if (marker.size() < 3) return;
	getCenter();
	getSequence();
	getHighLeft();
	getLowRight(getLowLeft(), getHighRight());
	/*
	drawQuadrilateral(marker[sequence[0]], Scalar(255, 0, 255));
	drawQuadrilateral(marker[sequence[1]], Scalar(0, 255, 0));
	drawQuadrilateral(marker[sequence[2]], Scalar(0, 0, 255));
	circle(_draw, lowLeft * _scale, 2, Scalar(0, 0, 255), FILLED, LINE_AA);
	circle(_draw, highLeft * _scale, 2, Scalar(0, 255, 0), FILLED, LINE_AA);
	circle(_draw, highRight * _scale, 2, Scalar(255, 0, 0), FILLED, LINE_AA);
	circle(_draw, lowRight * _scale, 2, Scalar(255, 0, 255), FILLED, LINE_AA);
	*/
}

static void getBarcode2D(Mat &input, Mat &output)
{
	vector<Point2f> object(4);
	vector<Point2f> scene(4);
	scene[0] = Point2f(0, 0);
	scene[1] = Point2f(BAR_2D_WIDTH, 0);
	scene[2] = Point2f(BAR_2D_WIDTH, BAR_2D_HEIGHT);
	scene[3] = Point2f(0, BAR_2D_HEIGHT);
	object[0] = highLeft * _scale;
	object[1] = highRight * _scale;
	object[2] = lowRight * _scale;
	object[3] = lowLeft * _scale;
	Mat tran = getPerspectiveTransform(object, scene);
	warpPerspective(input, output, tran, Size(BAR_2D_WIDTH, BAR_2D_HEIGHT));
}

static void drawRect(Point2f* rect)
{
	for (int i = 0; i < 4; i++) {
		line(_draw, rect[i] * _scale, rect[(i + 1) % 4] * _scale, Scalar(0, 0, 255), 1, LINE_AA);
	}
}

static void drawConvexHull(int index, vector<int> hull)
{
	Point prev, cur;
	int size = hull.size();
	prev = contours[index][hull[size - 1]] * _scale;
	for (int i = 0; i < size; i++) {
		cur = contours[index][hull[i]] * _scale;
		line(_draw, prev, cur, Scalar(255, 0, 255), 1, LINE_AA);
		circle(_draw, cur, 2, Scalar(255, 0, 255), FILLED, LINE_AA);
		prev = cur;
	}
}

static void drawQuadrilateral(vector<Point> quadrilateral, Scalar color)
{
	for (int i = 0; i < 4; i++) {
		line(_draw, quadrilateral[i] * _scale, quadrilateral[(i + 1) % 4] * _scale, color, 2, LINE_AA);
	}
}

static void printArea(void)
{
	cout << "area: ";
	for (int i = 0; i < sortedArea.size(); i++) {
		cout << sortedArea[i] << " ";
	}
	cout << endl;
	for (int i = 0; i < sortedArea.size(); i++) {
		cout << sortedIndex[i] << " ";
	}
	cout << endl << endl;
}
