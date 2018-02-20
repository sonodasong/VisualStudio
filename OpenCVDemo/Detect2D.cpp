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
static Point lowLeft, highLeft, highRight, lowRight, lowRightCenter;

static void detectMarker(void);
static void getBarcode2D(Mat &input, Mat &output);
static void getBarcode2DCenter(Mat &input, Mat &output);
static void drawQuadrilateral(vector<Point> &quadrilateral, Scalar color);
static void drawConvexHull(vector<Point> &hull, Scalar color);
static void printArea(void);

void detect2D(Mat &input, Mat &output, Mat &draw, int scale)
{
	vector<Vec4i> hierarchy;
	_draw = draw;
	_scale = scale;
	findContours(input, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	cout << contours.size() << endl; //
	detectMarker();
	getBarcode2D(draw, output);
	//getBarcode2DCenter(draw, output);
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

static vector<Point> getMinRect(vector<Point> &contour)
{
	vector<Point> minRect;
	Point2f temp[4];
	minAreaRect(Mat(contour)).points(temp);
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

static vector<Point> getConvexHull(vector<Point> &contour)
{
	vector<Point> originHull;
	vector<int> hull;
	convexHull(Mat(contour), hull, false); // false is clockwise, why?
	for (int i = 0; i < hull.size(); i++) {
		originHull.push_back(contour[hull[i]]);
	}
	return originHull;
}

static int len2(Point &p1, Point &p2)
{
	int x = p1.x - p2.x;
	int y = p1.y - p2.y;
	return x * x + y * y;
}

static int dot(Point &p1, Point &p2, Point &p3, Point &p4)
{
	return (p2.x - p1.x) * (p4.x - p3.x) + (p2.y - p1.y) * (p4.y - p3.y);
}

static vector<Point> getReduceConvexHull(vector<Point> &originHull)
{
	vector<Point> reduceHull;
	int size = originHull.size();
	Point prev = originHull[size - 1];
	Point cur = originHull[0];
	Point next;
	float lenPrev = len2(prev, cur);
	float lenCur;
	for (int i = 0; i < originHull.size(); i++) {
		next = originHull[(i + 1) % size];
		lenCur = len2(cur, next);
		int _dot = dot(cur, prev, cur, next);
		if (_dot * _dot < lenPrev * lenCur * COS_160 * COS_160) {
			reduceHull.push_back(cur);
		}
		prev = cur;
		cur = next;
		lenPrev = lenCur;
	}
	return reduceHull;
}

static bool validateEdge(vector<Point> &hull, int hullIndex, vector<Point> &mid, int midIndex, int det)
{
	Point p1 = hull[hullIndex];
	Point p2 = hull[(hullIndex + 1) % hull.size()];
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
static vector<int> getLineEquation(Point &p1, Point &p2)
{
	vector<int> e;
	e.push_back(p1.y - p2.y);
	e.push_back(p2.x - p1.x);
	e.push_back(p1.x * p2.y - p2.x * p1.y);
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

static vector<Point> getMinQuadrilateral(vector<Point> &contour)
{
	vector<Point> minQuadrilateral;
	vector<int> validEdge;
	int i;
	int activeMid;

	/*vector<Point> approx;
	approxPolyDP(Mat(contours[index]), approx, arcLength(Mat(contours[index]), true)*0.03, true);
	if (approx.size() == 4) {
		drawQuadrilateral(approx, BLUE);
		return approx;
	}*/

	vector<Point> &originHull = getConvexHull(contour);
	vector<Point> &reduceHull = getReduceConvexHull(originHull);
	//vector<Point> &reduceHull = originHull;
	if (reduceHull.size() < 4) return minQuadrilateral;
	//drawConvexHull(reduceHull, BLUE); //
	int size = reduceHull.size();
	vector<Point> &minRectMid = getMinRectMid(getMinRect(contour));
	int det[2] = {
		minRectMid[0].x * minRectMid[2].y - minRectMid[2].x * minRectMid[0].y,
		minRectMid[1].x * minRectMid[3].y - minRectMid[3].x * minRectMid[1].y
	};
	for (i = 0; i < size; i++) {
		if (validateEdge(reduceHull, i, minRectMid, 0, det[0])) {
			validEdge.push_back(i);
			activeMid = 1;
			break;
		}
		else if (validateEdge(reduceHull, i, minRectMid, 1, det[1])) {
			validEdge.push_back(i);
			activeMid = 0;
			break;
		}
	}
	for (i++; i < size; i++) {
		if (validateEdge(reduceHull, i, minRectMid, activeMid, det[activeMid])) {
			validEdge.push_back(i);
			activeMid = 1 - activeMid;
		}
	}
	if (validEdge.size() != 4) return minQuadrilateral;
	for (i = 0; i < 4; i++) {
		vector<int> &e1 = getLineEquation(reduceHull[validEdge[i]], reduceHull[(validEdge[i] + 1) % size]);
		vector<int> &e2 = getLineEquation(reduceHull[validEdge[(i + 1) % 4]], reduceHull[(validEdge[(i + 1) % 4] + 1) % size]);
		Point temp = getIntersection(e1, e2);
		if (temp.x < 0) return minQuadrilateral;
		minQuadrilateral.push_back(temp);
	}
	return minQuadrilateral;
}

static vector<Point> getReverseOrder(vector<Point> &input)
{
	vector<Point> output;
	int size = input.size();
	for (int i = 0; i < size; i++) {
		output.push_back(input[size - 1 - i]);
	}
	return output;
}

static vector<Point> getMinQuadrilateralDP(vector<Point> &contour)
{
	vector<Point> minQuadrilateral;
	vector<int> validEdge;
	int i;
	int activeMid;

	vector<Point> approx;
	approxPolyDP(Mat(contour), approx, arcLength(Mat(contour), true)*0.02, true);
	if (approx.size() > 5) return minQuadrilateral;
	vector<Point> &reduceHull = getReverseOrder(approx);
	if (reduceHull.size() < 4) return minQuadrilateral;
	//drawConvexHull(reduceHull, BLUE); //
	int size = reduceHull.size();
	vector<Point> &minRectMid = getMinRectMid(getMinRect(contour));
	int det[2] = {
		minRectMid[0].x * minRectMid[2].y - minRectMid[2].x * minRectMid[0].y,
		minRectMid[1].x * minRectMid[3].y - minRectMid[3].x * minRectMid[1].y
	};
	for (i = 0; i < size; i++) {
		if (validateEdge(reduceHull, i, minRectMid, 0, det[0])) {
			validEdge.push_back(i);
			activeMid = 1;
			break;
		}
		else if (validateEdge(reduceHull, i, minRectMid, 1, det[1])) {
			validEdge.push_back(i);
			activeMid = 0;
			break;
		}
	}
	for (i++; i < size; i++) {
		if (validateEdge(reduceHull, i, minRectMid, activeMid, det[activeMid])) {
			validEdge.push_back(i);
			activeMid = 1 - activeMid;
		}
	}
	if (validEdge.size() != 4) return minQuadrilateral;
	for (i = 0; i < 4; i++) {
		vector<int> &e1 = getLineEquation(reduceHull[validEdge[i]], reduceHull[(validEdge[i] + 1) % size]);
		vector<int> &e2 = getLineEquation(reduceHull[validEdge[(i + 1) % 4]], reduceHull[(validEdge[(i + 1) % 4] + 1) % size]);
		Point temp = getIntersection(e1, e2);
		if (temp.x < 0) return minQuadrilateral;
		minQuadrilateral.push_back(temp);
	}
	return minQuadrilateral;
}

static bool testSquare(vector<Point> &quadrilateral)
{
	float ratio = 1.0 * len2(quadrilateral[0], quadrilateral[1]) / len2(quadrilateral[1], quadrilateral[2]);
	return (ratio > 0.25 && ratio < 4) ? true : false;
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
	Point &low = markerCenter[(index + 1) % 3] - markerCenter[index];
	Point &right = markerCenter[(index + 2) % 3] - markerCenter[index];
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
	vector<int> &e1 = getLineEquation(marker[sequence[0]][ll], marker[sequence[0]][(ll + 3) % 4]);
	vector<int> &e2 = getLineEquation(marker[sequence[2]][hr], marker[sequence[2]][(hr + 1) % 4]);
	lowRight = getIntersection(e1, e2);
}

static void getLowRightCenter(int ll, int hr)
{
	vector<int> &left = getLineEquation(marker[sequence[2]][(hr + 2) % 4], marker[sequence[2]][(hr + 3) % 4]);
	vector<int> &right = getLineEquation(marker[sequence[2]][hr], marker[sequence[2]][(hr + 1) % 4]);
	vector<int> &low = getLineEquation(marker[sequence[0]][ll], marker[sequence[0]][(ll + 3) % 4]);
	vector<int> &high = getLineEquation(marker[sequence[0]][(ll + 1) % 4], marker[sequence[0]][(ll + 2) % 4]);
	lowRightCenter = (getIntersection(left, low) + getIntersection(left, high) + getIntersection(right, high) + lowRight) / 4;
}

static void detectMarker(void)
{
	marker.clear();
	getSortedArea();
	for (int i = 0; i < sortedIndex.size(); i++) {
		vector<Point> &temp = getMinQuadrilateral(contours[sortedIndex[i]]);
		if (temp.size() != 4) continue;
		if (!testSquare(temp)) continue;
		if (sortedArea[i] / contourArea(temp) > AREA_RATIO) {
			marker.push_back(temp);
		}
		if (marker.size() == 3) break;
	}
	if (marker.size() < 3) return;
	getCenter();
	getSequence();
	getHighLeft();
	int ll = getLowLeft();
	int hr = getHighRight();
	getLowRight(ll, hr);
	//getLowRightCenter(ll, hr);

	//drawQuadrilateral(marker[sequence[0]], Scalar(0, 0, 255));
	//drawQuadrilateral(marker[sequence[1]], Scalar(0, 255, 0));
	//drawQuadrilateral(marker[sequence[2]], Scalar(255, 0, 0));
	//circle(_draw, lowLeft * _scale, 2, Scalar(0, 0, 255), FILLED, LINE_AA);
	//circle(_draw, highLeft * _scale, 2, Scalar(0, 255, 0), FILLED, LINE_AA);
	//circle(_draw, highRight * _scale, 2, Scalar(255, 0, 0), FILLED, LINE_AA);
	//circle(_draw, lowRight * _scale, 2, Scalar(255, 0, 255), FILLED, LINE_AA);
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

static void getBarcode2DCenter(Mat &input, Mat &output)
{
	vector<Point2f> object(4);
	vector<Point2f> scene(4);
	scene[0] = Point2f(0, 0);
	scene[1] = Point2f(BAR_2D_WIDTH, 0);
	scene[2] = Point2f(BAR_2D_WIDTH, BAR_2D_HEIGHT);
	scene[3] = Point2f(0, BAR_2D_HEIGHT);
	object[0] = markerCenter[sequence[1]] * _scale;
	object[1] = markerCenter[sequence[2]] * _scale;
	object[2] = lowRightCenter * _scale;
	object[3] = markerCenter[sequence[0]] * _scale;
	Mat tran = getPerspectiveTransform(object, scene);
	warpPerspective(input, output, tran, Size(BAR_2D_WIDTH, BAR_2D_HEIGHT));
}

static void drawQuadrilateral(vector<Point> &quadrilateral, Scalar color)
{
	int size = quadrilateral.size();
	for (int i = 0; i < size; i++) {
		line(_draw, quadrilateral[i] * _scale, quadrilateral[(i + 1) % size] * _scale, color, 1, LINE_AA);
		circle(_draw, quadrilateral[i] * _scale, 2, color, FILLED, LINE_AA);
	}
}

static void drawConvexHull(vector<Point> &hull, Scalar color)
{
	Point prev, cur;
	int size = hull.size();
	prev = hull[size - 1] * _scale;
	for (int i = 0; i < size; i++) {
		cur = hull[i] * _scale;
		line(_draw, prev, cur, color, 1, LINE_AA);
		//circle(_draw, cur, 2, color, FILLED, LINE_AA);
		prev = cur;
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
