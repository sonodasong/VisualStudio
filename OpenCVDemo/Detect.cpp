#include "stdafx.h"

#include "Macros.h"
#include "Detect.h"

using namespace std;
using namespace cv;

static vector<Point> getMinRect(const vector<Point> &contour);
static vector<Point> getMinRectMid(const vector<Point> &minRect);
static vector<Point> getConvexHull(const vector<Point> &contour);
static vector<Point> getReduceConvexHull(const vector<Point> &originHull);
static bool validateEdge(const vector<Point> &hull, int hullIndex, const vector<Point> &mid, int midIndex, int det);

int len2(const Point &p1, const Point &p2)
{
	int x = p1.x - p2.x;
	int y = p1.y - p2.y;
	return x * x + y * y;
}

int dot(const Point &p1, const Point &p2, const Point &p3, const Point &p4)
{
	return (p2.x - p1.x) * (p4.x - p3.x) + (p2.y - p1.y) * (p4.y - p3.y);
}

// e[0] * x + e[1] * y + e[2] = 0;
vector<int> getLineEquation(const Point &p1, const Point &p2)
{
	vector<int> e;
	e.push_back(p1.y - p2.y);
	e.push_back(p2.x - p1.x);
	e.push_back(p1.x * p2.y - p2.x * p1.y);
	return e;
}

Point getIntersection(const vector<int> &e1, const vector<int> &e2)
{
	int det = e1[0] * e2[1] - e1[1] * e2[0];
	if (det == 0) return Point(-1, -1);
	int x = (e1[1] * e2[2] - e2[1] * e1[2]) / det;
	int y = (e2[0] * e1[2] - e1[0] * e2[2]) / det;
	return Point(x, y);
}

vector<Point> getMinQuadrilateral(const vector<Point> &contour)
{
	vector<Point> minQuadrilateral;
	vector<int> validEdge;
	int i;
	int activeMid;

	const vector<Point> &originHull = getConvexHull(contour);
	const vector<Point> &reduceHull = getReduceConvexHull(originHull);
	if (reduceHull.size() < 4) return minQuadrilateral;
	int size = reduceHull.size();
	const vector<Point> &minRectMid = getMinRectMid(getMinRect(contour));
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
		const vector<int> &e1 = getLineEquation(reduceHull[validEdge[i]], reduceHull[(validEdge[i] + 1) % size]);
		const vector<int> &e2 = getLineEquation(reduceHull[validEdge[(i + 1) % 4]], reduceHull[(validEdge[(i + 1) % 4] + 1) % size]);
		const Point &temp = getIntersection(e1, e2);
		if (temp.x < 0) return minQuadrilateral;
		minQuadrilateral.push_back(temp);
	}
	return minQuadrilateral;
}

void drawPolygon(Mat &draw, const vector<Point> &hull, Scalar color, int scale)
{
	Point prev, cur;
	int size = hull.size();
	prev = hull[size - 1] * scale;
	for (int i = 0; i < size; i++) {
		cur = hull[i] * scale;
		line(draw, prev, cur, color, 1, LINE_AA);
		//circle(draw, cur, 2, color, FILLED, LINE_AA);
		prev = cur;
	}
}

static vector<Point> getMinRect(const vector<Point> &contour)
{
	vector<Point> minRect;
	Point2f temp[4];
	minAreaRect(Mat(contour)).points(temp);
	for (int i = 0; i < 4; i++) {
		minRect.push_back(Point(temp[i].x, temp[i].y));
	}
	return minRect;
}

static vector<Point> getMinRectMid(const vector<Point> &minRect)
{
	vector<Point> minRectMid;
	for (int i = 0; i < 4; i++) {
		int next = (i + 1) % 4;
		minRectMid.push_back(Point((minRect[i].x + minRect[next].x) / 2, (minRect[i].y + minRect[next].y) / 2));
	}
	return minRectMid;
}

static vector<Point> getConvexHull(const vector<Point> &contour)
{
	vector<Point> originHull;
	vector<int> hull;
	convexHull(Mat(contour), hull, false); // false is clockwise, why?
	for (int i = 0; i < hull.size(); i++) {
		originHull.push_back(contour[hull[i]]);
	}
	return originHull;
}

static vector<Point> getReduceConvexHull(const vector<Point> &originHull)
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

static bool validateEdge(const vector<Point> &hull, int hullIndex, const vector<Point> &mid, int midIndex, int det)
{
	const Point &p1 = hull[hullIndex];
	const Point &p2 = hull[(hullIndex + 1) % hull.size()];
	const Point &m1 = mid[midIndex];
	const Point &m2 = mid[midIndex + 2];
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
