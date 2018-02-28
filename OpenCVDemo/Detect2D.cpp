#include "stdafx.h"

#include "Macros.h"
#include "Detect.h"
#include "Detect2D.h"

using namespace std;
using namespace cv;

static int _scale;
static Point markerCenter[3];
static int sequence[3];
static Point lowLeft, highLeft, highRight, lowRight;

static void detectMarker(const vector< vector<Point> > &contours);
static void getBarcode2D(const Mat &input, Mat &output);
static void printArea(const vector<double> &sortedArea, const vector<int> &sortedIndex);

void detect2D(const Mat &input, const Mat &original, Mat &output, int scale)
{
	vector< vector<Point> > contours;
	_scale = scale;
	findContours(input, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	detectMarker(contours);
	getBarcode2D(original, output);
}

static void getSortedArea(const vector< vector<Point> > &contours, vector<double> &sortedArea, vector<int> &sortedIndex)
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
	if (count < AREA_RANGE) {
		sortedArea.resize(count);
		sortedIndex.resize(count);
	}
}

static bool testSquare(const vector<Point> &quadrilateral)
{
	float ratio = 1.0 * len2(quadrilateral[0], quadrilateral[1]) / len2(quadrilateral[1], quadrilateral[2]);
	return (ratio > 0.25 && ratio < 4) ? true : false;
}

static void getCenter(const vector< vector<Point> > &marker)
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
	const Point &low = markerCenter[(index + 1) % 3] - markerCenter[index];
	const Point &right = markerCenter[(index + 2) % 3] - markerCenter[index];
	if (low.x * right.y - low.y * right.x < 0) {
		sequence[0] = (index + 1) % 3;
		sequence[2] = (index + 2) % 3;
	} else {
		sequence[0] = (index + 2) % 3;
		sequence[2] = (index + 1) % 3;
	}
	sequence[1] = index;
}

static int getLowLeft(const vector< vector<Point> > &marker)
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

static int getHighRight(const vector< vector<Point> > &marker)
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

static void getHighLeft(const vector< vector<Point> > &marker)
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

static void getLowRight(const vector< vector<Point> > &marker, int ll, int hr)
{
	const vector<int> &e1 = getLineEquation(marker[sequence[0]][ll], marker[sequence[0]][(ll + 3) % 4]);
	const vector<int> &e2 = getLineEquation(marker[sequence[2]][hr], marker[sequence[2]][(hr + 1) % 4]);
	lowRight = getIntersection(e1, e2);
}

static void detectMarker(const vector< vector<Point> > &contours)
{
	vector<double> sortedArea;
	vector<int> sortedIndex;
	vector< vector<Point> > marker;
	getSortedArea(contours, sortedArea, sortedIndex);
	for (int i = 0; i < sortedIndex.size(); i++) {
		const vector<Point> &temp = getMinQuadrilateral(contours[sortedIndex[i]]);
		if (temp.size() != 4) continue;
		if (!testSquare(temp)) continue;
		if (sortedArea[i] / contourArea(temp) < AREA_RATIO) continue;
		marker.push_back(getExtendedQuadrilateral(temp, EXTEND_2D));
		if (marker.size() == 3) break;
	}
	if (marker.size() < 3) return;
	getCenter(marker);
	getSequence();
	getHighLeft(marker);
	int ll = getLowLeft(marker);
	int hr = getHighRight(marker);
	getLowRight(marker, ll, hr);
}

static void getBarcode2D(const Mat &input, Mat &output)
{
	vector<Point2f> object(4);
	vector<Point2f> scene(4);
	scene[0] = Point2f(0, 0);
	scene[1] = Point2f(BAR_WIDTH, 0);
	scene[2] = Point2f(BAR_WIDTH, BAR_HEIGHT);
	scene[3] = Point2f(0, BAR_HEIGHT);
	object[0] = highLeft * _scale;
	object[1] = highRight * _scale;
	object[2] = lowRight * _scale;
	object[3] = lowLeft * _scale;
	Mat tran = getPerspectiveTransform(object, scene);
	warpPerspective(input, output, tran, Size(BAR_WIDTH, BAR_HEIGHT));
}

static void printArea(const vector<double> &sortedArea, const vector<int> &sortedIndex)
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
