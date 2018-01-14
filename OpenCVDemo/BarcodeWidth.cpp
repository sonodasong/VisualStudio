#include "stdafx.h"

#include "BarcodeWidth.h"

#define MIN_BAR_WDITH		4
#define MAX_BAR_WIDTH		12

static vector<bool> getLineProfile(Mat &input, int y);
static vector<int> getLineSegment(vector<bool> lineProfile);
static pair<int, int> getBarcodeWitdhSingleLine(vector<int> lineSegment);
static void segmentHist(String window, vector<int> lineSegment);
static void printVector(vector<int> input);
static pair<int, int> compareWidth(pair<int, int> x, pair<int, int> y);

int getBarcodeWidth(Mat &input, int step, int num)
{
	pair<int, int> width = pair<int, int>(0, 0);
	int h = input.rows;
	for (int y = (h + step) / 2; y < h / 2 + step * num; y += step) {
		vector<bool> lineProfile = getLineProfile(input, y);
		vector<int> lineSegment = getLineSegment(lineProfile);
		width = compareWidth(width, getBarcodeWitdhSingleLine(lineSegment));
	}
	for (int y = (h - step) / 2; y > h / 2 - step * num; y -= step) {
		vector<bool> lineProfile = getLineProfile(input, y);
		vector<int> lineSegment = getLineSegment(lineProfile);
		width = compareWidth(width, getBarcodeWitdhSingleLine(lineSegment));
	}
	return width.second;
}

int getBarcodeWidthLine(Mat &input, int y, String window)
{
	vector<bool> lineProfile = getLineProfile(input, y);
	vector<int> lineSegment = getLineSegment(lineProfile);
	/*
	for (int i = 0; i < lineSegment.size(); i++) {
		cout << lineSegment.at(i) << " ";
	}
	cout << endl;
	*/
	pair<int, int> width = getBarcodeWitdhSingleLine(lineSegment);
	//cout << endl << "<" << width.first << ", " << width.second << "> " << endl;

	segmentHist(window, lineSegment);
	return width.second;
}

static vector<bool> getLineProfile(Mat &input, int y)
{
	vector<bool> lineProfile;
	for (int i = 0; i < input.cols; i++) {
		lineProfile.push_back(input.at<uchar>(Point(i, y)) > 128 ? true : false);
	}
	return lineProfile;
}

static vector<int> getLineSegment(vector<bool> lineProfile)
{
	vector<int> lineSegment;
	bool state = lineProfile.at(0);
	int count = 1;
	for (int i = 1; i < lineProfile.size(); i++) {
		if (lineProfile.at(i) == state) {
			count++;
		}
		else {
			if (!state) lineSegment.push_back(count); // only black
			//lineSegment.push_back(count); // both white and black
			state = lineProfile.at(i);
			count = 1;
		}
	}
	lineSegment.push_back(count);
	return lineSegment;
}

static bool testThreshold(vector<int> consecutive, int max, bool debug) {
	float mean = 0;
	int size = consecutive.size();
	for (int i = 0; i < size; i++) {
		mean += consecutive.at(i);
	}
	mean /= size;
	if (debug) {
		cout << endl << "mean, max: " << mean << " " << max << endl;
		printVector(consecutive);
	}
	return (max < mean * THRESHOLD_COEFFICIENT) ? true : false;
}

static pair<int, int> getBarcodeWitdhSingleLineThreshold(vector<int> lineSegment, int threshold)
{
	vector<int> consecutive;
	vector<int> tempConsecutive;
	bool lock = false;
	int max = 0;
	int tempMax = 0;
	for (int i = 0; i < lineSegment.size(); i++) {
		int segment = lineSegment.at(i);
		if (lock) {
			if (segment > threshold) {
				lock = false;
				if ((tempConsecutive.size() > consecutive.size()) && testThreshold(tempConsecutive, tempMax, false)) {
					consecutive = tempConsecutive;
					max = tempMax;
				}
			} else {
				tempConsecutive.push_back(segment);
				if (segment > tempMax) { tempMax = segment; }
			}
		} else if (segment <= threshold) {
			lock = true;
			tempConsecutive.clear();
			tempConsecutive.push_back(segment);
			tempMax = segment;
		}
	}
	//cout << endl << "threshold: " << threshold << " max: "; //
	//printVector(consecutive); //
	//testThreshold(consecutive, max, true); //
	return pair<int, int>((int)consecutive.size(), max);
}

static pair<int, int> getBarcodeWitdhSingleLine(vector<int> lineSegment)
{
	pair<int, int> barcodeWidthSingleLine = getBarcodeWitdhSingleLineThreshold(lineSegment, MAX_BAR_WIDTH);
	for (int threshold = MIN_BAR_WDITH; threshold < MAX_BAR_WIDTH; threshold++) {
		pair<int, int> temp = getBarcodeWitdhSingleLineThreshold(lineSegment, threshold);
		barcodeWidthSingleLine = compareWidth(barcodeWidthSingleLine, temp);
	}
	return barcodeWidthSingleLine;
}

/*
static vector<pair<int, int>> analyzeLineSegment1(vector<int> lineSegment)
{
	vector<pair<int, int>>barWidth;
	bool lock = false;
	int length = 0;
	int max = 0;
	for (int i = 1; i < lineSegment.size(); i++) {
		int prev = lineSegment.at(i - 1);
		prev = (prev > 1) ? prev : 2; // minimum is 2
		int cur = lineSegment.at(i);
		cur = (cur > 1) ? cur : 2; // minimum is 2
		if (prev >= cur * 5) {
			lock = true;
			length = 1;
			max = cur;
		}
		else if (lock) {
			if (cur >= prev * 5) {
				lock = false;
				barWidth.push_back(pair<int, int>(length, max));
			}
			else {
				length++;
				max = (cur > max) ? cur : max;
			}
		}
	}
	return barWidth;
}

#define RATIO		3
#define OFFSET		2

static vector<int> getDeviation(vector<int> lineSegment)
{
	vector<int> deviation;
	int size = lineSegment.size();
	for (int i = 0; i < size; i++) {
		int x0 = (i == 0) ? 0 : lineSegment.at(i - 1);
		int x1 = lineSegment.at(i);
		int x2 = (i == (size - 1)) ? 0 : lineSegment.at(i + 1);
		int mean = (x0 + x1 + x2) / 3;
		int dev = (x0 * x0 + x1 * x1 + x2 * x2) / 3 - mean * mean;
		deviation.push_back(dev + OFFSET);
	}
	return deviation;
}

static vector<pair<int, int>> analyzeLineSegment2(vector<int> lineSegment)
{
	vector<int>deviation = getDeviation(lineSegment);
	vector<pair<int, int>>barWidth;
	bool devLock = false;
	int devLen = 0;
	int segMax = 0;
	for (int i = 1; i < deviation.size(); i++) {
		int devPrev = deviation.at(i - 1);
		int devCur = deviation.at(i);
		int segCur = lineSegment.at(i);
		if (devPrev >= devCur * RATIO) {
			devLock = true;
			devLen = 1;
			segMax = segCur;
		}
		else if (devLock) {
			if (devCur >= devPrev * RATIO) {
				devLock = false;
				barWidth.push_back(pair<int, int>(devLen, segMax));
			}
			else {
				devLen++;
				if (segCur > segMax) { segMax = segCur; }
			}
		}
	}
	return barWidth;
}
*/

static void segmentHist(String window, vector<int> lineSegment)
{
	Mat hist = Mat::zeros(Size(100, 200), CV_8U);
	for (int i = 0; i < lineSegment.size(); i++) {
		line(hist, Point(i, 0), Point(i, lineSegment.at(i)), 255);
	}
	imshow(window, hist);
}

static void printVector(vector<int> input)
{
	for (int i = 0; i < input.size(); i++) {
		cout << input.at(i) << " ";
	}
	cout << endl;
}

static pair<int, int> compareWidth(pair<int, int> x, pair<int, int> y)
{
	return x.first > y.first ? x : y;
}
