#include "stdafx.h"

#include "Width1D.h"

static int lineSegment[WIDTH];
static int lineSegmentSize = 0;

static pair<int, int> getWitdh1DLineThreshold(int threshold);
static void lineSegmentHist(String window);
static void printArray(String tag, int *array, int size);

static void getLineSegment(Mat &input, int y)
{
	bool state = input.at<uchar>(Point(0, y)) > 128 ? true : false;
	int count = 1;
	lineSegmentSize = 0;
	bool pixel;
	int i;
	for (i = 1; i < WIDTH; i++) {
		pixel = input.at<uchar>(Point(i, y)) > 128 ? true : false;
		if (pixel == state) {
			count++;
		} else {
			if (!state) {
				lineSegment[lineSegmentSize] = count;
				lineSegmentSize++;
			}
			state = pixel;
			count = 1;
		}
	}
}

int getWidth1DLineHist(Mat &input, int y, String window)
{
	getLineSegment(input, y);
	//printArray("lineSegment", lineSegment, lineSegmentSize);
	lineSegmentHist(window);
	getWitdh1DLineThreshold(6);
	return 0;
}

static bool testThreshold(int *consecutive, int consecutiveSize, int max, bool debug) {
	float mean = 0;
	for (int i = 0; i < consecutiveSize; i++) {
		mean += consecutive[i];
	}
	mean /= consecutiveSize;
	if (debug) {
		printArray("consecutive", consecutive, consecutiveSize);
		cout << "mean, max: " << mean << " " << max << endl;
	}
	return (max < mean * THRESHOLD_COEFFICIENT) ? true : false;
}

static pair<int, int> getWitdh1DLineThreshold(int threshold)
{
	int a[WIDTH], b[WIDTH];
	int *swap;
	int *consecutive = a;
	int consecutiveSize = 0;
	int *tempConsecutive = b;
	int tempConsecutiveSize = 0;
	bool lock = false;
	int max = 0;
	int tempMax = 0;
	int segment;
	int i;
	for (i = 0; i < lineSegmentSize; i++) {
		segment = lineSegment[i];
		if (lock) {
			if (segment > threshold) {
				lock = false;
				if ((tempConsecutiveSize > consecutiveSize) && testThreshold(tempConsecutive, tempConsecutiveSize, tempMax, false)) {
					swap = consecutive;
					consecutive = tempConsecutive;
					tempConsecutive = swap;
					consecutiveSize = tempConsecutiveSize;
					max = tempMax;
				}
			} else {
				tempConsecutive[tempConsecutiveSize] = segment;
				tempConsecutiveSize++;
				if (segment > tempMax) { tempMax = segment; }
			}
		} else if (segment <= threshold) {
			lock = true;
			tempConsecutive[0] = segment;
			tempConsecutiveSize = 1;
			tempMax = segment;
		}
	}
	//cout << "threshold: " << threshold << endl; //
	//testThreshold(consecutive, consecutiveSize, max, true); //
	return pair<int, int>(consecutiveSize, max);
}

static void lineSegmentHist(String window)
{
	Mat hist = Mat::zeros(Size(100, 200), CV_8U);
	for (int i = 0; i < lineSegmentSize; i++) {
		line(hist, Point(i, 0), Point(i, lineSegment[i]), 255);
	}
	imshow(window, hist);
}

static void printArray(String tag, int *array, int size)
{
	cout << tag << ": ";
	for (int i = 0; i < size; i++) {
		cout << array[i] << " ";
	}
	cout << endl;
}
