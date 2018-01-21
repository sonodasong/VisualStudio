#include "stdafx.h"

#include "Util.h"

static int64 start = 0;

void performanceStart(void)
{
	start = getTickCount();
}

void performanceStop(String tag)
{
	int64 stop = getTickCount();
	cout << tag << " performance: " << (stop - start) * 1000 / getTickFrequency() << "ms" << endl;
}

void getOctagonKernel(Mat &kernel, int size)
{
	kernel = Mat::ones(size, size, CV_8U);
	int border = (size - 1) / 3;
	for (int i = 0; i < size; i++) {
		for (int j = 0; j < size; j++) {
			if ((i + j) < border) {
				kernel.at<uchar>(Point(i, j)) = 0;
			}
			else if ((size - 1 - j + i) < border) {
				kernel.at<uchar>(Point(i, j)) = 0;
			}
			else if ((size * 2 - 2 - i - j) < border) {
				kernel.at<uchar>(Point(i, j)) = 0;
			}
			else if ((size - 1 - i + j) < border) {
				kernel.at<uchar>(Point(i, j)) = 0;
			}
		}
	}
}

void sharpen1(Mat &input, Mat &output)
{
	GaussianBlur(input, output, Size(0, 0), 3);
	addWeighted(input, 1.5, output, -0.5, 0, output);
}

void sharpen2(Mat &input, Mat &output)
{
	Mat kernel = Mat::zeros(3, 3, CV_8S);
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			kernel.at<uchar>(Point(i, j)) = -1;
		}
	}
	kernel.at<uchar>(Point(1, 1)) = 9;
	filter2D(input, output, -1, kernel);
}
