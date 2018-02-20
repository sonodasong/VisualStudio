#pragma once

#include <opencv2\opencv.hpp>

int len2(const cv::Point &p1, const cv::Point &p2);
int dot(const cv::Point &p1, const cv::Point &p2, const cv::Point &p3, const cv::Point &p4);
std::vector<int> getLineEquation(const cv::Point &p1, const cv::Point &p2);
cv::Point getIntersection(const std::vector<int> &e1, const std::vector<int> &e2);
std::vector<cv::Point> getMinQuadrilateral(const std::vector<cv::Point> &contour);
void drawPolygon(cv::Mat &draw, const std::vector<cv::Point> &hull, cv::Scalar color, int scale);
