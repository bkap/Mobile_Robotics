#ifndef _LIB_DEMO_H
#define _LIB_DEMO_H

#include <opencv/cv.h>

CvPoint2D64f blobfind(const cv::Mat& src, cv::Mat& out);

void normalizeColors(const cv::Mat& src, cv::Mat& out);

void findLines(const cv::Mat& src, cv::Mat& out);

#endif
