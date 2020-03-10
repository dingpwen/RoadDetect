//
// Created by wenpd on 2020/2/16.
//
#include <string>
#include "log.h"
#include <opencv2/opencv.hpp>
#include "LaneDetectorBase.h"

using namespace cv;
using namespace std;

#ifndef OPENCV_APPLICATION_ROADEDGE_H
#define OPENCV_APPLICATION_ROADEDGE_H

#define angle(x) (atan(x)*180/CV_PI)
const float MIN_SLOPE = (float)0.3;
const float MAX_SLOPE = 1/MIN_SLOPE;
const Scalar kColorRed(255, 0, 0, 255);
const Scalar kColorDefault(0, 0, 0, 0);
const Scalar kColorGrayMax(255);
const Scalar kColorGrayDefault(0);

class RoadEdge: public LaneDetectorBase{
private:
    int kSize;
    void grayScale(Mat &src, Mat &dst);
    void regionOfInterest(Mat &img, const Point** pts, const int* npts);
    void drawLines(Mat &img, const vector<Vec4f> &lines, const Scalar& color = kColorRed, int thickness = 6);
    Mat houghLines(Mat &img, int threshold, double minLineLen, double maxLineGap);
    template <typename T> T getMean(vector<T> values);
    Mat enhance(const Mat &img);

public:
    RoadEdge(Mat rgbMat, int width, int height):LaneDetectorBase(rgbMat, width, height){
        kSize = 7;
    }
    virtual int processImage(Mat &img);
};


#endif //OPENCV_APPLICATION_ROADEDGE_H
