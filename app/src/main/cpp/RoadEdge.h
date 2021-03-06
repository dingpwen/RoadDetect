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
const float VALID_ANGLE = 30;
const float VALID_X = float(7)/24;
const Scalar kColorRed(255, 0, 0, 255);
const Scalar kColorDefault(0, 0, 0, 0);
const Scalar kColorGrayMax(255);
const Scalar kColorGrayDefault(0);

enum resultSet{
    RESULT_NONE = -1,
    RESULT_GO_STRAIGHT = 0,
    RESULT_TURN_LEFT,
    RESULT_TURN_RIGHT,
    RESULT_OBST_LEFT,
    RESULT_OBST_RIGHT,
    RESULT_OBST_FRONT,
    RESULT_CROSS_ROAD,
};


class RoadEdge: public LaneDetectorBase{
private:
    int kSize;
    void grayScale(Mat &src, Mat &dst);
    void regionOfInterest(Mat &img, const Point** pts, const int* npts);
    int findLines(Mat &img, const vector<Vec4f> &lines, const Scalar& color = kColorRed, int thickness = 6);
    int houghLines(const Mat &img, Mat &lineImage, int threshold, double minLineLen, double maxLineGap, bool retry = true);
    template <typename T> T getMean(vector<T> values);
    int validX(float x1,float x2);
    int validY(float y1, float y2);

public:
    RoadEdge(Mat rgbMat, int width, int height):LaneDetectorBase(rgbMat, width, height){
        kSize = 7;
    }
    virtual int processImage(Mat &img);
};


#endif //OPENCV_APPLICATION_ROADEDGE_H
